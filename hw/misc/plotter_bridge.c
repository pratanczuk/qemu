/*
 * plotter-bridge — SysBus device: file-backed shared memory + GPIO MMIO mirror.
 *
 * SysBus MMIO regions:
 *   0 — RAM: guest-visible copy of PlotterBridgeShm (mmap-friendly with
 *           property shm-path= on Linux).
 *   1 — GPIO: same struct fields via little-endian register access.
 *
 * GPIO map (all accesses little-endian; byte writes update subfields):
 *   0x00  motor_in guest view (low 24 bits; XOR gpio-invert-mask motor bits)
 *   0x04  encoder_ab guest view (same; XOR encoder bits of mask)
 *   0x08  sensors (low 3 bits), leds (bits 8–9: red, white)
 *   0x10  pulse_x, 0x14 pulse_y, 0x18 pulse_z (int32, LE; if max-*-pulses>0 clamp to [0,max])
 *   0x1c  vgpio_in (RW) — virtual port bits; mapped pins → motor_in (after invert)
 *   0x20  vgpio_out (RO) — encoder feedback on mapped enc_a/enc_b pins
 *   0x24  pwm_duty X/Y/Z (RW, low 24 bits LE; each byte 0–255 → duty %)
 * RAM region mirrors PlotterBridgeShm: motor/encoder bytes are internal (post-invert).
 *
 * Motor IN1/IN2 (per axis, bits 0/1): 10 = forward, 01 = reverse, 00/11 = stop.
 * A QEMU_CLOCK_VIRTUAL timer (property encoder-period-ns, default 100 µs) steps
 * quadrature encoder outputs and pulse_* counts using Gray code (00→10→11→01).
 * encoder-period-ns=0 disables emulation (manual encoder/pulse only).
 *
 * SysBus IRQ lines (for machine wiring): 0 = mech0, 1 = mech1, 2 = optical.
 * Level-sensitive from shm sensors byte; updated on sensor-poll-period-ns
 * (default 1 ms virtual) so host mmap writes are visible without guest MMIO.
 * sensor-poll-period-ns=0 disables IRQ polling.
 *
 * Machine: qdev_create/qdev_init_nofail("plotter-bridge", ...) then
 * sysbus_mmio_map(sbd, 0, phys_shm); sysbus_mmio_map(sbd, 1, phys_gpio).
 * Optional properties map-shm-at / map-gpio-at (default -1 = unmapped) call
 * sysbus_mmio_map from realize for QTest or simple bring-up.
 *
 * On Linux, shm-path is a regular file (e.g. truncate -s 28 /tmp/x.shm); the
 * device mmap(2)s it MAP_SHARED so host tools (visualizer) see the same bytes.
 *
 * plotter-config-file= merges JSON (see contrib/qemu-grbl-plugin/
 * plotter_bridge_config.example.json); only keys present override prior state
 * (CLI/device properties apply first). max-x-pulses, max-y-pulses, max-z-pulses,
 * gpio-invert-mask are also settable as properties (0 = unlimited for limits).
 *
 * When max-*-pulses > 0, that axis is modeled as a hard travel range [0, max]
 * (pulses clamp there). On each device reset, pulse_x / pulse_y / pulse_z are
 * initialized to independent uniform random values in [0, max] for each axis
 * with a limit (internal_x/y/z_pos mirror those values for host/debug).
 * The encoder timer stops stepping quadrature (holds 00) if the motor is
 * driven past a boundary—negative at 0 or positive at max—until direction
 * reverses (stall-style homing cue for firmware). Stall wins over PWM: at a
 * rail, encoder stays 00 even at 100% duty.
 *
 * PWM duty (0x24, RW): low 24 bits are duty bytes for X/Y/Z (0–255 → 0–100%).
 * Deadband and reset defaults come from JSON (pwm_deadband_percent,
 * pwm_duty_default) or property pwm-deadband-percent. Below deadband %,
 * effective drive is zero; above, linear map to normalized speed to 100%.
 * Direction comes from motor_in IN1/IN2. Velocity is low-pass filtered per tick.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/qdev.h"
#include "hw/irq.h"
#include "exec/memory.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include <string.h>
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "qapi/qmp/qdict.h"
#include "qapi/qmp/qjson.h"
#include "glib.h"
#ifdef __linux__
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#include "contrib/qemu-grbl-plugin/plotter_bridge_shm.h"
#include "contrib/qemu-grbl-plugin/plotter_bridge_config.h"

QEMU_BUILD_BUG_ON(sizeof(PlotterBridgeShm) != PLOTTER_BRIDGE_SHM_SIZE);

#define TYPE_PLOTTER_BRIDGE "plotter-bridge"
#define PLOTTER_BRIDGE(obj) OBJECT_CHECK(PlotterBridgeState, (obj), TYPE_PLOTTER_BRIDGE)

#define PLOTTER_BRIDGE_GPIO_SIZE 0x30

/* Default virtual-time interval between encoder updates (100 µs). */
#define PLOTTER_BRIDGE_DEFAULT_ENCODER_PERIOD_NS 100000ULL
/* Poll SHM sensors for IRQ lines (host mmap); 1 ms virtual. */
#define PLOTTER_BRIDGE_DEFAULT_SENSOR_POLL_NS 1000000ULL

#define PLOTTER_PWM_LINEAR_HI 100.0f
/* Low-pass on normalized speed 0…1 each encoder tick (higher = snappier). */
#define PLOTTER_PWM_INERTIA_ALPHA 0.35f
#define PLOTTER_PWM_VEL_EPSILON 1.0e-5f

typedef struct PlotterBridgeState {
    SysBusDevice parent_obj;

    MemoryRegion shm;
    MemoryRegion gpio;

    char *shm_path;
    uint64_t map_shm_at;
    uint64_t map_gpio_at;

    QEMUTimer *encoder_timer;
    uint64_t encoder_period_ns;
    uint8_t quad_phase[3];

    QEMUTimer *sensor_poll_timer;
    uint64_t sensor_poll_period_ns;
    qemu_irq sensor_irq[3];

    char *plotter_config_file;
    int32_t max_x_pulses;
    int32_t max_y_pulses;
    int32_t max_z_pulses;
    uint32_t gpio_invert_mask;
    uint8_t pin_in1[3];
    uint8_t pin_in2[3];
    uint8_t pin_enc_a[3];
    uint8_t pin_enc_b[3];
    uint32_t pwm_deadband_percent;
    uint8_t pwm_duty_default[3];
    uint32_t vgpio_in;

    /* Logical carriage positions; kept equal to pulse_* (see refresh helper). */
    int32_t internal_x_pos;
    int32_t internal_y_pos;
    int32_t internal_z_pos;

    uint8_t pwm_duty[3];
    float pwm_vel_smooth[3];
    float pwm_step_accum[3];

#ifdef __linux__
    /* If shm-path: host mmap; munmap in finalize (RAM is RAM_PREALLOC). */
    void *shm_host_mmap;
    size_t shm_host_mmap_len;
#endif
} PlotterBridgeState;

static PlotterBridgeShm *plotter_bridge_shm(PlotterBridgeState *s)
{
    return (PlotterBridgeShm *)memory_region_get_ram_ptr(&s->shm);
}

#ifdef __linux__
/*
 * Map a host file for PlotterBridgeShm so QEMU and external tools share memory.
 * The file must be at least PLOTTER_BRIDGE_SHM_SIZE bytes; extended to a full
 * host page if needed for mmap.
 */
static bool plotter_bridge_init_shm_host_file(PlotterBridgeState *s, Error **errp)
{
    int fd;
    struct stat st;
    size_t pgsz = (size_t)qemu_real_host_page_size;
    size_t maplen = ROUND_UP((size_t)PLOTTER_BRIDGE_SHM_SIZE, pgsz);
    void *p;

    fd = open(s->shm_path, O_RDWR);
    if (fd < 0) {
        error_setg_errno(errp, errno, "plotter-bridge: cannot open shm-path %s",
                         s->shm_path);
        return false;
    }
    if (fstat(fd, &st) != 0) {
        error_setg_errno(errp, errno, "plotter-bridge: cannot stat shm-path %s",
                         s->shm_path);
        close(fd);
        return false;
    }
    if (st.st_size < (off_t)PLOTTER_BRIDGE_SHM_SIZE) {
        error_setg(errp, "plotter-bridge: %s must be at least %u bytes (got %jd)",
                   s->shm_path, (unsigned)PLOTTER_BRIDGE_SHM_SIZE,
                   (intmax_t)st.st_size);
        close(fd);
        return false;
    }
    if ((uint64_t)st.st_size < maplen) {
        if (ftruncate(fd, (off_t)maplen) != 0) {
            error_setg_errno(errp, errno, "plotter-bridge: cannot extend %s",
                             s->shm_path);
            close(fd);
            return false;
        }
    }
    p = mmap(NULL, maplen, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) {
        error_setg_errno(errp, errno, "plotter-bridge: cannot mmap %s",
                         s->shm_path);
        close(fd);
        return false;
    }
    close(fd);

    memory_region_init_ram_ptr(&s->shm, OBJECT(s), "plotter-bridge-shm",
                               PLOTTER_BRIDGE_SHM_SIZE, p);
    s->shm_host_mmap = p;
    s->shm_host_mmap_len = maplen;
    return true;
}
#endif

static bool plotter_bridge_pin_ok(uint64_t v)
{
    return v <= 31 || v == PLOTTER_BRIDGE_PIN_UNMAPPED;
}

static uint8_t plotter_motor_internal_from_guest(uint8_t g2, unsigned axis,
                                                 uint32_t inv)
{
    return (uint8_t)((g2 & 3) ^ ((inv >> PLOTTER_BRIDGE_INVERT_MOTOR_SHIFT(axis)) & 3));
}

static uint8_t plotter_motor_guest_view(uint8_t internal2, unsigned axis,
                                        uint32_t inv)
{
    return (uint8_t)((internal2 & 3) ^ ((inv >> PLOTTER_BRIDGE_INVERT_MOTOR_SHIFT(axis)) & 3));
}

static uint8_t plotter_enc_internal_from_guest(uint8_t g2, unsigned axis,
                                               uint32_t inv)
{
    return (uint8_t)((g2 & 3) ^ ((inv >> PLOTTER_BRIDGE_INVERT_ENCODER_SHIFT(axis)) & 3));
}

static uint8_t plotter_enc_guest_view(uint8_t internal2, unsigned axis,
                                      uint32_t inv)
{
    return (uint8_t)((internal2 & 3) ^ ((inv >> PLOTTER_BRIDGE_INVERT_ENCODER_SHIFT(axis)) & 3));
}

static uint32_t plotter_motor_pack_guest(PlotterBridgeShm *shm, PlotterBridgeState *s)
{
    uint32_t w = 0;
    int i;

    for (i = 0; i < 3; i++) {
        uint8_t gv = plotter_motor_guest_view(shm->motor_in[i], i, s->gpio_invert_mask);
        w |= (uint32_t)gv << (i * 8);
    }
    return w;
}

static void plotter_motor_unpack_guest(PlotterBridgeState *s, PlotterBridgeShm *shm,
                                       uint32_t wgv)
{
    int i;

    for (i = 0; i < 3; i++) {
        uint8_t g = (uint8_t)((wgv >> (i * 8)) & 3);
        shm->motor_in[i] = plotter_motor_internal_from_guest(g, i, s->gpio_invert_mask);
    }
}

static uint32_t plotter_enc_pack_guest(PlotterBridgeShm *shm, PlotterBridgeState *s)
{
    uint32_t w = 0;
    int i;

    for (i = 0; i < 3; i++) {
        uint8_t gv = plotter_enc_guest_view(shm->encoder_ab[i], i, s->gpio_invert_mask);
        w |= (uint32_t)gv << (i * 8);
    }
    return w;
}

static void plotter_enc_unpack_guest(PlotterBridgeState *s, PlotterBridgeShm *shm,
                                     uint32_t wgv)
{
    int i;

    for (i = 0; i < 3; i++) {
        uint8_t g = (uint8_t)((wgv >> (i * 8)) & 3);
        shm->encoder_ab[i] = plotter_enc_internal_from_guest(g, i, s->gpio_invert_mask);
    }
}

static void plotter_bridge_sync_motors_from_vgpio(PlotterBridgeState *s)
{
    PlotterBridgeShm *shm = plotter_bridge_shm(s);
    unsigned ax;

    for (ax = 0; ax < 3; ax++) {
        uint8_t p1 = s->pin_in1[ax];
        uint8_t p2 = s->pin_in2[ax];
        uint8_t raw;

        if (p1 > 31 || p2 > 31) {
            continue;
        }
        raw = 0;
        if (extract32(s->vgpio_in, p1, 1)) {
            raw |= PLOTTER_LINE_A;
        }
        if (extract32(s->vgpio_in, p2, 1)) {
            raw |= PLOTTER_LINE_B;
        }
        shm->motor_in[ax] = plotter_motor_internal_from_guest(raw, ax,
                                                                s->gpio_invert_mask);
    }
}

static uint32_t plotter_build_vgpio_out(PlotterBridgeState *s, PlotterBridgeShm *shm)
{
    uint32_t out = 0;
    unsigned ax;

    for (ax = 0; ax < 3; ax++) {
        uint8_t gv = plotter_enc_guest_view(shm->encoder_ab[ax], ax, s->gpio_invert_mask);

        if (s->pin_enc_a[ax] <= 31) {
            out = deposit32(out, s->pin_enc_a[ax], 1,
                            (gv & PLOTTER_LINE_A) ? 1u : 0u);
        }
        if (s->pin_enc_b[ax] <= 31) {
            out = deposit32(out, s->pin_enc_b[ax], 1,
                            (gv & PLOTTER_LINE_B) ? 1u : 0u);
        }
    }
    return out;
}

static void plotter_bridge_refresh_internal_pos(PlotterBridgeState *s,
                                                PlotterBridgeShm *shm)
{
    s->internal_x_pos = (int32_t)ldl_le_p((uint8_t *)&shm->pulse_x);
    s->internal_y_pos = (int32_t)ldl_le_p((uint8_t *)&shm->pulse_y);
    s->internal_z_pos = (int32_t)ldl_le_p((uint8_t *)&shm->pulse_z);
}

static int32_t plotter_axis_max_limit(const PlotterBridgeState *s, int axis)
{
    switch (axis) {
    case 0:
        return s->max_x_pulses;
    case 1:
        return s->max_y_pulses;
    case 2:
        return s->max_z_pulses;
    default:
        return 0;
    }
}

static int32_t plotter_axis_pulse_read(const PlotterBridgeShm *shm, int axis)
{
    switch (axis) {
    case 0:
        return (int32_t)ldl_le_p((uint8_t *)&shm->pulse_x);
    case 1:
        return (int32_t)ldl_le_p((uint8_t *)&shm->pulse_y);
    case 2:
        return (int32_t)ldl_le_p((uint8_t *)&shm->pulse_z);
    default:
        return 0;
    }
}

/*
 * When maxlim > 0, position is in [0, maxlim]. Encoder steps only if the
 * motor direction moves away from a hard stop (stall if driven into the rail).
 */
static bool plotter_encoder_step_allowed(int32_t pos, int32_t maxlim, int dir)
{
    if (dir == 0) {
        return false;
    }
    if (maxlim <= 0) {
        return true;
    }
    if (pos <= 0 && dir < 0) {
        return false;
    }
    if (pos >= maxlim && dir > 0) {
        return false;
    }
    return true;
}

static void plotter_pulse_clamp(PlotterBridgeState *s, PlotterBridgeShm *shm, int axis)
{
    int32_t lim = 0;
    int32_t v;

    switch (axis) {
    case 0:
        lim = s->max_x_pulses;
        v = (int32_t)ldl_le_p((uint8_t *)&shm->pulse_x);
        break;
    case 1:
        lim = s->max_y_pulses;
        v = (int32_t)ldl_le_p((uint8_t *)&shm->pulse_y);
        break;
    case 2:
        lim = s->max_z_pulses;
        v = (int32_t)ldl_le_p((uint8_t *)&shm->pulse_z);
        break;
    default:
        return;
    }
    if (lim <= 0) {
        return;
    }
    /* Bounded axis: physical segment [0, lim]. */
    if (v > lim) {
        v = lim;
    }
    if (v < 0) {
        v = 0;
    }
    switch (axis) {
    case 0:
        stl_le_p((uint8_t *)&shm->pulse_x, (uint32_t)v);
        break;
    case 1:
        stl_le_p((uint8_t *)&shm->pulse_y, (uint32_t)v);
        break;
    case 2:
        stl_le_p((uint8_t *)&shm->pulse_z, (uint32_t)v);
        break;
    default:
        break;
    }
    plotter_bridge_refresh_internal_pos(s, shm);
}

static void plotter_bridge_randomize_bounded_positions(PlotterBridgeState *s,
                                                       PlotterBridgeShm *shm)
{
    int32_t px = 0;
    int32_t py = 0;
    int32_t pz = 0;

    if (s->max_x_pulses > 0) {
        px = g_random_int_range(0, s->max_x_pulses + 1);
    }
    if (s->max_y_pulses > 0) {
        py = g_random_int_range(0, s->max_y_pulses + 1);
    }
    if (s->max_z_pulses > 0) {
        pz = g_random_int_range(0, s->max_z_pulses + 1);
    }
    stl_le_p((uint8_t *)&shm->pulse_x, (uint32_t)px);
    stl_le_p((uint8_t *)&shm->pulse_y, (uint32_t)py);
    stl_le_p((uint8_t *)&shm->pulse_z, (uint32_t)pz);
    plotter_bridge_refresh_internal_pos(s, shm);
}

static bool plotter_bridge_apply_axis_from_qdict(QDict *ax, uint8_t *pin_in1,
                                                 uint8_t *pin_in2, uint8_t *pin_enc_a,
                                                 uint8_t *pin_enc_b, Error **errp)
{
    if (qdict_haskey(ax, "in1")) {
        int64_t v = qdict_get_int(ax, "in1");

        if (!plotter_bridge_pin_ok((uint64_t)v)) {
            error_setg(errp, "plotter-bridge: gpio_map in1 must be 0-31 or 255");
            return false;
        }
        *pin_in1 = (uint8_t)v;
    }
    if (qdict_haskey(ax, "in2")) {
        int64_t v = qdict_get_int(ax, "in2");

        if (!plotter_bridge_pin_ok((uint64_t)v)) {
            error_setg(errp, "plotter-bridge: gpio_map in2 must be 0-31 or 255");
            return false;
        }
        *pin_in2 = (uint8_t)v;
    }
    if (qdict_haskey(ax, "enc_a")) {
        int64_t v = qdict_get_int(ax, "enc_a");

        if (!plotter_bridge_pin_ok((uint64_t)v)) {
            error_setg(errp, "plotter-bridge: gpio_map enc_a must be 0-31 or 255");
            return false;
        }
        *pin_enc_a = (uint8_t)v;
    }
    if (qdict_haskey(ax, "enc_b")) {
        int64_t v = qdict_get_int(ax, "enc_b");

        if (!plotter_bridge_pin_ok((uint64_t)v)) {
            error_setg(errp, "plotter-bridge: gpio_map enc_b must be 0-31 or 255");
            return false;
        }
        *pin_enc_b = (uint8_t)v;
    }
    return true;
}

static bool plotter_bridge_apply_pwm_duty_defaults_from_qdict(QDict *pd, uint8_t *def3,
                                                              Error **errp)
{
    static const char *axis_keys[] = { "x", "y", "z" };
    int ax;

    for (ax = 0; ax < 3; ax++) {
        const char *k = axis_keys[ax];

        if (!qdict_haskey(pd, k)) {
            continue;
        }
        int64_t v = qdict_get_int(pd, k);

        if (v < 0 || v > 255) {
            error_setg(errp, "plotter-bridge: pwm_duty_default.%s must be 0-255", k);
            return false;
        }
        def3[ax] = (uint8_t)v;
    }
    return true;
}

static bool plotter_bridge_config_merge_qdict(PlotterBridgeState *s, QDict *root,
                                              Error **errp)
{
    static const char *axis_keys[] = { "x", "y", "z" };

    if (qdict_haskey(root, "max_x_pulses")) {
        s->max_x_pulses = (int32_t)qdict_get_int(root, "max_x_pulses");
    }
    if (qdict_haskey(root, "max_z_pulses")) {
        s->max_z_pulses = (int32_t)qdict_get_int(root, "max_z_pulses");
    }
    if (qdict_haskey(root, "max_y_pulses")) {
        s->max_y_pulses = (int32_t)qdict_get_int(root, "max_y_pulses");
    } else if (qdict_haskey(root, "y_length")) {
        s->max_y_pulses = (int32_t)qdict_get_int(root, "y_length");
    }
    if (qdict_haskey(root, "gpio_invert_mask")) {
        s->gpio_invert_mask = (uint32_t)qdict_get_int(root, "gpio_invert_mask");
    }
    if (qdict_haskey(root, "gpio_map")) {
        QObject *gm_o = qdict_get(root, "gpio_map");
        QDict *gm = qobject_to_qdict(gm_o);
        int ax;

        if (!gm) {
            error_setg(errp, "plotter-bridge: gpio_map must be a JSON object");
            return false;
        }
        for (ax = 0; ax < 3; ax++) {
            const char *k = axis_keys[ax];

            if (!qdict_haskey(gm, k)) {
                continue;
            }
            {
                QDict *ad = qobject_to_qdict(qdict_get(gm, k));

                if (!ad) {
                    error_setg(errp, "plotter-bridge: gpio_map.%s must be an object", k);
                    return false;
                }
                if (!plotter_bridge_apply_axis_from_qdict(ad, &s->pin_in1[ax],
                                                          &s->pin_in2[ax],
                                                          &s->pin_enc_a[ax],
                                                          &s->pin_enc_b[ax],
                                                          errp)) {
                    return false;
                }
            }
        }
    }
    if (qdict_haskey(root, "pwm_deadband_percent")) {
        int64_t v = qdict_get_int(root, "pwm_deadband_percent");

        if (v < 0 || v > 100) {
            error_setg(errp, "plotter-bridge: pwm_deadband_percent must be 0-100");
            return false;
        }
        s->pwm_deadband_percent = (uint32_t)v;
    }
    if (qdict_haskey(root, "pwm_duty_default")) {
        QDict *pd = qobject_to_qdict(qdict_get(root, "pwm_duty_default"));

        if (!pd) {
            error_setg(errp, "plotter-bridge: pwm_duty_default must be a JSON object");
            return false;
        }
        if (!plotter_bridge_apply_pwm_duty_defaults_from_qdict(pd, s->pwm_duty_default,
                                                               errp)) {
            return false;
        }
    }
    return true;
}

static bool plotter_bridge_config_load_file(PlotterBridgeState *s, Error **errp)
{
    const char *path = s->plotter_config_file;
    gchar *contents = NULL;
    gsize len = 0;
    GError *gerr = NULL;
    QObject *obj;
    QDict *root;
    bool ok;

    if (!path || !path[0]) {
        return true;
    }
    if (!g_file_get_contents(path, &contents, &len, &gerr)) {
        error_setg(errp, "plotter-bridge: cannot read plotter-config-file %s: %s",
                   path, gerr->message);
        g_error_free(gerr);
        return false;
    }
    obj = qobject_from_json(contents);
    g_free(contents);
    if (!obj) {
        error_setg(errp, "plotter-bridge: invalid JSON in %s", path);
        return false;
    }
    root = qobject_to_qdict(obj);
    if (!root) {
        qobject_decref(obj);
        error_setg(errp, "plotter-bridge: config root must be a JSON object");
        return false;
    }
    ok = plotter_bridge_config_merge_qdict(s, root, errp);
    qobject_decref(obj);
    return ok;
}

/*
 * Quadrature Gray sequence (A,B): 00 → 10 → 11 → 01 → 00 (forward).
 * PLOTTER_LINE_A = bit0 (phase A), PLOTTER_LINE_B = bit1 (phase B).
 */
static uint8_t plotter_quad_phase_to_ab(unsigned phase)
{
    static const uint8_t gray_ab[4] = {
        0,
        PLOTTER_LINE_A,
        PLOTTER_LINE_A | PLOTTER_LINE_B,
        PLOTTER_LINE_B,
    };

    return gray_ab[phase & 3];
}

/* IN1 = bit0 (LINE_A), IN2 = bit1 (LINE_B). */
static int plotter_motor_direction(uint8_t in01)
{
    switch (in01 & 3) {
    case PLOTTER_LINE_A:              /* IN1=1 IN2=0 */
        return 1;
    case PLOTTER_LINE_B:              /* IN1=0 IN2=1 */
        return -1;
    default:                          /* 00 or 11: brake / stop */
        return 0;
    }
}

static float plotter_pwm_duty_percent(uint8_t raw)
{
    return (float)raw * (100.0f / 255.0f);
}

/*
 * Map duty % to normalized speed 0…1: below db_pct → 0; linear from db_pct to 100%.
 */
static float plotter_pwm_speed_from_duty_percent(float pct, float db_pct)
{
    if (db_pct < 0.0f) {
        db_pct = 0.0f;
    }
    if (db_pct >= 99.0f) {
        db_pct = 99.0f;
    }
    if (pct < db_pct) {
        return 0.0f;
    }
    if (pct >= PLOTTER_PWM_LINEAR_HI) {
        return 1.0f;
    }
    return (pct - db_pct) / (PLOTTER_PWM_LINEAR_HI - db_pct);
}

static void plotter_bridge_pwm_reset_state(PlotterBridgeState *s)
{
    float db = (float)s->pwm_deadband_percent;
    int i;

    for (i = 0; i < 3; i++) {
        uint8_t raw = s->pwm_duty_default[i];
        float pct = plotter_pwm_duty_percent(raw);
        float vt = plotter_pwm_speed_from_duty_percent(pct, db);

        s->pwm_duty[i] = raw;
        s->pwm_vel_smooth[i] = vt;
        s->pwm_step_accum[i] = 0.0f;
    }
}

static void plotter_pulse_add(PlotterBridgeState *s, PlotterBridgeShm *shm,
                              int axis, int delta)
{
    int32_t v;

    switch (axis) {
    case 0:
        v = (int32_t)ldl_le_p((uint8_t *)&shm->pulse_x) + delta;
        stl_le_p((uint8_t *)&shm->pulse_x, (uint32_t)v);
        break;
    case 1:
        v = (int32_t)ldl_le_p((uint8_t *)&shm->pulse_y) + delta;
        stl_le_p((uint8_t *)&shm->pulse_y, (uint32_t)v);
        break;
    case 2:
        v = (int32_t)ldl_le_p((uint8_t *)&shm->pulse_z) + delta;
        stl_le_p((uint8_t *)&shm->pulse_z, (uint32_t)v);
        break;
    default:
        return;
    }
    plotter_pulse_clamp(s, shm, axis);
    plotter_bridge_refresh_internal_pos(s, shm);
}

static void plotter_bridge_encoder_arm(PlotterBridgeState *s)
{
    int64_t now;
    int64_t period;

    if (!s->encoder_timer || s->encoder_period_ns == 0) {
        return;
    }
    period = (int64_t)s->encoder_period_ns;
    if (period < 1) {
        period = 1;
    }
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    timer_mod_ns(s->encoder_timer, now + period);
}

static void plotter_bridge_encoder_tick(void *opaque)
{
    PlotterBridgeState *s = opaque;
    PlotterBridgeShm *shm = plotter_bridge_shm(s);
    int axis;

    for (axis = 0; axis < 3; axis++) {
        int dir = plotter_motor_direction(shm->motor_in[axis]);
        int32_t maxlim = plotter_axis_max_limit(s, axis);
        int32_t pos;
        float pct = plotter_pwm_duty_percent(s->pwm_duty[axis]);
        float v_target = plotter_pwm_speed_from_duty_percent(
            pct, (float)s->pwm_deadband_percent);
        float vs;

        if (dir == 0) {
            v_target = 0.0f;
        }

        s->pwm_vel_smooth[axis] = PLOTTER_PWM_INERTIA_ALPHA * v_target
            + (1.0f - PLOTTER_PWM_INERTIA_ALPHA) * s->pwm_vel_smooth[axis];
        vs = s->pwm_vel_smooth[axis];

        pos = plotter_axis_pulse_read(shm, axis);
        if (!plotter_encoder_step_allowed(pos, maxlim, dir) && dir != 0) {
            /* Rail stall: encoder frozen (00) regardless of PWM / inertia. */
            shm->encoder_ab[axis] = 0;
            s->pwm_step_accum[axis] = 0.0f;
            continue;
        }

        if (dir == 0 || vs < PLOTTER_PWM_VEL_EPSILON) {
            /* Brake, deadband, or decayed inertia — no quadrature advance. */
            shm->encoder_ab[axis] = plotter_quad_phase_to_ab(s->quad_phase[axis]);
            if (dir == 0) {
                s->pwm_step_accum[axis] = 0.0f;
            }
            continue;
        }

        s->pwm_step_accum[axis] += vs * (float)dir;

        if (dir > 0) {
            while (s->pwm_step_accum[axis] >= 1.0f) {
                pos = plotter_axis_pulse_read(shm, axis);
                if (!plotter_encoder_step_allowed(pos, maxlim, 1)) {
                    break;
                }
                s->pwm_step_accum[axis] -= 1.0f;
                s->quad_phase[axis] = (uint8_t)((s->quad_phase[axis] + 1) & 3);
                plotter_pulse_add(s, shm, axis, 1);
            }
        } else {
            while (s->pwm_step_accum[axis] <= -1.0f) {
                pos = plotter_axis_pulse_read(shm, axis);
                if (!plotter_encoder_step_allowed(pos, maxlim, -1)) {
                    break;
                }
                s->pwm_step_accum[axis] += 1.0f;
                s->quad_phase[axis] = (uint8_t)((s->quad_phase[axis] + 3) & 3);
                plotter_pulse_add(s, shm, axis, -1);
            }
        }

        shm->encoder_ab[axis] = plotter_quad_phase_to_ab(s->quad_phase[axis]);
    }

    plotter_bridge_encoder_arm(s);
}

static void plotter_bridge_sensor_poll_arm(PlotterBridgeState *s)
{
    int64_t now;
    int64_t period;

    if (!s->sensor_poll_timer || s->sensor_poll_period_ns == 0) {
        return;
    }
    period = (int64_t)s->sensor_poll_period_ns;
    if (period < 1) {
        period = 1;
    }
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    timer_mod_ns(s->sensor_poll_timer, now + period);
}

static void plotter_bridge_sensor_poll_tick(void *opaque)
{
    PlotterBridgeState *s = opaque;
    PlotterBridgeShm *shm = plotter_bridge_shm(s);
    uint8_t sens = shm->sensors & 0x07;

    qemu_set_irq(s->sensor_irq[0], !!(sens & PLOTTER_SENSOR_MECH0));
    qemu_set_irq(s->sensor_irq[1], !!(sens & PLOTTER_SENSOR_MECH1));
    qemu_set_irq(s->sensor_irq[2], !!(sens & PLOTTER_SENSOR_OPTICAL));

    plotter_bridge_sensor_poll_arm(s);
}

static void plotter_bridge_init_shm(PlotterBridgeState *s)
{
    PlotterBridgeShm *shm = plotter_bridge_shm(s);

    memset(shm, 0, PLOTTER_BRIDGE_SHM_SIZE);
    stl_le_p((uint8_t *)&shm->magic, PLOTTER_BRIDGE_MAGIC);
    stl_le_p((uint8_t *)&shm->version, PLOTTER_BRIDGE_VERSION);
    plotter_bridge_refresh_internal_pos(s, shm);
}

static uint64_t plotter_bridge_gpio_read(void *opaque, hwaddr addr, unsigned size)
{
    PlotterBridgeState *s = opaque;
    PlotterBridgeShm *shm = plotter_bridge_shm(s);
    uint32_t word = 0;

    if (addr + size > PLOTTER_BRIDGE_GPIO_SIZE) {
        return 0;
    }

    switch (addr & ~3) {
    case 0x00:
        word = plotter_motor_pack_guest(shm, s);
        break;
    case 0x04:
        word = plotter_enc_pack_guest(shm, s);
        break;
    case 0x08:
        word = ((uint32_t)(shm->sensors & 0x07) << 0) |
               ((uint32_t)(shm->leds & 0x03) << 8);
        break;
    case 0x0c:
        /* reserved / padding to align pulse block */
        word = 0;
        break;
    case 0x10:
        word = ldl_le_p((uint8_t *)&shm->pulse_x);
        break;
    case 0x14:
        word = ldl_le_p((uint8_t *)&shm->pulse_y);
        break;
    case 0x18:
        word = ldl_le_p((uint8_t *)&shm->pulse_z);
        break;
    case 0x1c:
        word = s->vgpio_in;
        break;
    case 0x20:
        word = plotter_build_vgpio_out(s, shm);
        break;
    case 0x24:
        word = (uint32_t)s->pwm_duty[0] | ((uint32_t)s->pwm_duty[1] << 8)
            | ((uint32_t)s->pwm_duty[2] << 16);
        break;
    default:
        return 0;
    }

    return extract32(word, (addr & 3) * 8, size * 8);
}

static void plotter_bridge_gpio_write(void *opaque, hwaddr addr, uint64_t data,
                                      unsigned size)
{
    PlotterBridgeState *s = opaque;
    PlotterBridgeShm *shm = plotter_bridge_shm(s);
    hwaddr waddr = addr & ~3;
    uint32_t cur = 0;
    uint32_t val;
    uint32_t nval;

    if (addr + size > PLOTTER_BRIDGE_GPIO_SIZE) {
        return;
    }

    switch (waddr) {
    case 0x00:
        cur = plotter_motor_pack_guest(shm, s);
        val = (uint32_t)data;
        nval = deposit32(cur, (addr & 3) * 8, size * 8, val);
        plotter_motor_unpack_guest(s, shm, nval);
        break;
    case 0x04:
        cur = plotter_enc_pack_guest(shm, s);
        val = (uint32_t)data;
        nval = deposit32(cur, (addr & 3) * 8, size * 8, val);
        plotter_enc_unpack_guest(s, shm, nval);
        break;
    case 0x08:
        cur = ((uint32_t)shm->sensors << 0) | ((uint32_t)shm->leds << 8);
        val = (uint32_t)data;
        nval = deposit32(cur, (addr & 3) * 8, size * 8, val);
        shm->sensors = (uint8_t)(nval & 0x07);
        shm->leds = (uint8_t)((nval >> 8) & 0x03);
        break;
    case 0x10:
        cur = ldl_le_p((uint8_t *)&shm->pulse_x);
        val = (uint32_t)deposit32(cur, (addr & 3) * 8, size * 8, (uint32_t)data);
        stl_le_p((uint8_t *)&shm->pulse_x, val);
        plotter_pulse_clamp(s, shm, 0);
        plotter_bridge_refresh_internal_pos(s, shm);
        break;
    case 0x14:
        cur = ldl_le_p((uint8_t *)&shm->pulse_y);
        val = (uint32_t)deposit32(cur, (addr & 3) * 8, size * 8, (uint32_t)data);
        stl_le_p((uint8_t *)&shm->pulse_y, val);
        plotter_pulse_clamp(s, shm, 1);
        plotter_bridge_refresh_internal_pos(s, shm);
        break;
    case 0x18:
        cur = ldl_le_p((uint8_t *)&shm->pulse_z);
        val = (uint32_t)deposit32(cur, (addr & 3) * 8, size * 8, (uint32_t)data);
        stl_le_p((uint8_t *)&shm->pulse_z, val);
        plotter_pulse_clamp(s, shm, 2);
        plotter_bridge_refresh_internal_pos(s, shm);
        break;
    case 0x1c:
        cur = s->vgpio_in;
        val = (uint32_t)data;
        nval = deposit32(cur, (addr & 3) * 8, size * 8, val);
        s->vgpio_in = nval;
        plotter_bridge_sync_motors_from_vgpio(s);
        break;
    case 0x20:
        break;
    case 0x24:
        cur = (uint32_t)s->pwm_duty[0] | ((uint32_t)s->pwm_duty[1] << 8)
            | ((uint32_t)s->pwm_duty[2] << 16);
        val = (uint32_t)data;
        nval = deposit32(cur, (addr & 3) * 8, size * 8, val);
        s->pwm_duty[0] = (uint8_t)(nval & 0xff);
        s->pwm_duty[1] = (uint8_t)((nval >> 8) & 0xff);
        s->pwm_duty[2] = (uint8_t)((nval >> 16) & 0xff);
        shm->pwm_duty[0] = s->pwm_duty[0];
        shm->pwm_duty[1] = s->pwm_duty[1];
        shm->pwm_duty[2] = s->pwm_duty[2];
        break;
    default:
        break;
    }
}

static const MemoryRegionOps plotter_bridge_gpio_ops = {
    .read = plotter_bridge_gpio_read,
    .write = plotter_bridge_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void plotter_bridge_reset(DeviceState *dev)
{
    PlotterBridgeState *s = PLOTTER_BRIDGE(dev);
    PlotterBridgeShm *shm = plotter_bridge_shm(s);

    /*
     * Do not clear magic/version: a file-backed region may be mmap'd by a
     * host tool across guest resets.
     */
    memset(shm->motor_in, 0, sizeof(shm->motor_in));
    memset(shm->encoder_ab, 0, sizeof(shm->encoder_ab));
    shm->sensors = 0;
    shm->leds = 0;
    plotter_bridge_randomize_bounded_positions(s, shm);

    s->quad_phase[0] = s->quad_phase[1] = s->quad_phase[2] = 0;
    s->vgpio_in = 0;
    plotter_bridge_pwm_reset_state(s);
    shm->pwm_duty[0] = s->pwm_duty[0];
    shm->pwm_duty[1] = s->pwm_duty[1];
    shm->pwm_duty[2] = s->pwm_duty[2];

    qemu_set_irq(s->sensor_irq[0], 0);
    qemu_set_irq(s->sensor_irq[1], 0);
    qemu_set_irq(s->sensor_irq[2], 0);
}

static void plotter_bridge_realize(DeviceState *dev, Error **errp)
{
    PlotterBridgeState *s = PLOTTER_BRIDGE(dev);
    Error *local_err = NULL;

#ifdef __linux__
    if (s->shm_path && s->shm_path[0] != '\0') {
        if (!plotter_bridge_init_shm_host_file(s, errp)) {
            return;
        }
    } else {
        memory_region_init_ram(&s->shm, OBJECT(s), "plotter-bridge-shm",
                               PLOTTER_BRIDGE_SHM_SIZE, &local_err);
        if (local_err) {
            error_propagate(errp, local_err);
            return;
        }
    }
#else
    if (s->shm_path && s->shm_path[0] != '\0') {
        error_setg(errp, "plotter-bridge: shm-path is only supported on Linux");
        return;
    }
    memory_region_init_ram(&s->shm, OBJECT(s), "plotter-bridge-shm",
                           PLOTTER_BRIDGE_SHM_SIZE, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }
#endif

    if (!plotter_bridge_config_load_file(s, errp)) {
        return;
    }

    memory_region_init_io(&s->gpio, OBJECT(s), &plotter_bridge_gpio_ops, s,
                          "plotter-bridge-gpio", PLOTTER_BRIDGE_GPIO_SIZE);

    plotter_bridge_init_shm(s);

    if (s->map_shm_at != UINT64_MAX) {
        sysbus_mmio_map(SYS_BUS_DEVICE(s), 0, (hwaddr)s->map_shm_at);
    }
    if (s->map_gpio_at != UINT64_MAX) {
        sysbus_mmio_map(SYS_BUS_DEVICE(s), 1, (hwaddr)s->map_gpio_at);
    }

    if (s->encoder_period_ns > 0) {
        s->encoder_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                        plotter_bridge_encoder_tick, s);
        plotter_bridge_encoder_arm(s);
    }

    if (s->sensor_poll_period_ns > 0) {
        s->sensor_poll_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                            plotter_bridge_sensor_poll_tick, s);
        plotter_bridge_sensor_poll_arm(s);
    }
}

static void plotter_bridge_finalize(Object *obj)
{
    PlotterBridgeState *s = PLOTTER_BRIDGE(obj);

#ifdef __linux__
    if (s->shm_host_mmap) {
        munmap(s->shm_host_mmap, s->shm_host_mmap_len);
        s->shm_host_mmap = NULL;
        s->shm_host_mmap_len = 0;
    }
#endif

    g_free(s->plotter_config_file);
    s->plotter_config_file = NULL;

    if (s->encoder_timer) {
        timer_free(s->encoder_timer);
        s->encoder_timer = NULL;
    }
    if (s->sensor_poll_timer) {
        timer_free(s->sensor_poll_timer);
        s->sensor_poll_timer = NULL;
    }
}

static void plotter_bridge_init(Object *obj)
{
    PlotterBridgeState *s = PLOTTER_BRIDGE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    PlotterBridgeConfig dflt;

    plotter_bridge_config_defaults(&dflt);
    s->max_x_pulses = dflt.max_x_pulses;
    s->max_y_pulses = dflt.max_y_pulses;
    s->max_z_pulses = dflt.max_z_pulses;
    s->gpio_invert_mask = dflt.gpio_invert_mask;
    memcpy(s->pin_in1, dflt.pin_in1, sizeof(s->pin_in1));
    memcpy(s->pin_in2, dflt.pin_in2, sizeof(s->pin_in2));
    memcpy(s->pin_enc_a, dflt.pin_enc_a, sizeof(s->pin_enc_a));
    memcpy(s->pin_enc_b, dflt.pin_enc_b, sizeof(s->pin_enc_b));
    s->pwm_deadband_percent = dflt.pwm_deadband_percent;
    memcpy(s->pwm_duty_default, dflt.pwm_duty_default, sizeof(s->pwm_duty_default));
#ifdef __linux__
    s->shm_host_mmap = NULL;
    s->shm_host_mmap_len = 0;
#endif
    s->vgpio_in = 0;
    s->internal_x_pos = 0;
    s->internal_y_pos = 0;
    s->internal_z_pos = 0;
    plotter_bridge_pwm_reset_state(s);

    sysbus_init_mmio(sbd, &s->shm);
    sysbus_init_mmio(sbd, &s->gpio);
    sysbus_init_irq(sbd, &s->sensor_irq[0]);
    sysbus_init_irq(sbd, &s->sensor_irq[1]);
    sysbus_init_irq(sbd, &s->sensor_irq[2]);
}

static Property plotter_bridge_properties[] = {
    DEFINE_PROP_STRING("shm-path", PlotterBridgeState, shm_path),
    DEFINE_PROP_UINT64("map-shm-at", PlotterBridgeState, map_shm_at, UINT64_MAX),
    DEFINE_PROP_UINT64("map-gpio-at", PlotterBridgeState, map_gpio_at, UINT64_MAX),
    DEFINE_PROP_UINT64("encoder-period-ns", PlotterBridgeState, encoder_period_ns,
                       PLOTTER_BRIDGE_DEFAULT_ENCODER_PERIOD_NS),
    DEFINE_PROP_UINT64("sensor-poll-period-ns", PlotterBridgeState,
                       sensor_poll_period_ns, PLOTTER_BRIDGE_DEFAULT_SENSOR_POLL_NS),
    DEFINE_PROP_STRING("plotter-config-file", PlotterBridgeState, plotter_config_file),
    DEFINE_PROP_INT32("max-x-pulses", PlotterBridgeState, max_x_pulses, 0),
    DEFINE_PROP_INT32("max-y-pulses", PlotterBridgeState, max_y_pulses, 0),
    DEFINE_PROP_INT32("max-z-pulses", PlotterBridgeState, max_z_pulses, 0),
    DEFINE_PROP_UINT32("gpio-invert-mask", PlotterBridgeState, gpio_invert_mask, 0),
    DEFINE_PROP_UINT32("pwm-deadband-percent", PlotterBridgeState, pwm_deadband_percent,
                       40),
    DEFINE_PROP_END_OF_LIST(),
};

static void plotter_bridge_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = plotter_bridge_realize;
    dc->reset = plotter_bridge_reset;
    dc->props = plotter_bridge_properties;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo plotter_bridge_info = {
    .name          = TYPE_PLOTTER_BRIDGE,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PlotterBridgeState),
    .instance_init = plotter_bridge_init,
    .instance_finalize = plotter_bridge_finalize,
    .class_init    = plotter_bridge_class_init,
};

static void plotter_bridge_register_types(void)
{
    type_register_static(&plotter_bridge_info);
}

type_init(plotter_bridge_register_types)
