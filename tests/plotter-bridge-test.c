/*
 * QTest for plotter-bridge (SysBus shared memory + GPIO MMIO).
 *
 * Under qtest, `memory_region_init_ram_from_file` (shm-path) is not usable with
 * this QEMU’s RAM backend, so this suite exercises anonymous RAM only.
 *
 * The bridge is attached from mips_mipssim when qtest is active (see
 * hw/mips/mips_mipssim.c) at SHM_PHYS / GPIO_PHYS — MIPS softmmu machines
 * reject generic `-device plotter-bridge` on the command line.
 *
 * Optional: set QTEST_PLOTTER_CONFIG to a JSON file path (same schema as
 * plotter-config-file=) to exercise gpio_map, limits, and vgpio MMIO.
 * Headers: contrib/qemu-grbl-plugin/ (submodule).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <glib.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include "libqtest.h"
#include "contrib/qemu-grbl-plugin/plotter_bridge_shm.h"

/* Unmapped hole before mips_mipssim BIOS at 0x1fc00000 */
#define SHM_PHYS  0x1f800000ULL
#define GPIO_PHYS 0x1f800040ULL

static void unset_plotter_config_env(void)
{
    unsetenv("QTEST_PLOTTER_CONFIG");
}

static gchar *write_temp_plotter_json(const char *json_body)
{
    gchar *path = NULL;
    GError *err = NULL;
    int fd = g_file_open_tmp("pbXXXXXX.json", &path, &err);
    ssize_t w;

    g_assert_no_error(err);
    g_assert_cmpint(fd, >=, 0);
    w = write(fd, json_body, strlen(json_body));
    g_assert_cmpint((int)w, ==, (int)strlen(json_body));
    close(fd);
    return path;
}

static char *make_cmdline(void)
{
    return g_strdup("-machine mipssim -m 64M");
}

static void test_magic_header(void)
{
    uint8_t buf[8];
    char *cmd = make_cmdline();

    unset_plotter_config_env();
    qtest_start(cmd);
    g_free(cmd);

    qtest_memread(global_qtest, SHM_PHYS, buf, sizeof(buf));
    g_assert_cmpuint(buf[0], ==, (PLOTTER_BRIDGE_MAGIC >> 0) & 0xff);
    g_assert_cmpuint(buf[1], ==, (PLOTTER_BRIDGE_MAGIC >> 8) & 0xff);
    g_assert_cmpuint(buf[2], ==, (PLOTTER_BRIDGE_MAGIC >> 16) & 0xff);
    g_assert_cmpuint(buf[3], ==, (PLOTTER_BRIDGE_MAGIC >> 24) & 0xff);
    g_assert_cmpuint(buf[4], ==, (PLOTTER_BRIDGE_VERSION >> 0) & 0xff);
    g_assert_cmpuint(buf[5], ==, (PLOTTER_BRIDGE_VERSION >> 8) & 0xff);
    g_assert_cmpuint(buf[6], ==, (PLOTTER_BRIDGE_VERSION >> 16) & 0xff);
    g_assert_cmpuint(buf[7], ==, (PLOTTER_BRIDGE_VERSION >> 24) & 0xff);

    qtest_end();
}

static void test_gpio_updates_shm_motors(void)
{
    uint8_t m[3];
    char *cmd = make_cmdline();

    unset_plotter_config_env();
    qtest_start(cmd);
    g_free(cmd);

    /* Little-endian packing: motor_in[0]=1, [1]=2, [2]=3 (2 bits each, masked) */
    writel(GPIO_PHYS + 0x00, 0x00030201);

    qtest_memread(global_qtest, SHM_PHYS + 8, m, 3);
    g_assert_cmpuint(m[0], ==, 1);
    g_assert_cmpuint(m[1], ==, 2);
    g_assert_cmpuint(m[2], ==, 3);

    g_assert_cmpuint(readl(GPIO_PHYS + 0x00), ==, 0x00030201);

    qtest_end();
}

static void test_shm_ram_visible_on_gpio(void)
{
    uint8_t blob[3] = { 0x2, 0x1, 0x3 };
    char *cmd = make_cmdline();

    unset_plotter_config_env();
    qtest_start(cmd);
    g_free(cmd);

    qtest_memwrite(global_qtest, SHM_PHYS + 8, blob, 3);
    g_assert_cmpuint(readl(GPIO_PHYS + 0x00), ==, 0x00030102);

    qtest_end();
}

static void test_encoder_sensors_leds_pulses(void)
{
    char *cmd = make_cmdline();

    unset_plotter_config_env();
    qtest_start(cmd);
    g_free(cmd);

    writel(GPIO_PHYS + 0x04, 0x00030201);
    uint8_t enc[3];
    qtest_memread(global_qtest, SHM_PHYS + 11, enc, 3);
    g_assert_cmpuint(enc[0], ==, 1);
    g_assert_cmpuint(enc[1], ==, 2);
    g_assert_cmpuint(enc[2], ==, 3);

    /* sensors low 3 bits = 5, leds in high byte = 2 */
    writew(GPIO_PHYS + 0x08, (uint16_t)((2 << 8) | 5));
    uint8_t sens, leds;
    qtest_memread(global_qtest, SHM_PHYS + 14, &sens, 1);
    qtest_memread(global_qtest, SHM_PHYS + 15, &leds, 1);
    g_assert_cmpuint(sens, ==, 5);
    g_assert_cmpuint(leds, ==, 2);
    g_assert_cmpuint(readw(GPIO_PHYS + 0x08), ==, (2 << 8) | 5);

    writel(GPIO_PHYS + 0x10, (uint32_t)(int32_t)-7);
    writel(GPIO_PHYS + 0x14, 100);
    writel(GPIO_PHYS + 0x18, 200);
    g_assert_cmpuint(readl(GPIO_PHYS + 0x10), ==, (uint32_t)(int32_t)-7);
    g_assert_cmpuint(readl(GPIO_PHYS + 0x14), ==, 100);
    g_assert_cmpuint(readl(GPIO_PHYS + 0x18), ==, 200);

    int32_t px, py, pz;
    qtest_memread(global_qtest, SHM_PHYS + 16, &px, 4);
    qtest_memread(global_qtest, SHM_PHYS + 20, &py, 4);
    qtest_memread(global_qtest, SHM_PHYS + 24, &pz, 4);
    g_assert_cmpint(px, ==, -7);
    g_assert_cmpint(py, ==, 100);
    g_assert_cmpint(pz, ==, 200);

    qtest_end();
}

/*
 * JSON: map X IN1→virtual pin 2, IN2→pin 3; ENC A→4, B→5.
 * Set vgpio_in bit 2 only → internal motor X pattern 0x01 (IN1).
 */
static void test_config_vgpio_motor_sync(void)
{
    const char *json = "{\"gpio_map\":{\"x\":{\"in1\":2,\"in2\":3,\"enc_a\":4,"
                       "\"enc_b\":5}}}";
    gchar *jpath = write_temp_plotter_json(json);
    char *cmd = make_cmdline();
    uint8_t m0;

    g_assert_true(g_setenv("QTEST_PLOTTER_CONFIG", jpath, TRUE));
    qtest_start(cmd);
    g_free(cmd);

    writel(GPIO_PHYS + 0x1c, 1u << 2);
    qtest_memread(global_qtest, SHM_PHYS + 8, &m0, 1);
    g_assert_cmpuint(m0, ==, 1);

    qtest_end();
    unlink(jpath);
    g_free(jpath);
    unset_plotter_config_env();
}

static void test_config_pulse_clamp_zero_max(void)
{
    const char *json = "{\"max_x_pulses\":50}";
    gchar *jpath = write_temp_plotter_json(json);
    char *cmd = make_cmdline();
    int32_t px;

    g_assert_true(g_setenv("QTEST_PLOTTER_CONFIG", jpath, TRUE));
    qtest_start(cmd);
    g_free(cmd);

    writel(GPIO_PHYS + 0x10, (uint32_t)(int32_t)-100);
    qtest_memread(global_qtest, SHM_PHYS + 16, &px, 4);
    g_assert_cmpint(px, ==, 0);

    writel(GPIO_PHYS + 0x10, 999);
    qtest_memread(global_qtest, SHM_PHYS + 16, &px, 4);
    g_assert_cmpint(px, ==, 50);

    qtest_end();
    unlink(jpath);
    g_free(jpath);
    unset_plotter_config_env();
}

/* Encoder guest 0x01 on axis X → vgpio_out bits at enc_a=4, enc_b=5 */
static void test_config_vgpio_encoder_out(void)
{
    const char *json = "{\"gpio_map\":{\"x\":{\"in1\":2,\"in2\":3,\"enc_a\":4,"
                       "\"enc_b\":5}}}";
    gchar *jpath = write_temp_plotter_json(json);
    char *cmd = make_cmdline();
    uint32_t vo;

    g_assert_true(g_setenv("QTEST_PLOTTER_CONFIG", jpath, TRUE));
    qtest_start(cmd);
    g_free(cmd);

    writel(GPIO_PHYS + 0x04, 0x00000001);
    vo = readl(GPIO_PHYS + 0x20);
    g_assert_cmpuint((vo >> 4) & 1u, ==, 1u);
    g_assert_cmpuint((vo >> 5) & 1u, ==, 0u);

    qtest_end();
    unlink(jpath);
    g_free(jpath);
    unset_plotter_config_env();
}

static void test_vgpio_read_write_unmapped_defaults(void)
{
    char *cmd = make_cmdline();

    unset_plotter_config_env();
    qtest_start(cmd);
    g_free(cmd);

    g_assert_cmpuint(readl(GPIO_PHYS + 0x1c), ==, 0);
    writel(GPIO_PHYS + 0x1c, 0xdeadbeef);
    g_assert_cmpuint(readl(GPIO_PHYS + 0x1c), ==, 0xdeadbeef);
    /* Write to RO vgpio_out is ignored */
    writel(GPIO_PHYS + 0x20, 0xffffffff);
    g_assert_cmpuint(readl(GPIO_PHYS + 0x20), ==, 0);

    qtest_end();
}

/* pwm_duty_default in JSON → MMIO 0x24 after reset (LE triplet). */
static void test_config_pwm_duty_default_json(void)
{
    const char *json = "{\"pwm_duty_default\":{\"x\":17,\"y\":18,\"z\":19}}";
    gchar *jpath = write_temp_plotter_json(json);
    char *cmd = make_cmdline();
    uint32_t w;

    g_assert_true(g_setenv("QTEST_PLOTTER_CONFIG", jpath, TRUE));
    qtest_start(cmd);
    g_free(cmd);

    w = readl(GPIO_PHYS + 0x24);
    g_assert_cmpuint(w, ==, 17u | (18u << 8) | (19u << 16));

    qtest_end();
    unlink(jpath);
    g_free(jpath);
    unset_plotter_config_env();
}

/* PWM duty triplet at 0x24: default 0xFFFFFF (255 each); guest may set per axis. */
static void test_pwm_duty_register(void)
{
    char *cmd = make_cmdline();

    unset_plotter_config_env();
    qtest_start(cmd);
    g_free(cmd);

    g_assert_cmpuint(readl(GPIO_PHYS + 0x24), ==, 0x00ffffff);
    writel(GPIO_PHYS + 0x24, 0x00010203);
    g_assert_cmpuint(readl(GPIO_PHYS + 0x24), ==, 0x00010203);

    qtest_end();
}

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);

    g_assert(getenv("QTEST_QEMU_BINARY"));
    g_assert_cmpstr(qtest_get_arch(), ==, "mipsel");

    qtest_add_func("/plotter_bridge/magic_header", test_magic_header);
    qtest_add_func("/plotter_bridge/gpio_motors", test_gpio_updates_shm_motors);
    qtest_add_func("/plotter_bridge/shm_to_gpio", test_shm_ram_visible_on_gpio);
    qtest_add_func("/plotter_bridge/encoder_sensors_pulses",
                   test_encoder_sensors_leds_pulses);
    qtest_add_func("/plotter_bridge/config_vgpio_motor", test_config_vgpio_motor_sync);
    qtest_add_func("/plotter_bridge/config_pulse_clamp", test_config_pulse_clamp_zero_max);
    qtest_add_func("/plotter_bridge/config_vgpio_encoder_out",
                   test_config_vgpio_encoder_out);
    qtest_add_func("/plotter_bridge/vgpio_mmio_default", test_vgpio_read_write_unmapped_defaults);
    qtest_add_func("/plotter_bridge/config_pwm_duty_default_json",
                   test_config_pwm_duty_default_json);
    qtest_add_func("/plotter_bridge/pwm_duty_register", test_pwm_duty_register);

    ret = g_test_run();
    return ret;
}
