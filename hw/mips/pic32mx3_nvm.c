/*
 * PIC32MX3 NVMCON flash programming for pic32mx3-generic.
 *
 * Models word program, row program, and page erase against the same RAM
 * backing as -kernel / hex load. Completes instantly; clears NVMCON.WR when done.
 * Requires the two-step NVMKEY unlock before WR is honored.
 *
 * Page / row sizes match PIC32MX3 family (e.g. MX350F256H).
 */
#include "hw/hw.h"
#include "exec/cpu-common.h"
#include "translate-all.h"
#include "sysemu/sysemu.h"
#include "qemu/error-report.h"
#include "pic32_peripherals.h"
#include "pic32mx3_nvm.h"
#include "pic32mx.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

/* PIC32MX350-class flash geometry */
#define NVM_PAGE_SIZE   1024
#define NVM_ROW_SIZE    512

#define NVMKEY1         0xaa996655u
#define NVMKEY2         0x556699aau

static int nvm_trace(void)
{
    static int tr = -1;
    if (tr < 0) {
        tr = getenv("QEMU_PIC32_TRACE_NVM") != NULL;
    }
    return tr;
}

static void nvm_invalidate_phys(uint32_t bus_addr, size_t len)
{
    hwaddr pa = bus_addr & 0x1fffffffu;

    tb_invalidate_phys_range((tb_page_addr_t)pa, (tb_page_addr_t)(pa + len));
}

/*
 * PIC32 maps KSEG0 and KSEG1 to the same physical RAM for main memory.
 * cpu_mips_kseg0_to_phys (VA & 0x1fffffff) is wrong for KSEG1 (0xAxxx -> 0x2xxxxxxx).
 */
static hwaddr pic32mx3_nvmsrc_phys(uint32_t va)
{
    if (va >= 0x80000000 && va < 0xa0000000) {
        return va - 0x80000000;
    }
    if (va >= 0xa0000000 && va < 0xc0000000) {
        return va - 0xa0000000;
    }
    return va & 0x1fffffffu;
}

static int nvm_flash_window(pic32_t *s, uint32_t bus_addr,
                          uint8_t **host_base, hwaddr *offset_out, hwaddr *len_out)
{
    uint32_t p = bus_addr & 0x1fffffffu;

    if (s->prog_flash_host && p >= s->prog_flash_bus_base &&
        p < s->prog_flash_bus_base + s->prog_flash_size) {
        *host_base = s->prog_flash_host;
        *offset_out = p - s->prog_flash_bus_base;
        *len_out = s->prog_flash_size;
        return 0;
    }
    if (s->boot_flash_host && p >= s->boot_flash_bus_base &&
        p < s->boot_flash_bus_base + s->boot_flash_size) {
        *host_base = s->boot_flash_host;
        *offset_out = p - s->boot_flash_bus_base;
        *len_out = s->boot_flash_size;
        return 0;
    }
    return -1;
}

void pic32mx3_nvm_bind_flash(pic32_t *s,
                           uint8_t *prog_host, uint8_t *boot_host,
                           uint32_t prog_sz, uint32_t boot_sz,
                           uint32_t prog_bus, uint32_t boot_bus)
{
    s->prog_flash_host = prog_host;
    s->boot_flash_host = boot_host;
    s->prog_flash_size = prog_sz;
    s->boot_flash_size = boot_sz;
    s->prog_flash_bus_base = prog_bus;
    s->boot_flash_bus_base = boot_bus;
    s->nvm_key_step = 0;
}

void pic32mx3_nvm_reset(pic32_t *s)
{
    s->nvm_key_step = 0;
}

void pic32mx3_nvm_key_write(pic32_t *s, uint32_t value)
{
    if (value == NVMKEY1 && s->nvm_key_step == 0) {
        s->nvm_key_step = 1;
    } else if (value == NVMKEY2 && s->nvm_key_step == 1) {
        s->nvm_key_step = 2;
    } else {
        s->nvm_key_step = 0;
    }
}

static uint32_t nvm_fail(pic32_t *s, uint32_t con)
{
    s->nvm_key_step = 0;
    con &= ~PIC32_NVMCON_WR;
    con |= PIC32_NVMCON_WRERR;
    return con;
}

static uint32_t nvm_do_word_program(pic32_t *s, uint32_t con)
{
    uint8_t *base;
    hwaddr off, winlen;
    uint32_t addr = VALUE(NVMADDR);
    uint32_t data = VALUE(NVMDATA);

    if (addr & 3u) {
        return nvm_fail(s, con);
    }
    if (nvm_flash_window(s, addr, &base, &off, &winlen) < 0) {
        return nvm_fail(s, con);
    }
    if (off + 4 > winlen) {
        return nvm_fail(s, con);
    }
    memcpy(base + off, &data, 4);
    if (nvm_trace()) {
        fprintf(stderr, "pic32 nvm: word program pa=%08x data=%08x\n", addr, data);
    }
    nvm_invalidate_phys(addr, 4);
    s->nvm_key_step = 0;
    con &= ~PIC32_NVMCON_WR;
    return con;
}

static uint32_t nvm_do_page_erase(pic32_t *s, uint32_t con)
{
    uint8_t *base;
    hwaddr off, winlen;
    uint32_t addr = VALUE(NVMADDR);
    hwaddr page_off;

    if (nvm_flash_window(s, addr, &base, &off, &winlen) < 0) {
        return nvm_fail(s, con);
    }
    page_off = off & ~(hwaddr)(NVM_PAGE_SIZE - 1);
    if (page_off + NVM_PAGE_SIZE > winlen) {
        return nvm_fail(s, con);
    }
    memset(base + page_off, 0xff, NVM_PAGE_SIZE);
    if (nvm_trace()) {
        fprintf(stderr, "pic32 nvm: page erase pa=%08x (host off %#lx)\n",
                addr, (unsigned long)page_off);
    }
    {
        uint32_t page_bus = (base == s->prog_flash_host)
            ? s->prog_flash_bus_base + (uint32_t)page_off
            : s->boot_flash_bus_base + (uint32_t)page_off;
        nvm_invalidate_phys(page_bus, NVM_PAGE_SIZE);
    }
    s->nvm_key_step = 0;
    con &= ~PIC32_NVMCON_WR;
    return con;
}

static uint32_t nvm_do_row_program(pic32_t *s, uint32_t con)
{
    uint8_t *base, rowbuf[NVM_ROW_SIZE];
    hwaddr off, winlen;
    uint32_t dst = VALUE(NVMADDR);
    uint32_t src_va = VALUE(NVMSRCADDR);
    hwaddr src_pa = pic32mx3_nvmsrc_phys(src_va);
    hwaddr row_off;

    if (nvm_flash_window(s, dst, &base, &off, &winlen) < 0) {
        return nvm_fail(s, con);
    }
    row_off = off & ~(hwaddr)(NVM_ROW_SIZE - 1);
    if (row_off + NVM_ROW_SIZE > winlen) {
        return nvm_fail(s, con);
    }
    cpu_physical_memory_read(src_pa, rowbuf, NVM_ROW_SIZE);
    memcpy(base + row_off, rowbuf, NVM_ROW_SIZE);
    if (nvm_trace()) {
        fprintf(stderr,
                "pic32 nvm: row program dst_pa=%08x src_va=%08x src_pa=%#lx\n",
                dst, src_va, (unsigned long)src_pa);
    }
    {
        uint32_t row_bus = (base == s->prog_flash_host)
            ? s->prog_flash_bus_base + (uint32_t)row_off
            : s->boot_flash_bus_base + (uint32_t)row_off;
        nvm_invalidate_phys(row_bus, NVM_ROW_SIZE);
    }
    s->nvm_key_step = 0;
    con &= ~PIC32_NVMCON_WR;
    return con;
}

uint32_t pic32mx3_nvm_nvmcon_write(pic32_t *s, uint32_t old_con, uint32_t new_con)
{
    unsigned op = new_con & PIC32_NVMCON_NVMOP;

    (void)old_con;

    if (!(new_con & PIC32_NVMCON_WR)) {
        return new_con;
    }
    /* WR set: consume keys even if we abort */
    if (s->nvm_key_step != 2) {
        if (nvm_trace()) {
            fprintf(stderr, "pic32 nvm: WR without unlock (step=%u)\n", s->nvm_key_step);
        }
        return nvm_fail(s, new_con);
    }
    if (!(new_con & PIC32_NVMCON_WREN)) {
        if (nvm_trace()) {
            fprintf(stderr, "pic32 nvm: WR without WREN\n");
        }
        return nvm_fail(s, new_con);
    }

    switch (op) {
    case PIC32_NVMCON_WORD_PGM:
        return nvm_do_word_program(s, new_con);
    case PIC32_NVMCON_ROW_PGM:
        return nvm_do_row_program(s, new_con);
    case PIC32_NVMCON_PAGE_ERASE:
        return nvm_do_page_erase(s, new_con);
    case PIC32_NVMCON_NOP:
        if (nvm_trace()) {
            fprintf(stderr, "pic32 nvm: NVMOP nop with WR\n");
        }
        s->nvm_key_step = 0;
        new_con &= ~PIC32_NVMCON_WR;
        return new_con;
    default:
        if (nvm_trace()) {
            fprintf(stderr, "pic32 nvm: unsupported NVMOP %u\n", op);
        }
        return nvm_fail(s, new_con);
    }
}

static void pic32mx3_flash_exit_notify(Notifier *n, void *unused)
{
    pic32_t *s = container_of(n, pic32_t, flash_exit_notifier);

    (void)unused;

    if (s->prog_flash_path && s->prog_flash_path[0] && s->prog_flash_host) {
        FILE *f = fopen(s->prog_flash_path, "wb");

        if (!f) {
            error_report("pic32: cannot write prog-flash '%s': %s",
                         s->prog_flash_path, strerror(errno));
        } else {
            size_t nw = fwrite(s->prog_flash_host, 1, s->prog_flash_size, f);

            if (nw != s->prog_flash_size) {
                error_report("pic32: short write prog-flash (%zu of %u bytes)",
                             nw, s->prog_flash_size);
            }
            fclose(f);
        }
    }
    if (s->boot_flash_path && s->boot_flash_path[0] && s->boot_flash_host) {
        FILE *f = fopen(s->boot_flash_path, "wb");

        if (!f) {
            error_report("pic32: cannot write boot-flash '%s': %s",
                         s->boot_flash_path, strerror(errno));
        } else {
            size_t nw = fwrite(s->boot_flash_host, 1, s->boot_flash_size, f);

            if (nw != s->boot_flash_size) {
                error_report("pic32: short write boot-flash (%zu of %u bytes)",
                             nw, s->boot_flash_size);
            }
            fclose(f);
        }
    }
}

void pic32mx3_flash_load_images(pic32_t *s)
{
    if (s->prog_flash_host && s->prog_flash_size) {
        memset(s->prog_flash_host, 0xff, s->prog_flash_size);
        if (s->prog_flash_path && s->prog_flash_path[0]) {
            FILE *f = fopen(s->prog_flash_path, "rb");

            if (f) {
                size_t n = fread(s->prog_flash_host, 1, s->prog_flash_size, f);

                fclose(f);
                if (nvm_trace()) {
                    fprintf(stderr, "pic32 flash: read %zu prog bytes from '%s'\n",
                            n, s->prog_flash_path);
                }
            }
        }
    }
    if (s->boot_flash_host && s->boot_flash_size) {
        memset(s->boot_flash_host, 0xff, s->boot_flash_size);
        if (s->boot_flash_path && s->boot_flash_path[0]) {
            FILE *f = fopen(s->boot_flash_path, "rb");

            if (f) {
                size_t n = fread(s->boot_flash_host, 1, s->boot_flash_size, f);

                fclose(f);
                if (nvm_trace()) {
                    fprintf(stderr, "pic32 flash: read %zu boot bytes from '%s'\n",
                            n, s->boot_flash_path);
                }
            }
        }
    }
}

void pic32mx3_flash_register_exit_save(pic32_t *s)
{
    if ((!s->prog_flash_path || !s->prog_flash_path[0]) &&
        (!s->boot_flash_path || !s->boot_flash_path[0])) {
        return;
    }
    s->flash_exit_notifier.notify = pic32mx3_flash_exit_notify;
    qemu_add_exit_notifier(&s->flash_exit_notifier);
}
