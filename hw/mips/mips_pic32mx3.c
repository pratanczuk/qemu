/*
 * QEMU support for Microchip PIC32MX3 microcontroller.
 *
 * Copyright (c) 2015 Serge Vakulenko
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/* Only 32-bit little endian mode supported. */
#include "config.h"
#if !defined TARGET_MIPS64 && !defined TARGET_WORDS_BIGENDIAN

#include "hw/i386/pc.h"
#include "hw/char/serial.h"
#include "hw/mips/cpudevs.h"
#include "hw/qdev-properties.h"
#include "sysemu/char.h"
#include "hw/loader.h"
#include "qemu/error-report.h"
#include "sysemu/sysemu.h"
#include "hw/empty_slot.h"
#include "elf.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "hw/qdev.h"
#include "sysemu/qtest.h"
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <endian.h>

#define PIC32MX3
#include "pic32mx.h"
#include "pic32_peripherals.h"
#include "pic32mx3_nvm.h"

/* Hardware addresses */
#define PROGRAM_FLASH_START 0x1d000000
#define BOOT_FLASH_START    0x1fc00000
#define DATA_MEM_START      0x00000000
#define IO_MEM_START        0x1f800000

#define PROGRAM_FLASH_SIZE  (256*1024)          // 256 kbytes
#define BOOT_FLASH_SIZE     (12*1024)           // 12 kbytes
#define DATA_MEM_SIZE       (64*1024)           // 64 kbytes
#define USER_MEM_START      0xbf000000

#define TYPE_MIPS_PIC32     "mips-pic32mx3"

/*
 * Board variants.
 */
enum {
    BOARD_MX350_GENERIC,            /* Generic PIC32MX350F256H board */
};

static const char *board_name[] = {
    "PIC32MX350F256H Generic",
};

/*
 * Pointers to Flash memory contents.
 */
static char *prog_ptr;
static char *boot_ptr;

#define BOOTMEM(addr) ((uint32_t*) boot_ptr) [(addr & 0xffff) >> 2]

/*
 * TODO: add option to enable tracing.
 * IRQ: set QEMU_PIC32_TRACE_IRQ=1 to log each irq_raise() to stderr.
 */
#define TRACE   0

/*
 * PIC32MX350F256H interrupt vector numbers.
 * Source: p32mx350f256h.h (_xxx_VECTOR defines).
 * These differ from the PIC32MX7xx defines in pic32mx.h.
 */
#define MX3_VECT_CT       0
#define MX3_VECT_CS0      1
#define MX3_VECT_CS1      2
#define MX3_VECT_INT0     3
#define MX3_VECT_T1       4
#define MX3_VECT_IC1      5
#define MX3_VECT_OC1      6
#define MX3_VECT_INT1     7
#define MX3_VECT_T2       8
#define MX3_VECT_IC2      9
#define MX3_VECT_OC2      10
#define MX3_VECT_INT2     11
#define MX3_VECT_T3       12
#define MX3_VECT_IC3      13
#define MX3_VECT_OC3      14
#define MX3_VECT_INT3     15
#define MX3_VECT_T4       16
#define MX3_VECT_IC4      17
#define MX3_VECT_OC4      18
#define MX3_VECT_INT4     19
#define MX3_VECT_T5       20
#define MX3_VECT_IC5      21
#define MX3_VECT_OC5      22
#define MX3_VECT_AD1      23
#define MX3_VECT_FSCM     24
#define MX3_VECT_RTCC     25
#define MX3_VECT_FCE      26
#define MX3_VECT_CMP1     27
#define MX3_VECT_CMP2     28
/* 29 reserved */
#define MX3_VECT_SPI1     30
#define MX3_VECT_U1       31
#define MX3_VECT_I2C1     32
#define MX3_VECT_CN       33
#define MX3_VECT_PMP      34
#define MX3_VECT_SPI2     35
#define MX3_VECT_U2       36
#define MX3_VECT_I2C2     37
#define MX3_VECT_U3       38
#define MX3_VECT_U4       39
/* 40 reserved */
#define MX3_VECT_CTMU     41
#define MX3_VECT_DMA0     42
#define MX3_VECT_DMA1     43
#define MX3_VECT_DMA2     44
#define MX3_VECT_DMA3     45

/*
 * PIC32MX350F256H: irq index is the IFS bit position (IFS0: 0–31, IFS1: 32–63,
 * IFS2: 64–95). irq_raise() ORs 1 << irq into the corresponding IFS register.
 */
static const int irq_to_vector[] = {
    /* IFS0 */
    MX3_VECT_CT,       /* 0  CTIF */
    MX3_VECT_CS0,      /* 1  CS0IF */
    MX3_VECT_CS1,      /* 2  CS1IF */
    MX3_VECT_INT0,     /* 3  INT0IF */
    MX3_VECT_T1,       /* 4  T1IF */
    MX3_VECT_IC1,      /* 5  IC1EIF */
    MX3_VECT_IC1,      /* 6  IC1IF */
    MX3_VECT_OC1,      /* 7  OC1IF */
    MX3_VECT_INT1,     /* 8  INT1IF */
    MX3_VECT_T2,       /* 9  T2IF */
    MX3_VECT_IC2,      /* 10 IC2EIF */
    MX3_VECT_IC2,      /* 11 IC2IF */
    MX3_VECT_OC2,      /* 12 OC2IF */
    MX3_VECT_INT2,     /* 13 INT2IF */
    MX3_VECT_T3,       /* 14 T3IF */
    MX3_VECT_IC3,      /* 15 IC3EIF */
    MX3_VECT_IC3,      /* 16 IC3IF */
    MX3_VECT_OC3,      /* 17 OC3IF */
    MX3_VECT_INT3,     /* 18 INT3IF */
    MX3_VECT_T4,       /* 19 T4IF */
    MX3_VECT_IC4,      /* 20 IC4EIF */
    MX3_VECT_IC4,      /* 21 IC4IF */
    MX3_VECT_OC4,      /* 22 OC4IF */
    MX3_VECT_INT4,     /* 23 INT4IF */
    MX3_VECT_T5,       /* 24 T5IF */
    MX3_VECT_IC5,      /* 25 IC5EIF */
    MX3_VECT_IC5,      /* 26 IC5IF */
    MX3_VECT_OC5,      /* 27 OC5IF */
    MX3_VECT_AD1,      /* 28 AD1IF */
    MX3_VECT_FSCM,     /* 29 FSCMIF */
    MX3_VECT_RTCC,     /* 30 RTCCIF */
    MX3_VECT_FCE,      /* 31 FCEIF */
    /* IFS1 */
    MX3_VECT_CMP1,     /* 32 CMP1IF */
    MX3_VECT_CMP2,     /* 33 CMP2IF */
    -1,                /* 34 reserved */
    MX3_VECT_SPI1,     /* 35 SPI1EIF */
    MX3_VECT_SPI1,     /* 36 SPI1RXIF */
    MX3_VECT_SPI1,     /* 37 SPI1TXIF */
    MX3_VECT_U1,       /* 38 U1EIF */
    MX3_VECT_U1,       /* 39 U1RXIF */
    MX3_VECT_U1,       /* 40 U1TXIF */
    MX3_VECT_I2C1,     /* 41 I2C1BIF */
    MX3_VECT_I2C1,     /* 42 I2C1SIF */
    MX3_VECT_I2C1,     /* 43 I2C1MIF */
    MX3_VECT_CN,       /* 44 CNAIF */
    MX3_VECT_CN,       /* 45 CNBIF */
    MX3_VECT_CN,       /* 46 CNCIF */
    MX3_VECT_CN,       /* 47 CNDIF */
    MX3_VECT_CN,       /* 48 CNEIF */
    MX3_VECT_CN,       /* 49 CNFIF */
    MX3_VECT_CN,       /* 50 CNGIF */
    MX3_VECT_PMP,      /* 51 PMPIF */
    MX3_VECT_PMP,      /* 52 PMPEIF */
    MX3_VECT_SPI2,     /* 53 SPI2EIF */
    MX3_VECT_SPI2,     /* 54 SPI2RXIF */
    MX3_VECT_SPI2,     /* 55 SPI2TXIF */
    MX3_VECT_U2,       /* 56 U2EIF */
    MX3_VECT_U2,       /* 57 U2RXIF */
    MX3_VECT_U2,       /* 58 U2TXIF */
    MX3_VECT_I2C2,     /* 59 I2C2BIF */
    MX3_VECT_I2C2,     /* 60 I2C2SIF */
    MX3_VECT_I2C2,     /* 61 I2C2MIF */
    MX3_VECT_U3,       /* 62 U3EIF */
    MX3_VECT_U3,       /* 63 U3RXIF */
    /* IFS2 (MX350 subset) */
    MX3_VECT_U3,       /* 64 U3TXIF */
    MX3_VECT_U4,       /* 65 U4EIF */
    MX3_VECT_U4,       /* 66 U4RXIF */
    MX3_VECT_U4,       /* 67 U4TXIF */
    -1,                /* 68 */
    -1,                /* 69 */
    -1,                /* 70 */
    MX3_VECT_CTMU,     /* 71 CTMUIF */
    MX3_VECT_DMA0,     /* 72 DMA0IF */
    MX3_VECT_DMA1,     /* 73 DMA1IF */
    MX3_VECT_DMA2,     /* 74 DMA2IF */
    MX3_VECT_DMA3,     /* 75 DMA3IF */
};

/*
 * PIC32MX350 IPC registers are indexed by *vector number*, not IFS bit
 * position.  Multiple IFS bits can share one vector (e.g. ICxEIF + ICxIF,
 * UxEIF + UxRXIF + UxTXIF), so the mapping irq→IPC is:
 *   vec = irq_to_vector[irq];   IPC(vec >> 2), field (vec & 3).
 */
static int pic32mx3_irq_priority(const pic32_t *s, int irq)
{
    int v;
    int n;
    uint32_t ipc;

    if (irq < 0 || irq >= (int)(sizeof(irq_to_vector) / sizeof(irq_to_vector[0]))) {
        return 0;
    }
    v = irq_to_vector[irq];
    if (v < 0) {
        return 0;
    }
    n = v >> 2;
    if (n > 12) {
        return 0;
    }
    ipc = VALUE(IPC(n));
    return (ipc >> (2 + (v & 3) * 8)) & 7;
}

static void update_irq_status(pic32_t *s)
{
    /* Assume no interrupts pending. */
    int cause_ripl = 0;
    int vector = 0;
    CPUMIPSState *env = &s->cpu->env;
    int current_ripl = (env->CP0_Cause >> (CP0Ca_IP + 2)) & 0x3f;

    VALUE(INTSTAT) = 0;

    if ((VALUE(IFS0) & VALUE(IEC0)) ||
        (VALUE(IFS1) & VALUE(IEC1)) ||
        (VALUE(IFS2) & VALUE(IEC2)))
    {
        /* Find the most prioritized pending interrupt,
         * it's vector and level. */
        int irq;
        for (irq=0; irq<sizeof(irq_to_vector)/sizeof(int); irq++) {
            int n = irq >> 5;

            if (((VALUE(IFS(n)) & VALUE(IEC(n))) >> (irq & 31)) & 1) {
                /* Interrupt is pending. */
                int v = irq_to_vector [irq];
                if (v < 0)
                    continue;

                int level = pic32mx3_irq_priority(s, irq);
                if (level > cause_ripl) {
                    vector = v;
                    cause_ripl = level;
                }
            }
        }
        VALUE(INTSTAT) = vector | (cause_ripl << 8);
        env->eic_vector = vector & 0xff;
    } else {
        env->eic_vector = 0;
    }

    if (TRACE)
        fprintf(qemu_logfile, "--- Priority level Cause.RIPL = %u\n",
            cause_ripl);

    /*
     * Modify Cause.RIPL field and take EIC interrupt.
     */
    env->CP0_Cause &= ~(0x3f << (CP0Ca_IP + 2));
    env->CP0_Cause |= cause_ripl << (CP0Ca_IP + 2);
    /*
     * Always poke the CPU when any enabled interrupt is pending. Omitting
     * cpu_interrupt when Cause.RIPL was unchanged left guest tight loops
     * (e.g. a firmware busy-wait delay_ms) spinning in one TB while the T3 tick ISR
     * never ran.
     */
    if ((VALUE(IFS0) & VALUE(IEC0)) ||
        (VALUE(IFS1) & VALUE(IEC1)) ||
        (VALUE(IFS2) & VALUE(IEC2))) {
        cpu_interrupt(CPU(s->cpu), CPU_INTERRUPT_HARD);
    } else if (cause_ripl != current_ripl) {
        cpu_interrupt(CPU(s->cpu), CPU_INTERRUPT_HARD);
    }
}

/*
 * Decode IFS bit index for debug (set QEMU_PIC32_TRACE_IRQ=1).
 */
static const char *pic32mx3_irq_name(int irq)
{
    switch (irq) {
    case 0: return "CT";
    case 4: return "T1IF";
    case 6: return "IC1IF";
    case 7: return "OC1IF";
    case 9: return "T2IF";
    case 12: return "OC2IF";
    case 17: return "OC3IF";
    case 22: return "OC4IF";
    case 14: return "T3IF";
    case 19: return "T4IF";
    case 24: return "T5IF";
    case 28: return "U1TXIF";
    case 32: return "CNIF";
    case 33: return "AD1IF";
    case 43: return "I2C2MIF";
    case 51: return "PMPIF";
    case 56: return "U2TXIF";
    case 62: return "U3EIF";
    case 63: return "U3RXIF";
    case 64: return "U3TXIF";
    default: return "?";
    }
}

/*
 * Set interrupt flag status
 */
static void irq_raise(pic32_t *s, int irq)
{
    if (getenv("QEMU_PIC32_TRACE_IRQ")) {
        CPUMIPSState *env = &s->cpu->env;
        int m = irq >> 5;
        int b = irq & 31;
        uint32_t iec = m == 0 ? VALUE(IEC0) : (m == 1 ? VALUE(IEC1) : VALUE(IEC2));

        fprintf(stderr,
                "pic32mx3 irq_raise %s(%d) PC=0x%08x IFS&IEC[%d]=%d "
                "IFS0=%08x IFS1=%08x IFS2=%08x IEC0=%08x IEC1=%08x IEC2=%08x\n",
                pic32mx3_irq_name(irq), irq, (unsigned)env->active_tc.PC, irq,
                (int)((iec >> b) & 1u), VALUE(IFS0), VALUE(IFS1), VALUE(IFS2),
                VALUE(IEC0), VALUE(IEC1), VALUE(IEC2));
    }
    VALUE(IFS(irq >> 5)) |= 1u << (irq & 31);
    update_irq_status(s);
}

/*
 * Clear interrupt flag status
 */
static void irq_clear(pic32_t *s, int irq)
{
    if (! (VALUE(IFS(irq >> 5)) & (1 << (irq & 31))))
        return;

    VALUE(IFS(irq >> 5)) &= ~(1 << (irq & 31));
    update_irq_status(s);
}

/*
 * Timer interrupt.
 */
static void pic32_timer_irq(CPUMIPSState *env, int raise)
{
    pic32_t *s = env->eic_context;

    if (raise) {
        if (TRACE)
            fprintf(qemu_logfile, "--- %08x: Timer interrupt\n",
                env->active_tc.PC);
        irq_raise(s, 0);
    } else {
        if (TRACE)
            fprintf(qemu_logfile, "--- Clear timer interrupt\n");
        irq_clear(s, 0);
    }
}

/*
 * Software interrupt.
 */
static void pic32_soft_irq(CPUMIPSState *env, int num)
{
    pic32_t *s = env->eic_context;

    if (TRACE)
        fprintf(qemu_logfile, "--- %08x: Soft interrupt %u\n",
            env->active_tc.PC, num);
    irq_raise(s, num + 1);
}

/* PBCLK for PIC32MX350 generic board.
 * Must match the firmware's F_PB_HZ (48 MHz for Cricut Joy config). */
#define PIC32MX3_PBCLK_HZ 48000000ull

/*
 * T1–T5 period match uses the same PBCLK-derived nanosecond period.
 * With -icount, use VIRTUAL_RT so polled TxIF sees wall-time progress (see T3).
 */
static inline QEMUClockType pic32mx3_tmr3_clock_type(void)
{
    return use_icount ? QEMU_CLOCK_VIRTUAL_RT : QEMU_CLOCK_VIRTUAL;
}

static inline int64_t pic32mx3_tmr3_clock_get_ns(void)
{
    return qemu_clock_get_ns(pic32mx3_tmr3_clock_type());
}

/* IFS bit indices for T1IF..T5IF on PIC32MX350 (matches irq_to_vector[]). */
static const uint8_t pic32mx3_tmr_ifs_irq[PIC32_MX3_N_TMRS] = { 4, 9, 14, 19, 24 };

static uint64_t pic32mx3_typeb_period_ns(uint32_t tcon, uint32_t pr)
{
    unsigned tckps = (tcon >> 4) & 7u;
    static const uint32_t prescale8[8] = { 1, 2, 4, 8, 16, 32, 64, 256 };
    uint64_t pre = prescale8[tckps];
    uint64_t ticks = (uint64_t)(pr + 1u) * pre;

    if (ticks == 0) {
        return 1000000ull;
    }
    return (ticks * 1000000000ull) / PIC32MX3_PBCLK_HZ;
}

static uint64_t pic32mx3_tmr_period_ns(pic32_t *s, int idx)
{
    uint32_t tcon, pr;

    switch (idx) {
    case 0:                     /* T1 Type A */
        tcon = VALUE(T1CON);
        pr = (uint16_t)VALUE(PR1);
        if (tcon & (1u << 1)) { /* TCS external clock — stub */
            return 1000000000ull;
        }
        {
            unsigned tckps = (tcon >> 4) & 3u;
            static const uint32_t prescale4[4] = { 1, 8, 64, 256 };
            uint64_t pre = prescale4[tckps];
            uint64_t ticks = (uint64_t)(pr + 1u) * pre;

            if (ticks == 0) {
                return 1000000ull;
            }
            return (ticks * 1000000000ull) / PIC32MX3_PBCLK_HZ;
        }
    case 1:
        return pic32mx3_typeb_period_ns(VALUE(T2CON), (uint16_t)VALUE(PR2));
    case 2:
        return pic32mx3_typeb_period_ns(VALUE(T3CON), (uint16_t)VALUE(PR3));
    case 3: {
        uint32_t t4con = VALUE(T4CON);
        if (t4con & (1u << 3)) {
            /* T32: 32-bit mode — period is PR5:PR4 combined. */
            uint32_t pr32 = ((uint32_t)(uint16_t)VALUE(PR5) << 16)
                          | (uint32_t)(uint16_t)VALUE(PR4);
            return pic32mx3_typeb_period_ns(t4con, pr32);
        }
        return pic32mx3_typeb_period_ns(t4con, (uint16_t)VALUE(PR4));
    }
    case 4:
        /* If T4CON.T32 is active, TMR5 is not standalone — return huge period. */
        if (VALUE(T4CON) & (1u << 3)) {
            return 1000000000ull;
        }
        return pic32mx3_typeb_period_ns(VALUE(T5CON), (uint16_t)VALUE(PR5));
    default:
        return 1000000ull;
    }
}

static void pic32mx3_tmr_recompute(pic32_t *s, int idx);
static void pic32mx3_tmrs_recompute_all(pic32_t *s);

/*
 * Output compare (minimal): OCxIF when the selected time base wraps.  OCTSEL
 * bit 3: 0 = Timer2 (tmr idx 1), 1 = Timer3 (tmr idx 2).  Any non-zero OCM
 * enables the pulse.  Pin/compare value accuracy is not modeled (uart-cli).
 */
static void pic32mx3_oc_raise_for_timer(pic32_t *s, int tmr_idx)
{
    static const uint32_t oc_con[5] = {
        OC1CON, OC2CON, OC3CON, OC4CON, OC5CON
    };
    static const int oc_irq[5] = { 7, 12, 17, 22, 27 };
    int i;

    if (tmr_idx != 1 && tmr_idx != 2) {
        return;
    }
    for (i = 0; i < 5; i++) {
        unsigned con = VALUE(oc_con[i]);
        unsigned ocm = con & 7u;

        if (ocm == 0u) {
            continue;
        }
        if (((con >> 3) & 1u) != (unsigned)(tmr_idx - 1)) {
            continue;
        }
        irq_raise(s, oc_irq[i]);
    }
}

static void pic32mx3_tmr_cb(void *opaque)
{
    Pic32Mx3TmrOpaque *o = opaque;
    pic32_t *s = o->mcu;
    int idx = o->idx;

    switch (idx) {
    case 0:
        if (!(VALUE(T1CON) & 0x8000u)) {
            return;
        }
        VALUE(TMR1) = 0;
        break;
    case 1:
        if (!(VALUE(T2CON) & 0x8000u)) {
            return;
        }
        VALUE(TMR2) = 0;
        break;
    case 2:
        if (!(VALUE(T3CON) & 0x8000u)) {
            return;
        }
        VALUE(TMR3) = 0;
        break;
    case 3:
        if (!(VALUE(T4CON) & 0x8000u)) {
            if (getenv("QEMU_PIC32_TRACE_T32"))
                fprintf(stderr, "TMR4_CB idx=3 ON=0 → skip\n");
            return;
        }
        VALUE(TMR4) = 0;
        if (VALUE(T4CON) & (1u << 3)) {
            /* T32: 32-bit mode — clear TMR5 too, fire T5IF (slave). */
            VALUE(TMR5) = 0;
            if (getenv("QEMU_PIC32_TRACE_T32"))
                fprintf(stderr, "TMR4_CB T32 fire T5IF, IEC0=0x%08x\n",
                        VALUE(IEC0));
            irq_raise(s, pic32mx3_tmr_ifs_irq[4]); /* T5IF */
            pic32mx3_oc_raise_for_timer(s, idx);
            pic32mx3_tmr_recompute(s, idx);
            return;
        }
        break;
    case 4:
        if (!(VALUE(T5CON) & 0x8000u)) {
            return;
        }
        VALUE(TMR5) = 0;
        break;
    default:
        return;
    }
    irq_raise(s, pic32mx3_tmr_ifs_irq[idx]);
    pic32mx3_oc_raise_for_timer(s, idx);
    pic32mx3_tmr_recompute(s, idx);
}

static void pic32mx3_tmr_recompute(pic32_t *s, int idx)
{
    uint32_t tcon;

    if (idx < 0 || idx >= PIC32_MX3_N_TMRS || !s->tmr_timer[idx]) {
        return;
    }
    switch (idx) {
    case 0: tcon = VALUE(T1CON); break;
    case 1: tcon = VALUE(T2CON); break;
    case 2: tcon = VALUE(T3CON); break;
    case 3: tcon = VALUE(T4CON); break;
    case 4: tcon = VALUE(T5CON); break;
    default:
        return;
    }
    if (!(tcon & 0x8000u)) {
        if (idx == 3 && getenv("QEMU_PIC32_TRACE_T32"))
            fprintf(stderr, "TMR4_RECOMPUTE idx=3 ON=0 → timer_del\n");
        timer_del(s->tmr_timer[idx]);
        return;
    }
    {
        uint64_t per = pic32mx3_tmr_period_ns(s, idx);

        if (per < 1000ull) {
            per = 1000ull;
        }
        if (idx == 3 && getenv("QEMU_PIC32_TRACE_T32"))
            fprintf(stderr, "TMR4_RECOMPUTE idx=3 per=%llu ns T4CON=0x%08x PR4=0x%04x PR5=0x%04x\n",
                    (unsigned long long)per, tcon,
                    (unsigned)VALUE(PR4), (unsigned)VALUE(PR5));
        timer_mod(s->tmr_timer[idx],
                  pic32mx3_tmr3_clock_get_ns() + per);
    }
}

static void pic32mx3_tmrs_recompute_all(pic32_t *s)
{
    int i;

    for (i = 0; i < PIC32_MX3_N_TMRS; i++) {
        pic32mx3_tmr_recompute(s, i);
    }
}

/*
 * Perform an assign/clear/set/invert operation.
 */
static inline unsigned write_op(int a, int b, int op)
{
    switch (op & 0xc) {
    case 0x0: a = b;   break;   // Assign
    case 0x4: a &= ~b; break;   // Clear
    case 0x8: a |= b;  break;   // Set
    case 0xc: a ^= b;  break;   // Invert
    }
    return a;
}

/*
 * PIC32MX350F256H maps ports B–G with ANSEL and per-port change-notification
 * registers every 0x100 bytes starting at 0xBF886100 (physical offset 0x86100).
 * See Microchip p32mx350f256h.h — distinct from the compact map used for MX7.
 */
#define MX350_PORT0       0x86100
#define MX350_PORT_STRIDE 0x100
#define MX350_NPORTS      6

static inline int mx350_port_index(unsigned offset)
{
    if (offset < MX350_PORT0) {
        return -1;
    }
    offset -= MX350_PORT0;
    if (offset >= MX350_NPORTS * MX350_PORT_STRIDE) {
        return -1;
    }
    return (int) (offset / MX350_PORT_STRIDE);
}

static void mx350_port_write(pic32_t *s, unsigned offset, unsigned data)
{
    int port = mx350_port_index(offset);
    unsigned base = MX350_PORT0 + (unsigned) port * MX350_PORT_STRIDE;
    unsigned reg = offset - base;
    unsigned lat_word = base + 0x30;

    if (reg >= 0x20 && reg <= 0x2c) {
        s->iomem[lat_word >> 2] = write_op(s->iomem[lat_word >> 2], data, offset);
        pic32_gpio_write(s, port, s->iomem[lat_word >> 2]);
        return;
    }
    if (reg >= 0x30 && reg <= 0x3c) {
        s->iomem[lat_word >> 2] = write_op(s->iomem[lat_word >> 2], data, offset);
        pic32_gpio_write(s, port, s->iomem[lat_word >> 2]);
        return;
    }
    s->iomem[offset >> 2] = write_op(s->iomem[offset >> 2], data, offset);
}

/* Peripheral Pin Select (input + output), PIC32MX350 — see p32mx350f256h.h ~0xBF80FA00–0xBF80FCAF */
#define MX350_PPS_FIRST 0xfa00
#define MX350_PPS_LAST  0xfcff

static inline int mx350_pps_offset(unsigned offset)
{
    return offset >= MX350_PPS_FIRST && offset <= MX350_PPS_LAST;
}

static void mx350_pps_write(pic32_t *s, unsigned offset, unsigned data)
{
    s->iomem[offset >> 2] = write_op(s->iomem[offset >> 2], data, offset);
}

static void io_reset(pic32_t *s)
{
    int i;

    /*
     * Bus matrix control registers.
     */
    VALUE(BMXCON)    = 0x001f0041;      // Bus Matrix Control
    VALUE(BMXDKPBA)  = 0;               // Data RAM kernel program base address
    VALUE(BMXDUDBA)  = 0;               // Data RAM user data base address
    VALUE(BMXDUPBA)  = 0;               // Data RAM user program base address
    VALUE(BMXPUPBA)  = 0;               // Program Flash user program base address
    VALUE(BMXDRMSZ)  = 64 * 1024;       // Data RAM memory size: 64 KB
    VALUE(BMXPFMSZ)  = 256 * 1024;      // Program Flash memory size: 256 KB
    VALUE(BMXBOOTSZ) = 12 * 1024;       // Boot Flash size: 12 KB

    /*
     * Prefetch controller.
     */
    VALUE(CHECON) = 0x00000007;

    /*
     * System controller.
     */
    VALUE(OSCTUN) = 0;
    VALUE(DDPCON) = 0;
    VALUE(SYSKEY) = 0;
    VALUE(REFOCON) = 0;
    VALUE(REFOTRIM) = 0;
    VALUE(CFGCON) = 0;
    VALUE(RCON)   = 0;
    VALUE(RSWRST) = 0;
    s->syskey_unlock = 0;

    /*
     * Analog to digital converter.
     */
    VALUE(AD1CON1) = 0;                 // Control register 1
    VALUE(AD1CON2) = 0;                 // Control register 2
    VALUE(AD1CON3) = 0;                 // Control register 3
    VALUE(AD1CHS)  = 0;                 // Channel select
    VALUE(AD1CSSL) = 0;                 // Input scan selection
    VALUE(AD1PCFG) = 0;                 // Port configuration
    for (int i = 0; i < 16; i++) {
        VALUE((ADC1BUF0 + i * 0x10)) = 0;
    }

    /*
     * General purpose IO signals.
     * All pins are inputs, high, open drains and pullups disabled.
     * No interrupts on change.
     * Note: Port A is not available on the 64-pin TQFP package.
     */
    VALUE(TRISB) = 0xFFFF;              // Port B: mask of inputs
    VALUE(PORTB) = 0xFFFF;              // Port B: read inputs, write outputs
    VALUE(LATB)  = 0xFFFF;              // Port B: read/write outputs
    VALUE(ODCB)  = 0;                   // Port B: open drain configuration
    VALUE(TRISC) = 0xFFFF;              // Port C: mask of inputs
    VALUE(PORTC) = 0xFFFF;              // Port C: read inputs, write outputs
    VALUE(LATC)  = 0xFFFF;              // Port C: read/write outputs
    VALUE(ODCC)  = 0;                   // Port C: open drain configuration
    VALUE(TRISD) = 0xFFFF;              // Port D: mask of inputs
    VALUE(PORTD) = 0xFFFF;              // Port D: read inputs, write outputs
    VALUE(LATD)  = 0xFFFF;              // Port D: read/write outputs
    VALUE(ODCD)  = 0;                   // Port D: open drain configuration
    VALUE(TRISE) = 0xFFFF;              // Port E: mask of inputs
    VALUE(PORTE) = 0xFFFF;              // Port E: read inputs, write outputs
    VALUE(LATE)  = 0xFFFF;              // Port E: read/write outputs
    VALUE(ODCE)  = 0;                   // Port E: open drain configuration
    VALUE(TRISF) = 0xFFFF;              // Port F: mask of inputs
    VALUE(PORTF) = 0xFFFF;              // Port F: read inputs, write outputs
    VALUE(LATF)  = 0xFFFF;              // Port F: read/write outputs
    VALUE(ODCF)  = 0;                   // Port F: open drain configuration
    VALUE(TRISG) = 0xFFFF;              // Port G: mask of inputs
    VALUE(PORTG) = 0xFFFF;              // Port G: read inputs, write outputs
    VALUE(LATG)  = 0xFFFF;              // Port G: read/write outputs
    VALUE(ODCG)  = 0;                   // Port G: open drain configuration
    VALUE(CNCON) = 0;                   // Interrupt-on-change control
    VALUE(CNEN)  = 0;                   // Input change interrupt enable
    VALUE(CNPUE) = 0;                   // Input pin pull-up enable

    /* MX350 expanded GPIO (ANSEL/CN per port) at Microchip addresses. */
    for (int p = 0; p < MX350_NPORTS; p++) {
        unsigned b = MX350_PORT0 + p * MX350_PORT_STRIDE;

        s->iomem[(b + 0x00) >> 2] = 0;
        s->iomem[(b + 0x10) >> 2] = 0xFFFF; /* TRIS */
        s->iomem[(b + 0x20) >> 2] = 0xFFFF; /* PORT */
        s->iomem[(b + 0x30) >> 2] = 0xFFFF; /* LAT */
        s->iomem[(b + 0x40) >> 2] = 0;      /* ODC */
        s->iomem[(b + 0x50) >> 2] = 0;      /* CNPU */
        s->iomem[(b + 0x60) >> 2] = 0;      /* CNPD */
        s->iomem[(b + 0x70) >> 2] = 0;      /* CNCON */
        s->iomem[(b + 0x80) >> 2] = 0;      /* CNEN */
        s->iomem[(b + 0x90) >> 2] = 0;      /* CNSTAT */
    }

    /*
     * Reset UARTs. MX350F256H: U1, U2, U3 (host -serial stdio is wired to U3).
     */
    VALUE(U1MODE)  = 0;
    VALUE(U1STA)   = PIC32_USTA_RIDLE | PIC32_USTA_TRMT;
    VALUE(U1TXREG) = 0;
    VALUE(U1RXREG) = 0;
    VALUE(U1BRG)   = 0;
    VALUE(U2MODE)  = 0;
    VALUE(U2STA)   = PIC32_USTA_RIDLE | PIC32_USTA_TRMT;
    VALUE(U2TXREG) = 0;
    VALUE(U2RXREG) = 0;
    VALUE(U2BRG)   = 0;
    VALUE(U3MODE)  = 0;
    VALUE(U3STA)   = PIC32_USTA_RIDLE | PIC32_USTA_TRMT;
    VALUE(U3TXREG) = 0;
    VALUE(U3RXREG) = 0;
    VALUE(U3BRG)   = 0;

    /*
     * Reset SPI. MX350F256H has SPI1, SPI2, SPI3.
     */
    VALUE(SPI1CON)  = 0;
    VALUE(SPI1STAT) = PIC32_SPISTAT_SPITBE;     // Transmit buffer is empty
    VALUE(SPI1BRG)  = 0;

    VALUE(SPI2CON)  = 0;
    VALUE(SPI2STAT) = PIC32_SPISTAT_SPITBE;     // Transmit buffer is empty
    VALUE(SPI2BRG)  = 0;

    VALUE(SPI3CON)  = 0;
    VALUE(SPI3STAT) = PIC32_SPISTAT_SPITBE;     // Transmit buffer is empty
    VALUE(SPI3BRG)  = 0;

    for (i=0; i<NUM_SPI; i++) {
        s->spi[i].rfifo = 0;
        s->spi[i].wfifo = 0;
    }

    /*
     * Reset Watchdog timer.
     */
    VALUE(WDTCON) = 0;

    /*
     * Reset NVM controller (idle).
     */
    VALUE(NVMCON) = 0;
    VALUE(NVMKEY) = 0;
    VALUE(NVMADDR) = 0;
    VALUE(NVMDATA) = 0;
    VALUE(NVMSRCADDR) = 0;
    pic32mx3_nvm_reset(s);

    /*
     * Reset Comparator voltage reference.
     */
    VALUE(CVRCON) = 0;

    /*
     * Reset Comparators.
     */
    VALUE(CM1CON) = 0;
    VALUE(CM2CON) = 0;
    VALUE(CM3CON) = 0;
    VALUE(CMSTAT) = 0;

    /*
     * Reset Real-Time Clock and Calendar.
     */
    VALUE(RTCCON)   = 0;
    VALUE(RTCALRM)  = 0;
    VALUE(RTCTIME)  = 0;
    VALUE(RTCDATE)  = 0;
    VALUE(ALRMTIME) = 0;
    VALUE(ALRMDATE) = 0;

    /*
     * Reset Timers T1-T5.
     */
    VALUE(T1CON) = 0;
    VALUE(TMR1)  = 0;
    VALUE(PR1)   = 0xFFFF;

    VALUE(T2CON) = 0;
    VALUE(TMR2)  = 0;
    VALUE(PR2)   = 0xFFFF;

    VALUE(T3CON) = 0;
    VALUE(TMR3)  = 0;
    VALUE(PR3)   = 0xFFFF;

    VALUE(T4CON) = 0;
    VALUE(TMR4)  = 0;
    VALUE(PR4)   = 0xFFFF;

    VALUE(T5CON) = 0;
    VALUE(TMR5)  = 0;
    VALUE(PR5)   = 0xFFFF;

    pic32mx3_tmrs_recompute_all(s);

    /*
     * Reset Input Capture IC1-IC5.
     */
    VALUE(IC1CON) = 0;
    VALUE(IC1BUF) = 0;
    VALUE(IC2CON) = 0;
    VALUE(IC3CON) = 0;
    VALUE(IC4CON) = 0;
    VALUE(IC5CON) = 0;

    /*
     * Reset Output Compare OC1-OC5.
     */
    VALUE(OC1CON) = 0;  VALUE(OC1R) = 0;  VALUE(OC1RS) = 0;
    VALUE(OC2CON) = 0;  VALUE(OC2R) = 0;  VALUE(OC2RS) = 0;
    VALUE(OC3CON) = 0;  VALUE(OC3R) = 0;  VALUE(OC3RS) = 0;
    VALUE(OC4CON) = 0;  VALUE(OC4R) = 0;  VALUE(OC4RS) = 0;
    VALUE(OC5CON) = 0;  VALUE(OC5R) = 0;  VALUE(OC5RS) = 0;

    /*
     * Reset I2C modules I2C1 and I2C2.
     */
    VALUE(I2C1CON)  = 0;
    VALUE(I2C1STAT) = 0;
    VALUE(I2C1ADD)  = 0;
    VALUE(I2C1MSK)  = 0;
    VALUE(I2C1BRG)  = 0;
    VALUE(I2C1TRN)  = 0xFF;
    VALUE(I2C1RCV)  = 0;

    VALUE(I2C2CON)  = 0;
    VALUE(I2C2STAT) = 0;
    VALUE(I2C2ADD)  = 0;
    VALUE(I2C2MSK)  = 0;
    VALUE(I2C2BRG)  = 0;
    VALUE(I2C2TRN)  = 0xFF;
    VALUE(I2C2RCV)  = 0;

    /*
     * Reset Parallel Master Port.
     */
    VALUE(PMCON)  = 0;
    VALUE(PMMODE) = 0;
    VALUE(PMADDR) = 0;
    VALUE(PMDOUT) = 0;
    VALUE(PMDIN)  = 0;
    VALUE(PMAEN)  = 0;
    VALUE(PMSTAT) = 0;

    /*
     * Clear interrupt controller latched flags and enables.  On silicon, system
     * reset (including the RSWRST path) clears IFS/IEC; leaving them across
     * io_reset() caused stale T1IE/T3IE (etc.) to pair with new TMR IRQs right
     * after qemu_system_reset_request(), firing ISRs before guest init.
     */
    VALUE(IFS0) = 0;
    VALUE(IFS1) = 0;
    VALUE(IFS2) = 0;
    VALUE(IEC0) = 0;
    VALUE(IEC1) = 0;
    VALUE(IEC2) = 0;
    update_irq_status(s);

    s->cpu->env.eic_multivec = (VALUE(INTCON) & PIC32_INTCON_MVEC) != 0;
    s->cpu->env.eic_vector = 0;
}

static unsigned io_read32(pic32_t *s, unsigned offset, const char **namep)
{
    unsigned *bufp = &VALUE(offset);

    if (mx350_port_index(offset) >= 0) {
        *namep = "mx350.gpio";
        return s->iomem[offset >> 2];
    }
    if (mx350_pps_offset(offset)) {
        *namep = "mx350.pps";
        return s->iomem[offset >> 2];
    }

    /* PMD*, ANCFG, etc.: stub until modeled (gap before RCON at 0xF600). */
    if (offset >= 0xf270 && offset < 0xf600) {
        *namep = "mx350.sysstub";
        return s->iomem[offset >> 2];
    }

    switch (offset) {
    /*-------------------------------------------------------------------------
     * Bus matrix control registers.
     */
    STORAGE(BMXCON); break;     // Bus Matrix Control
    STORAGE(BMXDKPBA); break;   // Data RAM kernel program base address
    STORAGE(BMXDUDBA); break;   // Data RAM user data base address
    STORAGE(BMXDUPBA); break;   // Data RAM user program base address
    STORAGE(BMXPUPBA); break;   // Program Flash user program base address
    STORAGE(BMXDRMSZ); break;   // Data RAM memory size
    STORAGE(BMXPFMSZ); break;   // Program Flash memory size
    STORAGE(BMXBOOTSZ); break;  // Boot Flash size

    /*-------------------------------------------------------------------------
     * Interrupt controller registers.
     */
    STORAGE(INTCON); break;     // Interrupt Control
    STORAGE(INTSTAT); break;    // Interrupt Status
    STORAGE(IFS0); break;       // IFS(0..2) - Interrupt Flag Status
    STORAGE(IFS1); break;
    STORAGE(IFS2); break;
    STORAGE(IEC0); break;       // IEC(0..2) - Interrupt Enable Control
    STORAGE(IEC1); break;
    STORAGE(IEC2); break;
    STORAGE(IPC0); break;       // IPC(0..11) - Interrupt Priority Control
    STORAGE(IPC1); break;
    STORAGE(IPC2); break;
    STORAGE(IPC3); break;
    STORAGE(IPC4); break;
    STORAGE(IPC5); break;
    STORAGE(IPC6); break;
    STORAGE(IPC7); break;
    STORAGE(IPC8); break;
    STORAGE(IPC9); break;
    STORAGE(IPC10); break;
    STORAGE(IPC11); break;
    STORAGE(IPC12); break;

    /*-------------------------------------------------------------------------
     * Prefetch controller.
     */
    STORAGE(CHECON); break;     // Prefetch Control

    /*-------------------------------------------------------------------------
     * System controller.
     */
    STORAGE(OSCCON); break;     // Oscillator Control
    STORAGE(OSCTUN); break;     // Oscillator Tuning
    STORAGE(DDPCON); break;     // Debug Data Port Control
    STORAGE(DEVID); break;      // Device Identifier
    STORAGE(SYSKEY); break;     // System Key
    STORAGE(REFOCON); break;
    STORAGE(REFOCONCLR); *bufp = 0; break;
    STORAGE(REFOCONSET); *bufp = 0; break;
    STORAGE(REFOCONINV); *bufp = 0; break;
    STORAGE(REFOTRIM); break;
    STORAGE(REFOTRIMCLR); *bufp = 0; break;
    STORAGE(REFOTRIMSET); *bufp = 0; break;
    STORAGE(REFOTRIMINV); *bufp = 0; break;
    STORAGE(CFGCON); break;
    STORAGE(CFGCONCLR); *bufp = 0; break;
    STORAGE(CFGCONSET); *bufp = 0; break;
    STORAGE(CFGCONINV); *bufp = 0; break;
    STORAGE(RCON); break;       // Reset Control
    STORAGE(RSWRST);            // Software Reset
        if ((VALUE(RSWRST) & 1) && s->stop_on_reset) {
            exit(0);
        }
        break;

    /*-------------------------------------------------------------------------
     * DMA controller.
     */
    STORAGE(DMACON); break;     // DMA Control
    STORAGE(DMASTAT); break;    // DMA Status
    STORAGE(DMAADDR); break;    // DMA Address

    /*-------------------------------------------------------------------------
     * Analog to digital converter.
     */
    STORAGE(AD1CON1); break;    // Control register 1
    STORAGE(AD1CON2); break;    // Control register 2
    STORAGE(AD1CON3); break;    // Control register 3
    STORAGE(AD1CHS); break;     // Channel select
    STORAGE(AD1CSSL); break;    // Input scan selection
    STORAGE(AD1PCFG); break;    // Port configuration
    STORAGE(ADC1BUF0); break;   // Result words
    STORAGE(ADC1BUF1); break;
    STORAGE(ADC1BUF2); break;
    STORAGE(ADC1BUF3); break;
    STORAGE(ADC1BUF4); break;
    STORAGE(ADC1BUF5); break;
    STORAGE(ADC1BUF6); break;
    STORAGE(ADC1BUF7); break;
    STORAGE(ADC1BUF8); break;
    STORAGE(ADC1BUF9); break;
    STORAGE(ADC1BUFA); break;
    STORAGE(ADC1BUFB); break;
    STORAGE(ADC1BUFC); break;
    STORAGE(ADC1BUFD); break;
    STORAGE(ADC1BUFE); break;
    STORAGE(ADC1BUFF); break;

    /*--------------------------------------
     * USB registers.
     */
    STORAGE(U1OTGIR); break;    // OTG interrupt flags
    STORAGE(U1OTGIE); break;    // OTG interrupt enable
    STORAGE(U1OTGSTAT); break;  // Comparator and pin status
    STORAGE(U1OTGCON); break;   // Resistor and pin control
    STORAGE(U1PWRC); break;     // Power control
    STORAGE(U1IR); break;       // Pending interrupt
    STORAGE(U1IE); break;       // Interrupt enable
    STORAGE(U1EIR); break;      // Pending error interrupt
    STORAGE(U1EIE); break;      // Error interrupt enable
    STORAGE(U1STAT); break;     // Status FIFO
    STORAGE(U1CON); break;      // Control
    STORAGE(U1ADDR); break;     // Address
    STORAGE(U1BDTP1); break;    // Buffer descriptor table pointer 1
    STORAGE(U1FRML); break;     // Frame counter low
    STORAGE(U1FRMH); break;     // Frame counter high
    STORAGE(U1TOK); break;      // Host control
    STORAGE(U1SOF); break;      // SOF counter
    STORAGE(U1BDTP2); break;    // Buffer descriptor table pointer 2
    STORAGE(U1BDTP3); break;    // Buffer descriptor table pointer 3
    STORAGE(U1CNFG1); break;    // Debug and idle
    STORAGE(U1EP(0)); break;    // Endpoint control
    STORAGE(U1EP(1)); break;
    STORAGE(U1EP(2)); break;
    STORAGE(U1EP(3)); break;
    STORAGE(U1EP(4)); break;
    STORAGE(U1EP(5)); break;
    STORAGE(U1EP(6)); break;
    STORAGE(U1EP(7)); break;
    STORAGE(U1EP(8)); break;
    STORAGE(U1EP(9)); break;
    STORAGE(U1EP(10)); break;
    STORAGE(U1EP(11)); break;
    STORAGE(U1EP(12)); break;
    STORAGE(U1EP(13)); break;
    STORAGE(U1EP(14)); break;
    STORAGE(U1EP(15)); break;

    /*-------------------------------------------------------------------------
     * General purpose IO signals.
     * Note: Port A not available on 64-pin TQFP package.
     */
    STORAGE(TRISB); break;      // Port B: mask of inputs
    STORAGE(PORTB); break;      // Port B: read inputs
    STORAGE(LATB); break;       // Port B: read outputs
    STORAGE(ODCB); break;       // Port B: open drain configuration
    STORAGE(TRISC); break;      // Port C: mask of inputs
    STORAGE(PORTC); break;      // Port C: read inputs
    STORAGE(LATC); break;       // Port C: read outputs
    STORAGE(ODCC); break;       // Port C: open drain configuration
    STORAGE(TRISD); break;      // Port D: mask of inputs
    STORAGE(PORTD); break;      // Port D: read inputs
    STORAGE(LATD); break;       // Port D: read outputs
    STORAGE(ODCD); break;       // Port D: open drain configuration
    STORAGE(TRISE); break;      // Port E: mask of inputs
    STORAGE(PORTE); break;      // Port E: read inputs
    STORAGE(LATE); break;       // Port E: read outputs
    STORAGE(ODCE); break;       // Port E: open drain configuration
    STORAGE(TRISF); break;      // Port F: mask of inputs
    STORAGE(PORTF); break;      // Port F: read inputs
    STORAGE(LATF); break;       // Port F: read outputs
    STORAGE(ODCF); break;       // Port F: open drain configuration
    STORAGE(TRISG); break;      // Port G: mask of inputs
    STORAGE(PORTG); break;      // Port G: read inputs
    STORAGE(LATG); break;       // Port G: read outputs
    STORAGE(ODCG); break;       // Port G: open drain configuration
    STORAGE(CNCON); break;      // Interrupt-on-change control
    STORAGE(CNEN); break;       // Input change interrupt enable
    STORAGE(CNPUE); break;      // Input pin pull-up enable

    /*-------------------------------------------------------------------------
     * UART 1.
     */
    STORAGE(U1RXREG);                           // Receive data
        *bufp = pic32_uart_get_char(s, 0);
        break;
    STORAGE(U1BRG); break;                      // Baud rate
    STORAGE(U1MODE); break;                     // Mode
    STORAGE(U1STA);                             // Status and control
        pic32_uart_poll_status(s, 0);
        break;
    STORAGE(U1TXREG);   *bufp = 0; break;       // Transmit
    STORAGE(U1MODECLR); *bufp = 0; break;
    STORAGE(U1MODESET); *bufp = 0; break;
    STORAGE(U1MODEINV); *bufp = 0; break;
    STORAGE(U1STACLR);  *bufp = 0; break;
    STORAGE(U1STASET);  *bufp = 0; break;
    STORAGE(U1STAINV);  *bufp = 0; break;
    STORAGE(U1BRGCLR);  *bufp = 0; break;
    STORAGE(U1BRGSET);  *bufp = 0; break;
    STORAGE(U1BRGINV);  *bufp = 0; break;

    /*-------------------------------------------------------------------------
     * UART 2.
     */
    STORAGE(U2RXREG);                           // Receive data
        *bufp = pic32_uart_get_char(s, 1);
        break;
    STORAGE(U2BRG); break;                      // Baud rate
    STORAGE(U2MODE); break;                     // Mode
    STORAGE(U2STA);                             // Status and control
        pic32_uart_poll_status(s, 1);
        break;
    STORAGE(U2TXREG);   *bufp = 0; break;      // Transmit
    STORAGE(U2MODECLR); *bufp = 0; break;
    STORAGE(U2MODESET); *bufp = 0; break;
    STORAGE(U2MODEINV); *bufp = 0; break;
    STORAGE(U2STACLR);  *bufp = 0; break;
    STORAGE(U2STASET);  *bufp = 0; break;
    STORAGE(U2STAINV);  *bufp = 0; break;
    STORAGE(U2BRGCLR);  *bufp = 0; break;
    STORAGE(U2BRGSET);  *bufp = 0; break;
    STORAGE(U2BRGINV);  *bufp = 0; break;

    /*-------------------------------------------------------------------------
     * UART 3.
     */
    STORAGE(U3RXREG);                           // Receive data
        *bufp = pic32_uart_get_char(s, 2);
        break;
    STORAGE(U3BRG); break;                      // Baud rate
    STORAGE(U3MODE); break;                     // Mode
    STORAGE(U3STA);                             // Status and control
        pic32_uart_poll_status(s, 2);
        break;
    STORAGE(U3TXREG);   *bufp = 0; break;       // Transmit
    STORAGE(U3MODECLR); *bufp = 0; break;
    STORAGE(U3MODESET); *bufp = 0; break;
    STORAGE(U3MODEINV); *bufp = 0; break;
    STORAGE(U3STACLR);  *bufp = 0; break;
    STORAGE(U3STASET);  *bufp = 0; break;
    STORAGE(U3STAINV);  *bufp = 0; break;
    STORAGE(U3BRGCLR);  *bufp = 0; break;
    STORAGE(U3BRGSET);  *bufp = 0; break;
    STORAGE(U3BRGINV);  *bufp = 0; break;

    /*-------------------------------------------------------------------------
     * SPI 1.
     */
    STORAGE(SPI1CON); break;                    // Control
    STORAGE(SPI1CONCLR); *bufp = 0; break;
    STORAGE(SPI1CONSET); *bufp = 0; break;
    STORAGE(SPI1CONINV); *bufp = 0; break;
    STORAGE(SPI1STAT); break;                   // Status
    STORAGE(SPI1STATCLR); *bufp = 0; break;
    STORAGE(SPI1STATSET); *bufp = 0; break;
    STORAGE(SPI1STATINV); *bufp = 0; break;
    STORAGE(SPI1BUF);                           // Buffer
        *bufp = pic32_spi_readbuf(s, 0);
        break;
    STORAGE(SPI1BRG); break;                    // Baud rate
    STORAGE(SPI1BRGCLR); *bufp = 0; break;
    STORAGE(SPI1BRGSET); *bufp = 0; break;
    STORAGE(SPI1BRGINV); *bufp = 0; break;

    /*-------------------------------------------------------------------------
     * SPI 2.
     */
    STORAGE(SPI2CON); break;                    // Control
    STORAGE(SPI2CONCLR); *bufp = 0; break;
    STORAGE(SPI2CONSET); *bufp = 0; break;
    STORAGE(SPI2CONINV); *bufp = 0; break;
    STORAGE(SPI2STAT); break;                   // Status
    STORAGE(SPI2STATCLR); *bufp = 0; break;
    STORAGE(SPI2STATSET); *bufp = 0; break;
    STORAGE(SPI2STATINV); *bufp = 0; break;
    STORAGE(SPI2BUF);                           // Buffer
        *bufp = pic32_spi_readbuf(s, 1);
        break;
    STORAGE(SPI2BRG); break;                    // Baud rate
    STORAGE(SPI2BRGCLR); *bufp = 0; break;
    STORAGE(SPI2BRGSET); *bufp = 0; break;
    STORAGE(SPI2BRGINV); *bufp = 0; break;

    /*-------------------------------------------------------------------------
     * SPI 3.
     */
    STORAGE(SPI3CON); break;                    // Control
    STORAGE(SPI3CONCLR); *bufp = 0; break;
    STORAGE(SPI3CONSET); *bufp = 0; break;
    STORAGE(SPI3CONINV); *bufp = 0; break;
    STORAGE(SPI3STAT); break;                   // Status
    STORAGE(SPI3STATCLR); *bufp = 0; break;
    STORAGE(SPI3STATSET); *bufp = 0; break;
    STORAGE(SPI3STATINV); *bufp = 0; break;
    STORAGE(SPI3BUF);                           // Buffer
        *bufp = pic32_spi_readbuf(s, 2);
        break;
    STORAGE(SPI3BRG); break;                    // Baud rate
    STORAGE(SPI3BRGCLR); *bufp = 0; break;
    STORAGE(SPI3BRGSET); *bufp = 0; break;
    STORAGE(SPI3BRGINV); *bufp = 0; break;

    /*-------------------------------------------------------------------------
     * Watchdog timer.
     */
    STORAGE(WDTCON); break;

    /*-------------------------------------------------------------------------
     * Flash NVM control (word/row program, page erase — see pic32mx3_nvm.c).
     */
    STORAGE(NVMCON); break;
    STORAGE(NVMKEY); break;
    STORAGE(NVMADDR); break;
    STORAGE(NVMDATA); break;
    STORAGE(NVMSRCADDR); break;

    /*-------------------------------------------------------------------------
     * Comparator voltage reference.
     */
    STORAGE(CVRCON); break;

    /*-------------------------------------------------------------------------
     * Comparators.
     */
    STORAGE(CM1CON); break;
    STORAGE(CM2CON); break;
    STORAGE(CM3CON); break;
    STORAGE(CMSTAT); break;

    /*-------------------------------------------------------------------------
     * Real-Time Clock and Calendar.
     */
    STORAGE(RTCCON); break;
    STORAGE(RTCALRM); break;
    STORAGE(RTCTIME); break;
    STORAGE(RTCDATE); break;
    STORAGE(ALRMTIME); break;
    STORAGE(ALRMDATE); break;

    /*-------------------------------------------------------------------------
     * Timers T1-T5.
     */
    STORAGE(T1CON); break;
    STORAGE(TMR1); break;
    STORAGE(PR1); break;
    STORAGE(T2CON); break;
    STORAGE(TMR2); break;
    STORAGE(PR2); break;
    STORAGE(T3CON); break;
    STORAGE(TMR3); break;
    STORAGE(PR3); break;
    STORAGE(T4CON); break;
    STORAGE(TMR4); break;
    STORAGE(PR4); break;
    STORAGE(T5CON); break;
    STORAGE(TMR5); break;
    STORAGE(PR5); break;

    /*-------------------------------------------------------------------------
     * Input Capture IC1-IC5.
     */
    STORAGE(IC1CON); break;
    STORAGE(IC1BUF); break;
    STORAGE(IC2CON); break;
    STORAGE(IC2BUF); break;
    STORAGE(IC3CON); break;
    STORAGE(IC3BUF); break;
    STORAGE(IC4CON); break;
    STORAGE(IC4BUF); break;
    STORAGE(IC5CON); break;
    STORAGE(IC5BUF); break;

    /*-------------------------------------------------------------------------
     * Output Compare OC1-OC5.
     */
    STORAGE(OC1CON); break;
    STORAGE(OC1R); break;
    STORAGE(OC1RS); break;
    STORAGE(OC2CON); break;
    STORAGE(OC2R); break;
    STORAGE(OC2RS); break;
    STORAGE(OC3CON); break;
    STORAGE(OC3R); break;
    STORAGE(OC3RS); break;
    STORAGE(OC4CON); break;
    STORAGE(OC4R); break;
    STORAGE(OC4RS); break;
    STORAGE(OC5CON); break;
    STORAGE(OC5R); break;
    STORAGE(OC5RS); break;

    /*-------------------------------------------------------------------------
     * I2C 1.
     */
    STORAGE(I2C1CON); break;
    STORAGE(I2C1STAT); break;
    STORAGE(I2C1ADD); break;
    STORAGE(I2C1MSK); break;
    STORAGE(I2C1BRG); break;
    STORAGE(I2C1TRN); break;
    STORAGE(I2C1RCV); break;

    /*-------------------------------------------------------------------------
     * I2C 2.
     */
    STORAGE(I2C2CON); break;
    STORAGE(I2C2STAT); break;
    STORAGE(I2C2ADD); break;
    STORAGE(I2C2MSK); break;
    STORAGE(I2C2BRG); break;
    STORAGE(I2C2TRN); break;
    STORAGE(I2C2RCV); break;

    /*-------------------------------------------------------------------------
     * Parallel Master Port.
     */
    STORAGE(PMCON); break;
    STORAGE(PMMODE); break;
    STORAGE(PMADDR); break;
    STORAGE(PMDOUT); break;
    STORAGE(PMDIN); break;
    STORAGE(PMAEN); break;
    STORAGE(PMSTAT); break;

    default:
        printf("--- Read 1f8%05x: peripheral register not supported\n",
            offset);
        if (TRACE)
            fprintf(qemu_logfile, "--- Read 1f8%05x: peripheral register not supported\n",
                offset);
        exit(1);
    }
    return *bufp;
}

/*
 * Minimal ADC1 model for pic32mx3-generic: manual conversion when SAMP goes 1->0
 * with ADON set. Fills ADC1BUF0 with a deterministic 10-bit pattern per CH0SA
 * (guest-visible; uart-cli "adc all" regression).
 */
#define AD1CON1_DONE  0x00000001u
#define AD1CON1_SAMP  0x00000002u
#define AD1CON1_ADON  0x00008000u

static void pic32mx3_adc_after_ad1con1_write(pic32_t *s, unsigned prev, unsigned newv)
{
    if (!(newv & AD1CON1_ADON)) {
        return;
    }
    /* Begin sampling: clear DONE (matches silicon when SAMP is set). */
    if (!(prev & AD1CON1_SAMP) && (newv & AD1CON1_SAMP)) {
        VALUE(AD1CON1) = newv & ~AD1CON1_DONE;
        return;
    }
    /* End sampling -> conversion done (instant in QEMU). */
    if ((prev & AD1CON1_SAMP) && !(newv & AD1CON1_SAMP)) {
        unsigned ch = (VALUE(AD1CHS) >> 16) & 0x1fu;
        unsigned res = (ch * 67u + 11u) & 0x3ffu;

        VALUE(ADC1BUF0) = res;
        VALUE(AD1CON1) = newv | AD1CON1_DONE;
        irq_raise(s, 28); /* IFS0 AD1IF */
    }
}

/*
 * Parallel Master Port (stub): a PMDOUT write with PMP enabled and master mode
 * 1/2 completes immediately — PMDIN gets a deterministic XOR pattern, BUSY
 * clears, and PMPIF is raised if IRQM requests interrupt-on-cycle-end.
 */
static void pic32mx3_pmp_after_pmdout_write(pic32_t *s)
{
    unsigned pmc = VALUE(PMCON);
    unsigned pmm = VALUE(PMMODE);
    unsigned dout;
    unsigned mode = pmm & PIC32_PMMODE_MODE;

    if ((pmc & PIC32_PMCON_ON) == 0) {
        return;
    }
    if (mode != PIC32_PMMODE_MODE_MAST1 && mode != PIC32_PMMODE_MODE_MAST2) {
        return;
    }
    dout = VALUE(PMDOUT) & 0xffffu;
    if (pmm & PIC32_PMMODE_MODE16) {
        VALUE(PMDIN) = dout ^ 0xa5a5u;
    } else {
        VALUE(PMDIN) = (dout & 0xffu) ^ 0x5au;
    }
    VALUE(PMMODE) &= ~PIC32_PMMODE_BUSY;
    if (pmm & PIC32_PMMODE_IRQM_END) {
        irq_raise(s, 51); /* IFS1 PMPIF */
    }
}

/* IC1: ON + non-zero ICM — instant capture from TMR2/TMR3 (ICTMR), IC1IF (IFS0 bit 6). */
#define IC1CON_ON       0x8000u
#define IC1CON_ICM_M    0x0007u
#define IC1CON_ICTMR    0x0080u

static void pic32mx3_ic1con_after_write(pic32_t *s, unsigned newv)
{
    unsigned icm = newv & IC1CON_ICM_M;

    if (!(newv & IC1CON_ON) || icm == 0) {
        return;
    }
    {
        unsigned tmr = (newv & IC1CON_ICTMR) ? VALUE(TMR3) : VALUE(TMR2);

        VALUE(IC1BUF) = tmr & 0xffffu;
    }
    irq_raise(s, 6); /* IC1IF */
}

static void pic32mx3_cm1con_after_write(pic32_t *s)
{
    if (VALUE(CM1CON) & 0x8000u) {
        VALUE(CMSTAT) |= 1u; /* C1OUT */
        irq_raise(s, 32);    /* IFS1 CMP1IF */
    } else {
        VALUE(CMSTAT) &= ~1u;
    }
}

static void pic32mx3_cm2con_after_write(pic32_t *s)
{
    if (VALUE(CM2CON) & 0x8000u) {
        VALUE(CMSTAT) |= 2u; /* C2OUT */
        irq_raise(s, 33);    /* IFS1 CMP2IF */
    } else {
        VALUE(CMSTAT) &= ~2u;
    }
}

/* I2C TRN write with I2CxCON ON: loopback into RCV + master IF (IFS1 43 / 61). */
static void pic32mx3_i2c_trn_write(pic32_t *s, int port, unsigned data)
{
    unsigned con = port == 0 ? VALUE(I2C1CON) : VALUE(I2C2CON);
    unsigned xorv = port == 0 ? 0xaaU : 0x55U;
    unsigned b = data & 0xffu;

    if (!(con & 0x8000u)) {
        return;
    }
    if (port == 0) {
        VALUE(I2C1RCV) = b ^ xorv;
        irq_raise(s, 43);
    } else {
        VALUE(I2C2RCV) = b ^ xorv;
        irq_raise(s, 61);
    }
}

static void io_write32(pic32_t *s, unsigned offset, unsigned data, const char **namep)
{
    unsigned *bufp = &VALUE(offset);

    if (mx350_port_index(offset) >= 0) {
        *namep = "mx350.gpio";
        mx350_port_write(s, offset, data);
        return;
    }
    if (mx350_pps_offset(offset)) {
        *namep = "mx350.pps";
        mx350_pps_write(s, offset, data);
        return;
    }

    if (offset >= 0xf270 && offset < 0xf600) {
        *namep = "mx350.sysstub";
        s->iomem[offset >> 2] = write_op(s->iomem[offset >> 2], data, offset);
        return;
    }

    switch (offset) {
    /*-------------------------------------------------------------------------
     * Bus matrix control registers.
     */
    WRITEOP(BMXCON); return;    // Bus Matrix Control
    STORAGE(BMXDKPBA); break;   // Data RAM kernel program base address
    STORAGE(BMXDUDBA); break;   // Data RAM user data base address
    STORAGE(BMXDUPBA); break;   // Data RAM user program base address
    STORAGE(BMXPUPBA); break;   // Program Flash user program base address
    READONLY(BMXDRMSZ);         // Data RAM memory size
    READONLY(BMXPFMSZ);         // Program Flash memory size
    READONLY(BMXBOOTSZ);        // Boot Flash size

    /*-------------------------------------------------------------------------
     * Interrupt controller registers.
     */
    case INTCON:
    case INTCON + 4:
    case INTCON + 8:
    case INTCON + 12:
        *namep = "INTCON";
        VALUE(INTCON) = write_op(VALUE(INTCON), data, offset);
        s->cpu->env.eic_multivec = (VALUE(INTCON) & PIC32_INTCON_MVEC) != 0;
        return;
    READONLY(INTSTAT);          // Interrupt Status
    WRITEOP(IPTMR);  return;    // Temporal Proximity Timer
    WRITEOP(IFS0); goto irq;    // IFS(0..2) - Interrupt Flag Status
    WRITEOP(IFS1); goto irq;
    WRITEOP(IFS2); goto irq;
    WRITEOP(IEC0); goto irq;    // IEC(0..2) - Interrupt Enable Control
    case IEC1:
    case IEC1 + 4:              /* CLR */
    case IEC1 + 8:              /* SET */
    case IEC1 + 12:             /* INV */
        *namep = "IEC1";
        {
            uint32_t prev = VALUE(IEC1);
            VALUE(IEC1) = write_op(prev, data, offset);
            {
                uint32_t nw = VALUE(IEC1);

                pic32_uart_on_tx_ie_enabled(s, 0, prev, nw);
                pic32_uart_on_tx_ie_enabled(s, 1, prev, nw);
            }
        }
        goto irq;
    case IEC2:
    case IEC2 + 4:              /* CLR */
    case IEC2 + 8:              /* SET */
    case IEC2 + 12:             /* INV */
        *namep = "IEC2";
        {
            uint32_t prev = VALUE(IEC2);
            VALUE(IEC2) = write_op(prev, data, offset);
            pic32_uart_on_tx_ie_enabled(s, 2, prev, VALUE(IEC2));
        }
        goto irq;
    WRITEOP(IPC0); goto irq;    // IPC(0..11) - Interrupt Priority Control
    WRITEOP(IPC1); goto irq;
    WRITEOP(IPC2); goto irq;
    WRITEOP(IPC3); goto irq;
    WRITEOP(IPC4); goto irq;
    WRITEOP(IPC5); goto irq;
    WRITEOP(IPC6); goto irq;
    WRITEOP(IPC7); goto irq;
    WRITEOP(IPC8); goto irq;
    WRITEOP(IPC9); goto irq;
    WRITEOP(IPC10); goto irq;
    WRITEOP(IPC11); goto irq;
    WRITEOP(IPC12);
irq:    update_irq_status(s);
        return;

    /*-------------------------------------------------------------------------
     * Prefetch controller.
     */
    WRITEOP(CHECON); return;    // Prefetch Control

    /*-------------------------------------------------------------------------
     * System controller.
     */
    WRITEOPR(OSCCON, PIC32_OSCCON_UNUSED); break; // Oscillator Control
    WRITEOPR(OSCTUN, PIC32_OSCTUN_UNUSED); break; // Oscillator Tuning
    STORAGE(DDPCON); break;     // Debug Data Port Control
    READONLY(DEVID);            // Device Identifier
    STORAGE(SYSKEY);            // System Key
        /* Unlock state machine: compare the write data (register is old until *bufp). */
        if (data == 0) {
            s->syskey_unlock = 0;
        } else if (s->syskey_unlock == 0 && data == 0xaa996655) {
            s->syskey_unlock = 1;
        } else if (s->syskey_unlock == 1 && data == 0x556699aa) {
            s->syskey_unlock = 2;
        } else {
            s->syskey_unlock = 0;
        }
        break;
    WRITEOP(REFOCON); return;       /* Reference clock output (stub) */
    WRITEOP(REFOTRIM); return;
    WRITEOP(CFGCON); return;        /* Configuration control (stub) */
    WRITEOPR(RCON, PIC32_RCON_UNUSED); break;       // Reset Control
    WRITEOP(RSWRST);            // Software Reset
        if (s->syskey_unlock == 2 && (VALUE(RSWRST) & 1)) {
            /* Reset CPU. */
            qemu_system_reset_request();

            /* Reset all devices */
            io_reset(s);
            pic32_sdcard_reset(s);
        }
        break;

    /*-------------------------------------------------------------------------
     * DMA controller.
     */
    WRITEOP(DMACON); return;    // DMA Control
    STORAGE(DMASTAT); break;    // DMA Status
    STORAGE(DMAADDR); break;    // DMA Address

    /*-------------------------------------------------------------------------
     * Analog to digital converter.
     */
    case AD1CON1: *namep = "AD1CON1"; goto op_AD1CON1;
    case AD1CON1CLR: *namep = "AD1CON1CLR"; goto op_AD1CON1;
    case AD1CON1SET: *namep = "AD1CON1SET"; goto op_AD1CON1;
    case AD1CON1INV: *namep = "AD1CON1INV"; op_AD1CON1: {
            unsigned prev = VALUE(AD1CON1);

            VALUE(AD1CON1) = write_op(prev, data, offset);
            pic32mx3_adc_after_ad1con1_write(s, prev, VALUE(AD1CON1));
        }
        return;
    WRITEOP(AD1CON2); return;   // Control register 2
    WRITEOP(AD1CON3); return;   // Control register 3
    WRITEOP(AD1CHS); return;    // Channel select
    WRITEOP(AD1CSSL); return;   // Input scan selection
    WRITEOP(AD1PCFG); return;   // Port configuration
    READONLY(ADC1BUF0);         // Result words
    READONLY(ADC1BUF1);
    READONLY(ADC1BUF2);
    READONLY(ADC1BUF3);
    READONLY(ADC1BUF4);
    READONLY(ADC1BUF5);
    READONLY(ADC1BUF6);
    READONLY(ADC1BUF7);
    READONLY(ADC1BUF8);
    READONLY(ADC1BUF9);
    READONLY(ADC1BUFA);
    READONLY(ADC1BUFB);
    READONLY(ADC1BUFC);
    READONLY(ADC1BUFD);
    READONLY(ADC1BUFE);
    READONLY(ADC1BUFF);

    /*--------------------------------------
     * USB registers.
     */
    STORAGE(U1OTGIR);           // OTG interrupt flags
        VALUE(U1OTGIR) = 0;
        return;
    STORAGE(U1OTGIE); break;    // OTG interrupt enable
    READONLY(U1OTGSTAT);        // Comparator and pin status
    STORAGE(U1OTGCON); break;   // Resistor and pin control
    STORAGE(U1PWRC); break;     // Power control
    STORAGE(U1IR);              // Pending interrupt
        VALUE(U1IR) = 0;
        return;
    STORAGE(U1IE); break;       // Interrupt enable
    STORAGE(U1EIR);             // Pending error interrupt
        VALUE(U1EIR) = 0;
        return;
    STORAGE(U1EIE); break;      // Error interrupt enable
    READONLY(U1STAT);           // Status FIFO
    STORAGE(U1CON); break;      // Control
    STORAGE(U1ADDR); break;     // Address
    STORAGE(U1BDTP1); break;    // Buffer descriptor table pointer 1
    READONLY(U1FRML);           // Frame counter low
    READONLY(U1FRMH);           // Frame counter high
    STORAGE(U1TOK); break;      // Host control
    STORAGE(U1SOF); break;      // SOF counter
    STORAGE(U1BDTP2); break;    // Buffer descriptor table pointer 2
    STORAGE(U1BDTP3); break;    // Buffer descriptor table pointer 3
    STORAGE(U1CNFG1); break;    // Debug and idle
    STORAGE(U1EP(0)); break;    // Endpoint control
    STORAGE(U1EP(1)); break;
    STORAGE(U1EP(2)); break;
    STORAGE(U1EP(3)); break;
    STORAGE(U1EP(4)); break;
    STORAGE(U1EP(5)); break;
    STORAGE(U1EP(6)); break;
    STORAGE(U1EP(7)); break;
    STORAGE(U1EP(8)); break;
    STORAGE(U1EP(9)); break;
    STORAGE(U1EP(10)); break;
    STORAGE(U1EP(11)); break;
    STORAGE(U1EP(12)); break;
    STORAGE(U1EP(13)); break;
    STORAGE(U1EP(14)); break;
    STORAGE(U1EP(15)); break;

    /*-------------------------------------------------------------------------
     * General purpose IO signals.
     * Note: Port A not available on 64-pin TQFP package.
     * GPIO port indices: B=0, C=1, D=2, E=3, F=4, G=5
     */
    WRITEOP(TRISB); return;         // Port B: mask of inputs
    WRITEOPX(PORTB, LATB);          // Port B: write outputs
    WRITEOP(LATB);                  // Port B: write outputs
        pic32_gpio_write(s, 0, VALUE(LATB));
        return;
    WRITEOP(ODCB); return;          // Port B: open drain configuration
    WRITEOP(TRISC); return;         // Port C: mask of inputs
    WRITEOPX(PORTC, LATC);          // Port C: write outputs
    WRITEOP(LATC);                  // Port C: write outputs
        pic32_gpio_write(s, 1, VALUE(LATC));
        return;
    WRITEOP(ODCC); return;          // Port C: open drain configuration
    WRITEOP(TRISD); return;         // Port D: mask of inputs
    WRITEOPX(PORTD, LATD);          // Port D: write outputs
    WRITEOP(LATD);                  // Port D: write outputs
        pic32_gpio_write(s, 2, VALUE(LATD));
        return;
    WRITEOP(ODCD); return;          // Port D: open drain configuration
    WRITEOP(TRISE); return;         // Port E: mask of inputs
    WRITEOPX(PORTE, LATE);          // Port E: write outputs
    WRITEOP(LATE);                  // Port E: write outputs
        pic32_gpio_write(s, 3, VALUE(LATE));
        return;
    WRITEOP(ODCE); return;          // Port E: open drain configuration
    WRITEOP(TRISF); return;         // Port F: mask of inputs
    WRITEOPX(PORTF, LATF);          // Port F: write outputs
    WRITEOP(LATF);                  // Port F: write outputs
        pic32_gpio_write(s, 4, VALUE(LATF));
        return;
    WRITEOP(ODCF); return;          // Port F: open drain configuration
    WRITEOP(TRISG); return;         // Port G: mask of inputs
    WRITEOPX(PORTG, LATG);          // Port G: write outputs
    WRITEOP(LATG);                  // Port G: write outputs
        pic32_gpio_write(s, 5, VALUE(LATG));
        return;
    WRITEOP(ODCG); return;          // Port G: open drain configuration
    WRITEOP(CNCON); return;         // Interrupt-on-change control
    WRITEOP(CNEN); return;          // Input change interrupt enable
    WRITEOP(CNPUE); return;         // Input pin pull-up enable

    /*-------------------------------------------------------------------------
     * UART 1.
     */
    STORAGE(U1TXREG);                               // Transmit
        pic32_uart_put_char(s, 0, data);
        break;
    WRITEOP(U1MODE);                                // Mode
        pic32_uart_update_mode(s, 0);
        return;
    WRITEOPR(U1STA,                                 // Status and control
        PIC32_USTA_URXDA | PIC32_USTA_FERR | PIC32_USTA_PERR |
        PIC32_USTA_RIDLE | PIC32_USTA_TRMT | PIC32_USTA_UTXBF);
        pic32_uart_update_status(s, 0);
        return;
    WRITEOP(U1BRG); return;                         // Baud rate
    READONLY(U1RXREG);                              // Receive

    /*-------------------------------------------------------------------------
     * UART 2.
     */
    STORAGE(U2TXREG);                               // Transmit
        pic32_uart_put_char(s, 1, data);
        break;
    WRITEOP(U2MODE);                                // Mode
        pic32_uart_update_mode(s, 1);
        return;
    WRITEOPR(U2STA,                                 // Status and control
        PIC32_USTA_URXDA | PIC32_USTA_FERR | PIC32_USTA_PERR |
        PIC32_USTA_RIDLE | PIC32_USTA_TRMT | PIC32_USTA_UTXBF);
        pic32_uart_update_status(s, 1);
        return;
    WRITEOP(U2BRG); return;                         // Baud rate
    READONLY(U2RXREG);                              // Receive

    /*-------------------------------------------------------------------------
     * UART 3.
     */
    STORAGE(U3TXREG);                               // Transmit
        pic32_uart_put_char(s, 2, data);
        break;
    WRITEOP(U3MODE);                                // Mode
        pic32_uart_update_mode(s, 2);
        return;
    WRITEOPR(U3STA,                                 // Status and control
        PIC32_USTA_URXDA | PIC32_USTA_FERR | PIC32_USTA_PERR |
        PIC32_USTA_RIDLE | PIC32_USTA_TRMT | PIC32_USTA_UTXBF);
        pic32_uart_update_status(s, 2);
        return;
    WRITEOP(U3BRG); return;                         // Baud rate
    READONLY(U3RXREG);                              // Receive

    /*-------------------------------------------------------------------------
     * SPI.
     */
    WRITEOP(SPI1CON);                               // Control
        pic32_spi_control(s, 0);
        return;
    WRITEOPR(SPI1STAT, ~PIC32_SPISTAT_SPIROV);      // Status
        return;                                     // Only ROV bit is writable
    STORAGE(SPI1BUF);                               // Buffer
        pic32_spi_writebuf(s, 0, data);
        return;
    WRITEOP(SPI1BRG); return;                       // Baud rate
    WRITEOP(SPI2CON);                               // Control
        pic32_spi_control(s, 1);
        return;
    WRITEOPR(SPI2STAT, ~PIC32_SPISTAT_SPIROV);      // Status
        return;                                     // Only ROV bit is writable
    STORAGE(SPI2BUF);                               // Buffer
        pic32_spi_writebuf(s, 1, data);
        return;
    WRITEOP(SPI2BRG); return;                       // Baud rate
    WRITEOP(SPI3CON);                               // Control
        pic32_spi_control(s, 2);
        return;
    WRITEOPR(SPI3STAT, ~PIC32_SPISTAT_SPIROV);      // Status
        return;                                     // Only ROV bit is writable
    STORAGE(SPI3BUF);                               // Buffer
        pic32_spi_writebuf(s, 2, data);
        return;
    WRITEOP(SPI3BRG); return;                       // Baud rate

    /*-------------------------------------------------------------------------
     * Watchdog timer.
     */
    WRITEOP(WDTCON); return;

    /*-------------------------------------------------------------------------
     * Flash NVM control.
     */
    case NVMKEY:
    case NVMKEY + 4:
    case NVMKEY + 8:
    case NVMKEY + 12:
        *namep = "NVMKEY";
        pic32mx3_nvm_key_write(s, write_op(VALUE(NVMKEY), data, offset));
        VALUE(NVMKEY) = 0;
        return;
    WRITEOP(NVMADDR); return;
    WRITEOP(NVMDATA); return;
    WRITEOP(NVMSRCADDR); return;
    case NVMCON:
    case NVMCONCLR:
    case NVMCONSET:
    case NVMCONINV:
        *namep = "NVMCON";
        {
            uint32_t oldc = VALUE(NVMCON);
            uint32_t newc = write_op(oldc, data, offset);

            newc = pic32mx3_nvm_nvmcon_write(s, oldc, newc);
            VALUE(NVMCON) = newc;
        }
        return;

    /*-------------------------------------------------------------------------
     * Comparator voltage reference.
     */
    WRITEOP(CVRCON); return;

    /*-------------------------------------------------------------------------
     * Comparators.
     */
    case CM1CON:
    case CM1CONCLR:
    case CM1CONSET:
    case CM1CONINV:
        *namep = "CM1CON";
        VALUE(CM1CON) = write_op(VALUE(CM1CON), data, offset);
        pic32mx3_cm1con_after_write(s);
        return;
    case CM2CON:
    case CM2CONCLR:
    case CM2CONSET:
    case CM2CONINV:
        *namep = "CM2CON";
        VALUE(CM2CON) = write_op(VALUE(CM2CON), data, offset);
        pic32mx3_cm2con_after_write(s);
        return;
    WRITEOP(CM3CON); return;
    WRITEOP(CMSTAT); return;

    /*-------------------------------------------------------------------------
     * Real-Time Clock and Calendar.
     */
    WRITEOP(RTCCON); return;
    WRITEOP(RTCALRM); return;
    WRITEOP(RTCTIME); return;
    WRITEOP(RTCDATE); return;
    WRITEOP(ALRMTIME); return;
    WRITEOP(ALRMDATE); return;

    /*-------------------------------------------------------------------------
     * Timers T1-T5.
     */
    case T1CON:
    case T1CONCLR:
    case T1CONSET:
    case T1CONINV:
        *namep = "T1CON";
        VALUE(T1CON) = write_op(VALUE(T1CON), data, offset);
        pic32mx3_tmr_recompute(s, 0);
        return;
    case TMR1:
    case TMR1CLR:
    case TMR1SET:
    case TMR1INV:
        *namep = "TMR1";
        VALUE(TMR1) = write_op(VALUE(TMR1), data, offset);
        pic32mx3_tmr_recompute(s, 0);
        return;
    case PR1:
    case PR1CLR:
    case PR1SET:
    case PR1INV:
        *namep = "PR1";
        VALUE(PR1) = write_op(VALUE(PR1), data, offset);
        pic32mx3_tmr_recompute(s, 0);
        return;
    case T2CON:
    case T2CONCLR:
    case T2CONSET:
    case T2CONINV:
        *namep = "T2CON";
        VALUE(T2CON) = write_op(VALUE(T2CON), data, offset);
        pic32mx3_tmr_recompute(s, 1);
        return;
    case TMR2:
    case TMR2CLR:
    case TMR2SET:
    case TMR2INV:
        *namep = "TMR2";
        VALUE(TMR2) = write_op(VALUE(TMR2), data, offset);
        pic32mx3_tmr_recompute(s, 1);
        return;
    case PR2:
    case PR2CLR:
    case PR2SET:
    case PR2INV:
        *namep = "PR2";
        VALUE(PR2) = write_op(VALUE(PR2), data, offset);
        pic32mx3_tmr_recompute(s, 1);
        return;
    case T3CON:
    case T3CONCLR:
    case T3CONSET:
    case T3CONINV:
        *namep = "T3CON";
        VALUE(T3CON) = write_op(VALUE(T3CON), data, offset);
        pic32mx3_tmr_recompute(s, 2);
        return;
    case TMR3:
    case TMR3CLR:
    case TMR3SET:
    case TMR3INV:
        *namep = "TMR3";
        VALUE(TMR3) = write_op(VALUE(TMR3), data, offset);
        pic32mx3_tmr_recompute(s, 2);
        return;
    case PR3:
    case PR3CLR:
    case PR3SET:
    case PR3INV:
        *namep = "PR3";
        VALUE(PR3) = write_op(VALUE(PR3), data, offset);
        pic32mx3_tmr_recompute(s, 2);
        return;
    case T4CON:
    case T4CONCLR:
    case T4CONSET:
    case T4CONINV:
        *namep = "T4CON";
        VALUE(T4CON) = write_op(VALUE(T4CON), data, offset);
        pic32mx3_tmr_recompute(s, 3);
        return;
    case TMR4:
    case TMR4CLR:
    case TMR4SET:
    case TMR4INV:
        *namep = "TMR4";
        VALUE(TMR4) = write_op(VALUE(TMR4), data, offset);
        pic32mx3_tmr_recompute(s, 3);
        return;
    case PR4:
    case PR4CLR:
    case PR4SET:
    case PR4INV:
        *namep = "PR4";
        VALUE(PR4) = write_op(VALUE(PR4), data, offset);
        pic32mx3_tmr_recompute(s, 3);
        return;
    case T5CON:
    case T5CONCLR:
    case T5CONSET:
    case T5CONINV:
        *namep = "T5CON";
        VALUE(T5CON) = write_op(VALUE(T5CON), data, offset);
        pic32mx3_tmr_recompute(s, 4);
        return;
    case TMR5:
    case TMR5CLR:
    case TMR5SET:
    case TMR5INV:
        *namep = "TMR5";
        VALUE(TMR5) = write_op(VALUE(TMR5), data, offset);
        pic32mx3_tmr_recompute(s, 4);
        return;
    case PR5:
    case PR5CLR:
    case PR5SET:
    case PR5INV:
        *namep = "PR5";
        VALUE(PR5) = write_op(VALUE(PR5), data, offset);
        pic32mx3_tmr_recompute(s, 4);
        /* If T4CON.T32, PR5 is the high half of the 32-bit period. */
        if (VALUE(T4CON) & (1u << 3))
            pic32mx3_tmr_recompute(s, 3);
        return;

    /*-------------------------------------------------------------------------
     * Input Capture IC1-IC5.
     */
    case IC1CON:
    case IC1CONCLR:
    case IC1CONSET:
    case IC1CONINV:
        *namep = "IC1CON";
        VALUE(IC1CON) = write_op(VALUE(IC1CON), data, offset);
        pic32mx3_ic1con_after_write(s, VALUE(IC1CON));
        return;
    WRITEOP(IC2CON); return;
    WRITEOP(IC3CON); return;
    WRITEOP(IC4CON); return;
    WRITEOP(IC5CON); return;

    /*-------------------------------------------------------------------------
     * Output Compare OC1-OC5.
     */
    WRITEOP(OC1CON); return;
    WRITEOP(OC1R); return;
    WRITEOP(OC1RS); return;
    WRITEOP(OC2CON); return;
    WRITEOP(OC2R); return;
    WRITEOP(OC2RS); return;
    WRITEOP(OC3CON); return;
    WRITEOP(OC3R); return;
    WRITEOP(OC3RS); return;
    WRITEOP(OC4CON); return;
    WRITEOP(OC4R); return;
    WRITEOP(OC4RS); return;
    WRITEOP(OC5CON); return;
    WRITEOP(OC5R); return;
    WRITEOP(OC5RS); return;

    /*-------------------------------------------------------------------------
     * I2C 1.
     */
    WRITEOP(I2C1CON); return;
    WRITEOPR(I2C1STAT, PIC32_I2CSTAT_TBF); return;  // TBF is read-only
    WRITEOP(I2C1ADD); return;
    WRITEOP(I2C1MSK); return;
    WRITEOP(I2C1BRG); return;
    case I2C1TRN:
    case I2C1TRNCLR:
    case I2C1TRNSET:
    case I2C1TRNINV:
        *namep = "I2C1TRN";
        VALUE(I2C1TRN) = write_op(VALUE(I2C1TRN), data, offset);
        pic32mx3_i2c_trn_write(s, 0, VALUE(I2C1TRN));
        return;
    READONLY(I2C1RCV);          // Receive: read-only

    /*-------------------------------------------------------------------------
     * I2C 2.
     */
    WRITEOP(I2C2CON); return;
    WRITEOPR(I2C2STAT, PIC32_I2CSTAT_TBF); return;  // TBF is read-only
    WRITEOP(I2C2ADD); return;
    WRITEOP(I2C2MSK); return;
    WRITEOP(I2C2BRG); return;
    case I2C2TRN:
    case I2C2TRNCLR:
    case I2C2TRNSET:
    case I2C2TRNINV:
        *namep = "I2C2TRN";
        VALUE(I2C2TRN) = write_op(VALUE(I2C2TRN), data, offset);
        pic32mx3_i2c_trn_write(s, 1, VALUE(I2C2TRN));
        return;
    READONLY(I2C2RCV);          // Receive: read-only

    /*-------------------------------------------------------------------------
     * Parallel Master Port.
     */
    WRITEOP(PMCON); return;
    WRITEOP(PMMODE); return;
    WRITEOP(PMADDR); return;
    case PMDOUT: *namep = "PMDOUT"; goto op_PMDOUT;
    case PMDOUTCLR: *namep = "PMDOUTCLR"; goto op_PMDOUT;
    case PMDOUTSET: *namep = "PMDOUTSET"; goto op_PMDOUT;
    case PMDOUTINV: *namep = "PMDOUTINV"; op_PMDOUT: {
            unsigned prev = VALUE(PMDOUT);

            VALUE(PMDOUT) = write_op(prev, data, offset);
            pic32mx3_pmp_after_pmdout_write(s);
        }
        return;
    STORAGE(PMDIN); break;
    WRITEOP(PMAEN); return;
    READONLY(PMSTAT);

    default:
        printf("--- Write %08x to 1f8%05x: peripheral register not supported\n",
            data, offset);
        if (TRACE)
            fprintf(qemu_logfile, "--- Write %08x to 1f8%05x: peripheral register not supported\n",
                data, offset);
        exit(1);
readonly:
        printf("--- Write %08x to %s: readonly register\n",
            data, *namep);
        if (TRACE)
            fprintf(qemu_logfile, "--- Write %08x to %s: readonly register\n",
                data, *namep);
        *namep = 0;
        return;
    }
    *bufp = data;
}

static uint64_t pic32_io_read(void *opaque, hwaddr addr, unsigned bytes)
{
    pic32_t *s = opaque;
    uint32_t offset = addr & 0xfffff;
    const char *name = "???";
    uint32_t data = 0;

    data = io_read32(s, offset & ~3, &name);
    switch (bytes) {
    case 1:
        if ((offset &= 3) != 0) {
            // Unaligned read.
            data >>= offset * 8;
        }
        data = (uint8_t) data;
        if (TRACE) {
            fprintf(qemu_logfile, "--- I/O Read  %02x from %s\n", data, name);
        }
        break;
    case 2:
        if (offset & 2) {
            // Unaligned read.
            data >>= 16;
        }
        data = (uint16_t) data;
        if (TRACE) {
            fprintf(qemu_logfile, "--- I/O Read  %04x from %s\n", data, name);
        }
        break;
    default:
        if (TRACE) {
            fprintf(qemu_logfile, "--- I/O Read  %08x from %s\n", data, name);
        }
        break;
    }
    return data;
}

static void pic32_io_write(void *opaque, hwaddr addr, uint64_t data, unsigned bytes)
{
    pic32_t *s = opaque;
    uint32_t offset = addr & 0xfffff;
    const char *name = "???";

    // Fetch data and align to word format.
    switch (bytes) {
    case 1:
        data = (uint8_t) data;
        data <<= (offset & 3) * 8;
        break;
    case 2:
        data = (uint16_t) data;
        data <<= (offset & 2) * 8;
        break;
    }
    io_write32(s, offset & ~3, data, &name);

    if (TRACE && name != 0) {
        fprintf(qemu_logfile, "--- I/O Write %08x to %s\n",
            (uint32_t) data, name);
    }
}

static const MemoryRegionOps pic32_io_ops = {
    .read       = pic32_io_read,
    .write      = pic32_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void main_cpu_reset(void *opaque)
{
    MIPSCPU *cpu = opaque;
    CPUMIPSState *env = &cpu->env;
    int i;

    cpu_reset(CPU(cpu));

    /* Adjust the initial configuration for M4K core. */
    env->CP0_IntCtl = 0;
    env->CP0_Debug = (1 << CP0DB_CNT) | (3 << CP0DB_VER);
    for (i=0; i<7; i++)
        env->CP0_WatchHi[i] = 0;
}

static void store_byte(unsigned address, unsigned char byte)
{
    if (address >= PROGRAM_FLASH_START &&
        address < PROGRAM_FLASH_START + PROGRAM_FLASH_SIZE)
    {
        //printf("Store %02x to program memory %08x\n", byte, address);
        prog_ptr[address & 0xfffff] = byte;
    }
    else if (address >= BOOT_FLASH_START &&
             address < BOOT_FLASH_START + BOOT_FLASH_SIZE)
    {
        //printf("Store %02x to boot memory %08x\n", byte, address);
        boot_ptr[address & 0xffff] = byte;
    }
    else {
        printf("Bad hex file: incorrect address %08X, must be %08X-%08X or %08X-%08X\n",
            address, PROGRAM_FLASH_START,
            PROGRAM_FLASH_START + PROGRAM_FLASH_SIZE - 1,
            BOOT_FLASH_START, BOOT_FLASH_START + BOOT_FLASH_SIZE - 1);
        exit(1);
    }
}

/*
 * Ignore ^C and ^\ signals and pass these characters to the target.
 */
static void pic32_pass_signal_chars(void)
{
    struct termios tty;

    tcgetattr(0, &tty);
    tty.c_lflag &= ~ISIG;
    tcsetattr(0, TCSANOW, &tty);
}

static void pic32_init(MachineState *machine, int board_type)
{
    const char *cpu_model = machine->cpu_model;
    unsigned ram_size = DATA_MEM_SIZE;
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *ram_main = g_new(MemoryRegion, 1);
    MemoryRegion *prog_mem = g_new(MemoryRegion, 1);
    MemoryRegion *boot_mem = g_new(MemoryRegion, 1);
    MemoryRegion *io_mem = g_new(MemoryRegion, 1);
    MIPSCPU *cpu;
    CPUMIPSState *env;

    DeviceState *dev = qdev_create(NULL, TYPE_MIPS_PIC32);
    pic32_t *s = OBJECT_CHECK(pic32_t, dev, TYPE_MIPS_PIC32);
    s->board_type = board_type;
    /*
     * If set, reading RSWRST with SWRST bit set calls exit(0).  That breaks
     * real firmware (e.g. MPLAB Harmony) which unlocks SYSKEY, writes
     * RSWRST, then polls RSWRST until the bit clears — they must keep running
     * after qemu_system_reset_request() + io_reset (handled on RSWRST write).
     */
    s->stop_on_reset = 0;
    s->iomem = g_malloc0(IO_MEM_SIZE);  /* backing storage for I/O area */

    qdev_init_nofail(dev);

    /* Init CPU. */
    if (! cpu_model) {
        cpu_model = "M4K";
    }
    printf("Board: %s\n", board_name[board_type]);
    if (qemu_logfile)
        fprintf(qemu_logfile, "Board: %s\n", board_name[board_type]);

    printf("Processor: %s\n", cpu_model);
    if (qemu_logfile)
        fprintf(qemu_logfile, "Processor: %s\n", cpu_model);

    cpu = cpu_mips_init(cpu_model);
    if (! cpu) {
        fprintf(stderr, "Unable to find CPU definition\n");
        exit(1);
    }
    s->cpu = cpu;
    env = &cpu->env;

    /* Register RAM */
    printf("RAM size: %u kbytes\n", ram_size / 1024);
    if (qemu_logfile)
        fprintf(qemu_logfile, "RAM size: %u kbytes\n", ram_size / 1024);

    memory_region_init_ram(ram_main, NULL, "kernel.ram",
        ram_size, &error_abort);
    vmstate_register_ram_global(ram_main);
    memory_region_add_subregion(system_memory, DATA_MEM_START, ram_main);

    /* Alias for user space.
     * For MX family only. */
    MemoryRegion *ram_user = g_new(MemoryRegion, 1);
    memory_region_init_alias(ram_user, NULL, "user.ram",
        ram_main, 0x8000, ram_size - 0x8000);
    memory_region_add_subregion(system_memory, USER_MEM_START + 0x8000, ram_user);

    /* Special function registers. */
    memory_region_init_io(io_mem, NULL, &pic32_io_ops, s,
                          "io", IO_MEM_SIZE);
    memory_region_add_subregion(system_memory, IO_MEM_START, io_mem);

    /*
     * Map the flash memory.
     */
    memory_region_init_ram(boot_mem, NULL, "boot.flash", BOOT_FLASH_SIZE, &error_abort);
    memory_region_init_ram(prog_mem, NULL, "prog.flash", PROGRAM_FLASH_SIZE, &error_abort);

    /* Load a Flash memory image. */
    if (! machine->kernel_filename) {
        error_report("No -kernel argument was specified.");
        exit(1);
    }
    prog_ptr = memory_region_get_ram_ptr(prog_mem);
    boot_ptr = memory_region_get_ram_ptr(boot_mem);
    pic32mx3_nvm_bind_flash(s, (uint8_t *)prog_ptr, (uint8_t *)boot_ptr,
                            PROGRAM_FLASH_SIZE, BOOT_FLASH_SIZE,
                            PROGRAM_FLASH_START, BOOT_FLASH_START);
    /*
     * Optional persisted images (-global mips-pic32mx3.prog-flash=…): erased
     * (0xFF) then file contents; -kernel / ELF overwrites firmware regions.
     */
    pic32mx3_flash_load_images(s);

    /* Map flash into address space before ELF loading (rom infrastructure
     * writes via cpu_physical_memory_write_rom which requires mapped regions). */
    memory_region_set_readonly(boot_mem, true);
    memory_region_set_readonly(prog_mem, true);
    memory_region_add_subregion(system_memory, BOOT_FLASH_START, boot_mem);
    memory_region_add_subregion(system_memory, PROGRAM_FLASH_START, prog_mem);

    /* Detect ELF magic to choose the loading path. */
    {
        FILE *fp = fopen(machine->kernel_filename, "rb");
        unsigned char magic[4] = {0};
        if (fp) {
            fread(magic, 1, 4, fp);
            fclose(fp);
        }
        if (magic[0] == 0x7f && magic[1] == 'E' &&
            magic[2] == 'L'  && magic[3] == 'F')
        {
            /*
             * ELF firmware: load PT_LOAD segments directly into flash RAM
             * pointers via store_byte.  The generic QEMU rom infrastructure
             * (load_elf → rom_add_elf_program → rom_load_all) rejects
             * overlapping physical regions, but PIC32 ELF images routinely
             * have kseg0 (0x9fc0xxxx) and kseg1 (0xbfc0xxxx) segments that
             * alias the same boot flash, so we bypass it.
             */
            int fd = open(machine->kernel_filename, O_RDONLY);
            if (fd < 0) {
                error_report("Failed to open ELF '%s': %s",
                             machine->kernel_filename, strerror(errno));
                exit(1);
            }
            Elf32_Ehdr ehdr;
            if (read(fd, &ehdr, sizeof(ehdr)) != sizeof(ehdr) ||
                memcmp(ehdr.e_ident, ELFMAG, SELFMAG) != 0 ||
                le16toh(ehdr.e_type) != ET_EXEC ||
                le16toh(ehdr.e_machine) != EM_MIPS) {
                error_report("'%s' is not a valid MIPS ELF executable",
                             machine->kernel_filename);
                close(fd);
                exit(1);
            }
            uint16_t phnum = le16toh(ehdr.e_phnum);
            uint32_t phoff = le32toh(ehdr.e_phoff);
            uint64_t elf_entry = (uint64_t)le32toh(ehdr.e_entry);
            int total_loaded = 0;

            /*
             * Read all PT_LOAD phdrs into an array, then sort by filesz
             * descending so that large "background" segments (e.g. the
             * kseg1 boot flash region 0xbfc00480–0xbfc03000 that includes
             * zero padding over the vector area) are written first, and
             * smaller, specific segments (e.g. the kseg0 vectors at
             * 0x9fc01180) overwrite the correct bytes afterward.
             */
            typedef struct {
                uint32_t paddr;
                uint32_t filesz;
                uint32_t offset;
                int      region; /* 0=flash, 1=ram */
            } SegInfo;

            SegInfo *segs = g_new0(SegInfo, phnum);
            int nseg = 0;
            uint16_t pi;
            for (pi = 0; pi < phnum; pi++) {
                Elf32_Phdr phdr;
                if (lseek(fd, phoff + pi * sizeof(phdr), SEEK_SET) < 0 ||
                    read(fd, &phdr, sizeof(phdr)) != sizeof(phdr)) {
                    break;
                }
                if (le32toh(phdr.p_type) != PT_LOAD) {
                    continue;
                }
                uint32_t filesz = le32toh(phdr.p_filesz);
                uint32_t vaddr  = le32toh(phdr.p_vaddr);
                uint32_t paddr  = vaddr & 0x1fffffffu;
                if (filesz == 0) {
                    continue;
                }
                int in_prog = (paddr >= PROGRAM_FLASH_START &&
                               paddr < PROGRAM_FLASH_START + PROGRAM_FLASH_SIZE);
                int in_boot = (paddr >= BOOT_FLASH_START &&
                               paddr < BOOT_FLASH_START + BOOT_FLASH_SIZE);
                int in_ram  = (paddr < DATA_MEM_START + DATA_MEM_SIZE);
                if (!in_prog && !in_boot && !in_ram) {
                    continue;
                }
                segs[nseg].paddr  = paddr;
                segs[nseg].filesz = filesz;
                segs[nseg].offset = le32toh(phdr.p_offset);
                segs[nseg].region = in_ram ? 1 : 0;
                nseg++;
            }

            /* Sort: largest filesz first so contained segments overwrite. */
            int si, sj;
            for (si = 0; si < nseg - 1; si++) {
                for (sj = si + 1; sj < nseg; sj++) {
                    if (segs[sj].filesz > segs[si].filesz) {
                        SegInfo tmp = segs[si];
                        segs[si] = segs[sj];
                        segs[sj] = tmp;
                    }
                }
            }

            for (si = 0; si < nseg; si++) {
                uint8_t *buf = g_malloc(segs[si].filesz);
                if (lseek(fd, segs[si].offset, SEEK_SET) < 0 ||
                    read(fd, buf, segs[si].filesz) !=
                        (ssize_t)segs[si].filesz) {
                    error_report("Failed to read ELF segment at offset %#x",
                                 segs[si].offset);
                    g_free(buf);
                    g_free(segs);
                    close(fd);
                    exit(1);
                }
                if (segs[si].region) {
                    cpu_physical_memory_write(segs[si].paddr, buf,
                                              segs[si].filesz);
                } else {
                    uint32_t i;
                    for (i = 0; i < segs[si].filesz; i++) {
                        store_byte(segs[si].paddr + i, buf[i]);
                    }
                }
                g_free(buf);
                total_loaded += segs[si].filesz;
            }
            g_free(segs);
            close(fd);
            printf("Firmware: ELF, %d bytes, entry 0x%08llx\n",
                   total_loaded, (unsigned long long)elf_entry);
            if (bios_name) {
                pic32_load_hex_file(bios_name, store_byte);
            }
        } else {
            /* Intel HEX or Motorola SREC firmware image. */
            if (bios_name)
                pic32_load_hex_file(bios_name, store_byte);
            pic32_load_hex_file(machine->kernel_filename, store_byte);
        }
    }

    /* Init internal devices */
    s->irq_raise = irq_raise;
    s->irq_clear = irq_clear;
    qemu_register_reset(main_cpu_reset, cpu);

    /* Setup interrupt controller in EIC mode. */
    env->CP0_Config3 |= 1 << CP0C3_VEIC;
    cpu_mips_irq_init_cpu(env);
    env->eic_timer_irq = pic32_timer_irq;
    env->eic_soft_irq = pic32_soft_irq;
    env->eic_context = s;

    /* CPU runs at 80MHz.
     * Count register increases at half this rate. */
    cpu_mips_clock_init(env, 24*1000*1000);  /* Count = SYS_CLK/2 = 48/2 = 24 MHz */

    /*
     * Initialize board-specific parameters.
     */
    int cs0_port, cs0_pin, cs1_port, cs1_pin;
    switch (board_type) {
    default:
    case BOARD_MX350_GENERIC:
        BOOTMEM(DEVCFG0) = 0xffffff7f;
        BOOTMEM(DEVCFG1) = 0x7f743cb9;     // FRC+PLL, PBDIV=1, WDT off
        BOOTMEM(DEVCFG2) = 0xfff9b11a;     // FPLLIDIV=2, FPLLMUL=20, FPLLODIV=1 -> 80MHz
        BOOTMEM(DEVCFG3) = 0xffff0000;
        VALUE(DEVID)     = 0x0516D053;      // PIC32MX350F256H rev A3
        VALUE(OSCCON)    = 0x00001120;      // FRC+PLL, PBDIV=1, 80MHz
        s->sdcard_spi_port = 1;             // SD card at SPI2,
        cs0_port = 0;  cs0_pin = 1;         // select0 at B1,
        cs1_port = -1; cs1_pin = -1;        // select1 not available
        break;
    }

    /*
     * UARTs: U1/U2/U3.
     * Host mapping for pic32mx3-generic:
     *   -serial #0 -> UART3 (115200; guest PPS for TX/RX pins is not applied to the char backend.)
     *   -serial #1 -> UART1 (e.g. debug on RE5/RF2)
     *   -serial #2 -> UART2 (optional)
     * Use "-serial null -serial stdio" if the image uses UART1 as console.
     * Debug: QEMU_PIC32_TRACE_UART=1 logs each UxTXREG write (unit index).
     * Flash persistence (NVS across runs):
     *   -global mips-pic32mx3.prog-flash=/path/to/prog.bin
     *   -global mips-pic32mx3.boot-flash=/path/to/boot.bin   (optional; board
     *     DEVCFG words are still written after load — see pic32mx3_nvm.c)
     */
    /* Base IRQ = IFS1 UxEIF bit index (see irq_to_vector): U1 38, U2 56, U3 62 (+2 = U3TX in IFS2). */
    s->pbclk_hz = PIC32MX3_PBCLK_HZ;
    pic32_uart_init(s, 0, 38, U1STA, U1MODE);
    pic32_uart_init(s, 1, 56, U2STA, U2MODE);
    pic32_uart_init(s, 2, 62, U3STA, U3MODE);
    if (serial_hds[0]) {
        s->uart[0].chr = NULL;
        pic32_uart_attach_chr(s, 2, serial_hds[0]);
    }
    if (serial_hds[1]) {
        s->uart[1].chr = NULL;
        pic32_uart_attach_chr(s, 0, serial_hds[1]);
    }
    if (serial_hds[2]) {
        pic32_uart_attach_chr(s, 1, serial_hds[2]);
    }

    /* SPIs: MX350F256H has SPI1, SPI2, SPI3 */
    pic32_spi_init(s, 0, PIC32_IRQ_SPI1E, SPI1CON, SPI1STAT);
    pic32_spi_init(s, 1, PIC32_IRQ_SPI2E, SPI2CON, SPI2STAT);
    pic32_spi_init(s, 2, PIC32_IRQ_SPI3E, SPI3CON, SPI3STAT);

    /*
     * Load SD card images.
     * Use options:
     *      -sd filename
     * or   -hda filename
     * and  -hdb filename
     */
    const char *sd0_file = 0, *sd1_file = 0;
    DriveInfo *dinfo = drive_get(IF_IDE, 0, 0);
    if (dinfo) {
        sd0_file = qemu_opt_get(dinfo->opts, "file");
        dinfo->is_default = 1;

        dinfo = drive_get(IF_IDE, 0, 1);
        if (dinfo) {
            sd1_file = qemu_opt_get(dinfo->opts, "file");
            dinfo->is_default = 1;
        }
    }
    if (! sd0_file) {
        dinfo = drive_get(IF_SD, 0, 0);
        if (dinfo) {
            sd0_file = qemu_opt_get(dinfo->opts, "file");
            dinfo->is_default = 1;
        }
    }
    pic32_sdcard_init(s, 0, "sd0", sd0_file, cs0_port, cs0_pin);
    pic32_sdcard_init(s, 1, "sd1", sd1_file, cs1_port, cs1_pin);

    io_reset(s);

    {
        int ti;

        for (ti = 0; ti < PIC32_MX3_N_TMRS; ti++) {
            s->tmr_opaque[ti].mcu = s;
            s->tmr_opaque[ti].idx = ti;
            if (!s->tmr_timer[ti]) {
                s->tmr_timer[ti] = timer_new_ns(pic32mx3_tmr3_clock_type(),
                                                pic32mx3_tmr_cb,
                                                &s->tmr_opaque[ti]);
            }
        }
    }

    pic32_sdcard_reset(s);
    pic32mx3_flash_register_exit_save(s);

    /*
     * Optional plotter-bridge (host SHM + MMIO) for contrib/qemu-grbl-plugin visualizer.
     *   PLOTTER_BRIDGE_SHM=/path/to/file     — create with: truncate -s 32 file
     *   PLOTTER_BRIDGE_CONFIG=/path/to.json  — optional (gpio limits, PWM defaults)
     * Guest physical: SHM 0x1f400000, GPIO 0x1f400040 (KSEG0 0xbf400000 / +0x40).
     * Omit env vars when not using the bridge (default).
     */
    if (!qtest_enabled()) {
        const char *pbs = getenv("PLOTTER_BRIDGE_SHM");

        if (pbs && pbs[0]) {
            DeviceState *pb = qdev_try_create(NULL, "plotter-bridge");

            if (pb) {
                SysBusDevice *pbd = SYS_BUS_DEVICE(pb);
                const char *pjson = getenv("PLOTTER_BRIDGE_CONFIG");

                object_property_set_str(OBJECT(pb), pbs, "shm-path", &error_abort);
                if (pjson && pjson[0]) {
                    object_property_set_str(OBJECT(pb), pjson, "plotter-config-file",
                                            &error_abort);
                }
                qdev_init_nofail(pb);
                sysbus_mmio_map(pbd, 0, 0x1f400000);
                sysbus_mmio_map(pbd, 1, 0x1f400040);
            }
        }
    }

    pic32_pass_signal_chars();
}

static void pic32_init_generic(MachineState *machine)
{
    pic32_init(machine, BOARD_MX350_GENERIC);
}

static int pic32_sysbus_device_init(SysBusDevice *sysbusdev)
{
    return 0;
}

static Property pic32_properties[] = {
    DEFINE_PROP_STRING("prog-flash", pic32_t, prog_flash_path),
    DEFINE_PROP_STRING("boot-flash", pic32_t, boot_flash_path),
    DEFINE_PROP_END_OF_LIST(),
};

static void pic32_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    k->init = pic32_sysbus_device_init;
    dc->props = pic32_properties;
}

static const TypeInfo pic32_device = {
    .name          = TYPE_MIPS_PIC32,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(pic32_t),
    .class_init    = pic32_class_init,
};

static void pic32_register_types(void)
{
    type_register_static(&pic32_device);
}

static QEMUMachine pic32_board[1] = {
    {
        .name       = "pic32mx3-generic",
        .desc       = "PIC32MX350F256H microcontroller (generic board)",
        .init       = pic32_init_generic,
        .max_cpus   = 1,
    },
};

static void pic32_machine_init(void)
{
    qemu_register_machine(&pic32_board[0]);
}

type_init(pic32_register_types)
machine_init(pic32_machine_init);

#endif /* !TARGET_MIPS64 && !TARGET_WORDS_BIGENDIAN */
