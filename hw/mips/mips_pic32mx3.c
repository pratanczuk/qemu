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
#include "sysemu/char.h"
#include "hw/loader.h"
#include "qemu/error-report.h"
#include "hw/empty_slot.h"
#include "elf.h"
#include <termios.h>

#define PIC32MX3
#include "pic32mx.h"
#include "pic32_peripherals.h"

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
 */
#define TRACE   0

/*
 * PIC32MX3 specific table:
 * translate IRQ number to interrupt vector.
 */
static const int irq_to_vector[] = {
    PIC32_VECT_CT,      /* 0  - Core Timer Interrupt */
    PIC32_VECT_CS0,     /* 1  - Core Software Interrupt 0 */
    PIC32_VECT_CS1,     /* 2  - Core Software Interrupt 1 */
    PIC32_VECT_INT0,    /* 3  - External Interrupt 0 */
    PIC32_VECT_T1,      /* 4  - Timer1 */
    PIC32_VECT_IC1,     /* 5  - Input Capture 1 */
    PIC32_VECT_OC1,     /* 6  - Output Compare 1 */
    PIC32_VECT_INT1,    /* 7  - External Interrupt 1 */
    PIC32_VECT_T2,      /* 8  - Timer2 */
    PIC32_VECT_IC2,     /* 9  - Input Capture 2 */
    PIC32_VECT_OC2,     /* 10 - Output Compare 2 */
    PIC32_VECT_INT2,    /* 11 - External Interrupt 2 */
    PIC32_VECT_T3,      /* 12 - Timer3 */
    PIC32_VECT_IC3,     /* 13 - Input Capture 3 */
    PIC32_VECT_OC3,     /* 14 - Output Compare 3 */
    PIC32_VECT_INT3,    /* 15 - External Interrupt 3 */
    PIC32_VECT_T4,      /* 16 - Timer4 */
    PIC32_VECT_IC4,     /* 17 - Input Capture 4 */
    PIC32_VECT_OC4,     /* 18 - Output Compare 4 */
    PIC32_VECT_INT4,    /* 19 - External Interrupt 4 */
    PIC32_VECT_T5,      /* 20 - Timer5 */
    PIC32_VECT_IC5,     /* 21 - Input Capture 5 */
    PIC32_VECT_OC5,     /* 22 - Output Compare 5 */
    PIC32_VECT_SPI1,    /* 23 - SPI1 Fault */
    PIC32_VECT_SPI1,    /* 24 - SPI1 Transfer Done */
    PIC32_VECT_SPI1,    /* 25 - SPI1 Receive Done */

    PIC32_VECT_U1,      /* 26 - UART1 Error (shared SPI3/I2C3 slot; MX3 uses U1) */
    PIC32_VECT_U1,      /* 27 - UART1 Receiver */
    PIC32_VECT_U1,      /* 28 - UART1 Transmitter */

    PIC32_VECT_I2C1,    /* 29 - I2C1 Bus Collision Event */
    PIC32_VECT_I2C1,    /* 30 - I2C1 Slave Event */
    PIC32_VECT_I2C1,    /* 31 - I2C1 Master Event */
    PIC32_VECT_CN,      /* 32 - Input Change Interrupt */
    PIC32_VECT_AD1,     /* 33 - ADC1 Convert Done */
    PIC32_VECT_PMP,     /* 34 - Parallel Master Port */
    PIC32_VECT_CMP1,    /* 35 - Comparator Interrupt */
    PIC32_VECT_CMP2,    /* 36 - Comparator Interrupt */

    PIC32_VECT_SPI2,    /* 37 - SPI2 Fault (shared U3/I2C4 slot; MX3 uses SPI2) */
    PIC32_VECT_SPI2,    /* 38 - SPI2 Transfer Done */
    PIC32_VECT_SPI2,    /* 39 - SPI2 Receive Done */

    PIC32_VECT_U2,      /* 40 - UART2 Error (shared SPI4/I2C5 slot; MX3 uses U2) */
    PIC32_VECT_U2,      /* 41 - UART2 Receiver */
    PIC32_VECT_U2,      /* 42 - UART2 Transmitter */

    PIC32_VECT_I2C2,    /* 43 - I2C2 Bus Collision Event */
    PIC32_VECT_I2C2,    /* 44 - I2C2 Slave Event */
    PIC32_VECT_I2C2,    /* 45 - I2C2 Master Event */
    PIC32_VECT_FSCM,    /* 46 - Fail-Safe Clock Monitor */
    PIC32_VECT_RTCC,    /* 47 - Real-Time Clock and Calendar */
    PIC32_VECT_DMA0,    /* 48 - DMA Channel 0 */
    PIC32_VECT_DMA1,    /* 49 - DMA Channel 1 */
    PIC32_VECT_DMA2,    /* 50 - DMA Channel 2 */
    PIC32_VECT_DMA3,    /* 51 - DMA Channel 3 */
    -1,                 /* 52 - DMA4 - not present on MX3xx */
    -1,                 /* 53 - DMA5 - not present on MX3xx */
    -1,                 /* 54 - DMA6 - not present on MX3xx */
    -1,                 /* 55 - DMA7 - not present on MX3xx */
    PIC32_VECT_FCE,     /* 56 - Flash Control Event */
    PIC32_VECT_USB,     /* 57 - USB */
};

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

                int level = VALUE(IPC(v >> 2));
                level >>= 2 + (v & 3) * 8;
                level &= 7;
                if (level > cause_ripl) {
                    vector = v;
                    cause_ripl = level;
                }
            }
        }
        VALUE(INTSTAT) = vector | (cause_ripl << 8);
    }

    if (cause_ripl == current_ripl)
        return;

    if (TRACE)
        fprintf(qemu_logfile, "--- Priority level Cause.RIPL = %u\n",
            cause_ripl);

    /*
     * Modify Cause.RIPL field and take EIC interrupt.
     */
    env->CP0_Cause &= ~(0x3f << (CP0Ca_IP + 2));
    env->CP0_Cause |= cause_ripl << (CP0Ca_IP + 2);
    cpu_interrupt(CPU(s->cpu), CPU_INTERRUPT_HARD);
}

/*
 * Set interrupt flag status
 */
static void irq_raise(pic32_t *s, int irq)
{
    if (VALUE(IFS(irq >> 5)) & (1 << (irq & 31)))
        return;

    VALUE(IFS(irq >> 5)) |= 1 << (irq & 31);
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

    /*
     * Reset UARTs. MX350F256H has U1 and U2 only.
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

    /*
     * Reset Input Capture IC1-IC5.
     */
    VALUE(IC1CON) = 0;
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
}

static unsigned io_read32(pic32_t *s, unsigned offset, const char **namep)
{
    unsigned *bufp = &VALUE(offset);

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

static void io_write32(pic32_t *s, unsigned offset, unsigned data, const char **namep)
{
    unsigned *bufp = &VALUE(offset);

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
    WRITEOP(INTCON); return;    // Interrupt Control
    READONLY(INTSTAT);          // Interrupt Status
    WRITEOP(IPTMR);  return;    // Temporal Proximity Timer
    WRITEOP(IFS0); goto irq;    // IFS(0..2) - Interrupt Flag Status
    WRITEOP(IFS1); goto irq;
    WRITEOP(IFS2); goto irq;
    WRITEOP(IEC0); goto irq;    // IEC(0..2) - Interrupt Enable Control
    WRITEOP(IEC1); goto irq;
    WRITEOP(IEC2); goto irq;
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
        /* Unlock state machine. */
        if (s->syskey_unlock == 0 && VALUE(SYSKEY) == 0xaa996655)
            s->syskey_unlock = 1;
        if (s->syskey_unlock == 1 && VALUE(SYSKEY) == 0x556699aa)
            s->syskey_unlock = 2;
        else
            s->syskey_unlock = 0;
        break;
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
    WRITEOP(AD1CON1); return;   // Control register 1
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
     * Timers T1-T5.
     */
    WRITEOP(T1CON); return;
    WRITEOP(TMR1); return;
    WRITEOP(PR1); return;
    WRITEOP(T2CON); return;
    WRITEOP(TMR2); return;
    WRITEOP(PR2); return;
    WRITEOP(T3CON); return;
    WRITEOP(TMR3); return;
    WRITEOP(PR3); return;
    WRITEOP(T4CON); return;
    WRITEOP(TMR4); return;
    WRITEOP(PR4); return;
    WRITEOP(T5CON); return;
    WRITEOP(TMR5); return;
    WRITEOP(PR5); return;

    /*-------------------------------------------------------------------------
     * Input Capture IC1-IC5.
     */
    WRITEOP(IC1CON); return;
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
    STORAGE(I2C1TRN); break;    // Transmit: write-only data register
    READONLY(I2C1RCV);          // Receive: read-only

    /*-------------------------------------------------------------------------
     * I2C 2.
     */
    WRITEOP(I2C2CON); return;
    WRITEOPR(I2C2STAT, PIC32_I2CSTAT_TBF); return;  // TBF is read-only
    WRITEOP(I2C2ADD); return;
    WRITEOP(I2C2MSK); return;
    WRITEOP(I2C2BRG); return;
    STORAGE(I2C2TRN); break;    // Transmit: write-only data register
    READONLY(I2C2RCV);          // Receive: read-only

    /*-------------------------------------------------------------------------
     * Parallel Master Port.
     */
    WRITEOP(PMCON); return;
    WRITEOP(PMMODE); return;
    WRITEOP(PMADDR); return;
    WRITEOP(PMDOUT); return;
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
    s->stop_on_reset = 1;               /* halt simulation on soft reset */
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
            /* ELF firmware image: load via QEMU rom infrastructure. */
            uint64_t elf_entry;
            const int big_endian = 0;   /* MIPS32 little-endian */
            const int clear_lsb = 0;    /* Not ARM Thumb mode */
            int elf_size = load_elf(machine->kernel_filename,
                                    cpu_mips_kseg0_to_phys, NULL,
                                    &elf_entry, NULL, NULL,
                                    big_endian,
                                    EM_MIPS,
                                    clear_lsb);
            if (elf_size < 0) {
                error_report("Failed to load ELF firmware '%s' (error %d)",
                             machine->kernel_filename, elf_size);
                exit(1);
            }
            printf("Firmware: ELF, %d bytes, entry 0x%08llx\n",
                   elf_size, (unsigned long long) elf_entry);
            if (bios_name) {
                /* Optional boot ROM as hex/srec alongside ELF kernel. */
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
    cpu_mips_clock_init(env, 40*1000*1000);

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

    /* UARTs: MX350F256H has U1 and U2 only */
    pic32_uart_init(s, 0, PIC32_IRQ_U1E, U1STA, U1MODE);
    pic32_uart_init(s, 1, PIC32_IRQ_U2E, U2STA, U2MODE);

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
    pic32_sdcard_reset(s);
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

static void pic32_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = pic32_sysbus_device_init;
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
