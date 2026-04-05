/*
 * UART ports.
 *
 * Copyright (C) 2014-2015 Serge Vakulenko
 *
 * Permission to use, copy, modify, and distribute this software
 * and its documentation for any purpose and without fee is hereby
 * granted, provided that the above copyright notice appear in all
 * copies and that both that the copyright notice and this
 * permission notice and warranty disclaimer appear in supporting
 * documentation, and that the name of the author not be used in
 * advertising or publicity pertaining to distribution of the
 * software without specific, written prior permission.
 *
 * The author disclaim all warranties with regard to this
 * software, including all implied warranties of merchantability
 * and fitness.  In no event shall the author be liable for any
 * special, indirect or consequential damages or any damages
 * whatsoever resulting from loss of use, data or profits, whether
 * in an action of contract, negligence or other tortious action,
 * arising out of or in connection with the use or performance of
 * this software.
 */
#include "hw/hw.h"
#include "hw/char/serial.h"
#include "sysemu/char.h"
#include "pic32_peripherals.h"
#include <stdlib.h>

#include "pic32mz.h"

#define UART_IRQ_ERR    0               // error irq offset
#define UART_IRQ_RX     1               // receiver irq offset
#define UART_IRQ_TX     2               // transmitter irq offset

/*
 * Read of UxRXREG register.
 */
unsigned pic32_uart_get_char(pic32_t *s, int unit)
{
    uart_t *u = &s->uart[unit];
    unsigned value;

    if (u->rx_len == 0) {
        return 0;
    }
    /* Pop one byte from RX FIFO. */
    value = u->rx_fifo[u->rx_rd];
    u->rx_rd = (u->rx_rd + 1) % PIC32_UART_RX_FIFO;
    u->rx_len--;

    if (u->rx_len == 0) {
        VALUE(u->sta) &= ~PIC32_USTA_URXDA;
        s->irq_clear(s, u->irq + UART_IRQ_RX);
    } else {
        VALUE(u->sta) |= PIC32_USTA_URXDA;
    }
    return value;
}

/*
 * Write to UxTXREG register.
 */
void pic32_uart_put_char(pic32_t *s, int unit, unsigned char byte)
{
    uart_t *u = &s->uart[unit];

    if (getenv("QEMU_PIC32_TRACE_UART")) {
        fprintf(stderr, "pic32 uart%u TX 0x%02x\n", unit, byte);
    }

    if (! u->chr) {
        printf("--- %s(unit = %u) serial port not configured\n",
            __func__, unit);
        return;
    }

    /* Send the byte. */
    if (qemu_chr_fe_write(u->chr, &byte, 1) != 1) {
        //TODO: suspend simulation until serial port ready
        printf("--- %s(unit = %u) failed\n", __func__, unit);
        return;
    }

    if ((VALUE(u->mode) & PIC32_UMODE_ON) &&
        (VALUE(u->sta) & PIC32_USTA_UTXEN))
    {
        VALUE(u->sta) |= PIC32_USTA_UTXBF;
        VALUE(u->sta) &= ~PIC32_USTA_TRMT;

        /* Generate TX interrupt with some delay. */
        timer_mod(u->transmit_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
            get_ticks_per_sec() / 5000);
        u->oactive = 1;
    }
}

/*
 * Called before reading a value of UxBRG, UxMODE or UxSTA registers.
 */
void pic32_uart_poll_status(pic32_t *s, int unit)
{
    uart_t *u = &s->uart[unit];

    // Keep receiver idle, transmit shift register always empty
    VALUE(u->sta) |= PIC32_USTA_RIDLE;

    //printf("<%x>", VALUE(u->sta)); fflush(stdout);
}

/*
 * Write to UxMODE register.
 */
void pic32_uart_update_mode(pic32_t *s, int unit)
{
    uart_t *u = &s->uart[unit];

    if (! (VALUE(u->mode) & PIC32_UMODE_ON)) {
        s->irq_clear(s, u->irq + UART_IRQ_RX);
        s->irq_clear(s, u->irq + UART_IRQ_TX);
        VALUE(u->sta) &= ~(PIC32_USTA_URXDA | PIC32_USTA_FERR |
                           PIC32_USTA_PERR | PIC32_USTA_UTXBF);
        VALUE(u->sta) |= PIC32_USTA_RIDLE | PIC32_USTA_TRMT;
    }
}

/*
 * Write to UxSTA register.
 */
void pic32_uart_update_status(pic32_t *s, int unit)
{
    uart_t *u = &s->uart[unit];

    if (! (VALUE(u->sta) & PIC32_USTA_URXEN)) {
        s->irq_clear(s, u->irq + UART_IRQ_RX);
        VALUE(u->sta) &= ~(PIC32_USTA_URXDA | PIC32_USTA_FERR |
                           PIC32_USTA_PERR);
    }
    if (! (VALUE(u->sta) & PIC32_USTA_UTXEN)) {
        s->irq_clear(s, u->irq + UART_IRQ_TX);
        VALUE(u->sta) &= ~PIC32_USTA_UTXBF;
        VALUE(u->sta) |= PIC32_USTA_TRMT;
    }
}

/*
 * When firmware sets the UART TX interrupt enable with data waiting (or an
 * empty transmitter after IFSCLR), hardware asserts UxTXIF. Without this,
 * interrupt-driven TX in the guest may never run.
 */
void pic32_uart_on_tx_ie_enabled(pic32_t *s, int unit, uint32_t prev_iec,
                                 uint32_t new_iec)
{
    uart_t *u = &s->uart[unit];
    unsigned tx_irq = u->irq + UART_IRQ_TX;
    uint32_t tx_ie_mask = 1u << (tx_irq & 31);

    if (!(new_iec & tx_ie_mask) || (prev_iec & tx_ie_mask)) {
        return;
    }
    if (!u->chr) {
        return;
    }
    if ((VALUE(u->mode) & PIC32_UMODE_ON) &&
        (VALUE(u->sta) & PIC32_USTA_UTXEN) &&
        !(VALUE(u->sta) & PIC32_USTA_UTXBF)) {
        s->irq_raise(s, u->irq + UART_IRQ_TX);
    }
}

/*
 * Return a number of free bytes in the receive FIFO.
 */
static int uart_can_receive(void *opaque)
{
    uart_t *u = opaque;
    pic32_t *s = u->mcu;        /* used in VALUE() */

//printf("--- %s(%p) called\n", __func__, u);

    if (! (VALUE(u->mode) & PIC32_UMODE_ON) ||
        ! (VALUE(u->sta) & PIC32_USTA_URXEN)) {
        /* UART disabled. */
        return 0;
    }

    if (u->rx_len >= PIC32_UART_RX_FIFO) {
        return 0;
    }
    return 1;
}

/*
 * Process the received data.
 */
static void uart_receive(void *opaque, const uint8_t *buf, int size)
{
    uart_t *u = opaque;
    pic32_t *s = u->mcu;        /* used in VALUE() */
    int i;

    if (! (VALUE(u->mode) & PIC32_UMODE_ON) ||
        ! (VALUE(u->sta) & PIC32_USTA_URXEN)) {
        /* UART disabled. */
        return;
    }

    /* Char backend may deliver multiple bytes per callback; queue all that fit. */
    for (i = 0; i < size; i++) {
        if (u->rx_len >= PIC32_UART_RX_FIFO) {
            break;
        }
        u->rx_fifo[(u->rx_rd + u->rx_len) % PIC32_UART_RX_FIFO] = buf[i];
        u->rx_len++;
    }
    if (i > 0) {
        VALUE(u->sta) |= PIC32_USTA_URXDA;
        s->irq_raise(s, u->irq + UART_IRQ_RX);
    }
}

/*
 * Activate the transmit interrupt.
 */
static void uart_timeout(void *opaque)
{
    uart_t *u = opaque;
    pic32_t *s = u->mcu;        /* used in VALUE() */

//printf("--- %s() called\n", __func__);
    if (u->oactive) {
        /* Activate transmit interrupt. */
//printf("uart%u: raise tx irq %u\n", unit, u->irq + UART_IRQ_TX);
        if ((VALUE(u->mode) & PIC32_UMODE_ON) &&
            (VALUE(u->sta) & PIC32_USTA_UTXEN))
            s->irq_raise(s, u->irq + UART_IRQ_TX);
        VALUE(u->sta) &= ~PIC32_USTA_UTXBF;
        VALUE(u->sta) |= PIC32_USTA_TRMT;
        u->oactive = 0;
    }
}

/*
 * Initialize the UART data structure.
 */
void pic32_uart_init(pic32_t *s, int unit, int irq, int sta, int mode)
{
    uart_t *u = &s->uart[unit];

    u->mcu = s;
    u->irq = irq;
    u->sta = sta;
    u->mode = mode;
    u->rx_rd = 0;
    u->rx_len = 0;
    u->transmit_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, uart_timeout, u);

    if (unit >= MAX_SERIAL_PORTS) {
        /* Cannot instantiate so many serial ports. */
        u->chr = 0;
        return;
    }
    u->chr = serial_hds[unit];

    /* Setup callback functions. */
    if (u->chr) {
        qemu_chr_add_handlers(u->chr, uart_can_receive, uart_receive, NULL, u);
    }
}

/*
 * Attach a host char device to a UART after pic32_uart_init (e.g. map -serial stdio to UART3).
 */
void pic32_uart_attach_chr(pic32_t *s, int unit, struct CharDriverState *chr)
{
    uart_t *u = &s->uart[unit];

    u->chr = chr;
    if (chr) {
        qemu_chr_add_handlers(chr, uart_can_receive, uart_receive, NULL, u);
    }
}
