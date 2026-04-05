/*
 * PIC32MX350F256H — UART3 LPBACK + TX ring drained only in UART ISR (grblHAL serial.c pattern).
 *
 * UART1: one-line status on stdio (-serial null -serial stdio -serial null => U1 is stdio).
 * UART3: internal loopback; no IFS2SET hacks — reproduces QEMU pic32mx3-generic TXIF/ISR issues.
 *
 * Build: see Makefile target uart3-isr-mx350.hex
 */
#include <stdint.h>
#include <xc.h>
#include <sys/attribs.h>
#include <pic32m-libs/cp0defs.h>

#define PBCLK_HZ        80000000u
#define UART_BAUD       115200u

#define TB_PRESCALE_256 (7u << 4)

#define TX_QSIZE        32u
#define TX_MASK         (TX_QSIZE - 1u)

static uint8_t txdata[TX_QSIZE];
static volatile uint16_t txhead;
static volatile uint16_t txtail;

#define RX_CAP 8u
static uint8_t rxcap[RX_CAP];
static volatile uint8_t rxn;

static volatile uint32_t tx_isr_writes;

static const uint8_t pat[] = { 0x55u, 0xAAu, 0x5Au, 0xA5u };

static void u1_init(void)
{
    U1MODE = 0;
    U1STA = 0;
    U1BRG = (PBCLK_HZ / (16u * UART_BAUD)) - 1u;
    U1MODE = 0x8000u;
    U1STA = (1u << 10) | (1u << 12);
}

static void u1_putc(char c)
{
    *(volatile uint32_t *)&U1TXREG = (unsigned char)c;
}

static void u1_puts(const char *s)
{
    while (*s) {
        u1_putc(*s++);
    }
}

/*
 * ~1 ms delay using Timer3 period match (polled), same style as uart-cli main.c timerN_sleep_ms.
 * No T3 CPU interrupt — avoids relying on T3 ISR + UART ISR interaction under QEMU.
 */
static void timer3_init_poll_1ms(void)
{
    uint32_t pr = (PBCLK_HZ / 256u / 1000u);

    if (pr > 0u) {
        pr--;
    }
    T3CON = 0;
    TMR3 = 0;
    PR3 = (uint16_t)pr;
    IFS0CLR = _IFS0_T3IF_MASK;
    IEC0CLR = _IEC0_T3IE_MASK;
    T3CON = TB_PRESCALE_256 | 0x8000u;
}

static void poll_delay_1ms(void)
{
    IFS0CLR = _IFS0_T3IF_MASK;
    while ((IFS0 & _IFS0_T3IF_MASK) == 0) {
    }
}

void __ISR(_UART_3_VECTOR, IPL2AUTO) uart3_isr(void)
{
    if (IFS1bits.U3RXIF) {
        while (U3STAbits.URXDA) {
            uint8_t d = (uint8_t)U3RXREG;
            if (rxn < RX_CAP) {
                rxcap[rxn++] = d;
            }
        }
        IFS1CLR = _IFS1_U3RXIF_MASK;
    }

    if (IFS2bits.U3TXIF && IEC2bits.U3TXIE) {
        uint_fast16_t tail = txtail;
        if (tail != txhead) {
            U3TXREG = txdata[tail];
            txtail = (uint16_t)((tail + 1u) & TX_MASK);
            tx_isr_writes++;
        }
        if (txtail == txhead) {
            IEC2CLR = _IEC2_U3TXIE_MASK;
        }
        IFS2CLR = _IFS2_U3TXIF_MASK;
    }
}

static int u3_putc_enqueue(uint8_t c)
{
    volatile uint32_t guard = 0;
    uint16_t next;

    for (;;) {
        uint16_t t = txtail;
        next = (uint16_t)((txhead + 1u) & TX_MASK);
        if (next != t) {
            break;
        }
        if (++guard > 5000000u) {
            return -1;
        }
    }
    txdata[txhead] = c;
    txhead = next;
    IEC2SET = _IEC2_U3TXIE_MASK;
    return 0;
}

static void u3_hw_init_lpback(void)
{
    U3MODE = 0;
    U3STA = 0;
    U3STAbits.UTXEN = 1;
    U3STAbits.URXEN = 1;
    U3MODEbits.BRGH = 0;
    U3BRG = (uint16_t)((PBCLK_HZ / (16u * UART_BAUD)) - 1u);
    U3MODEbits.LPBACK = 1;
    U3MODEbits.UEN = 0;
    U3MODEbits.ON = 1;

    IPC9bits.U3IP = 2;
    IPC9bits.U3IS = 0;

    while (U3STAbits.URXDA) {
        (void)U3RXREG;
    }

    IFS1CLR = _IFS1_U3RXIF_MASK;
    IFS2CLR = _IFS2_U3TXIF_MASK;
    IEC1SET = _IEC1_U3RXIE_MASK;
    IEC2CLR = _IEC2_U3TXIE_MASK;
}

static void run_uart3_isr_lpback(void)
{
    unsigned i;
    uint32_t elapsed;

    txhead = 0;
    txtail = 0;
    rxn = 0;
    tx_isr_writes = 0;

    __builtin_disable_interrupts();
    IEC1CLR = _IEC1_U3RXIE_MASK;
    IEC2CLR = _IEC2_U3TXIE_MASK;
    IFS1CLR = _IFS1_U3RXIF_MASK;
    IFS2CLR = _IFS2_U3TXIF_MASK;
    U3MODEbits.ON = 0;

    u3_hw_init_lpback();

    __builtin_enable_interrupts();

    for (i = 0; i < sizeof pat; i++) {
        if (u3_putc_enqueue(pat[i]) != 0) {
            u1_puts("ERR uart3_isr_lpback 1\r\n");
            return;
        }
    }

    elapsed = 0;
    while (txtail != txhead) {
        if (elapsed >= 500u) {
            u1_puts("ERR uart3_isr_lpback 2\r\n");
            return;
        }
        poll_delay_1ms();
        elapsed++;
    }

    elapsed = 0;
    while (rxn < sizeof pat) {
        if (elapsed >= 500u) {
            u1_puts("ERR uart3_isr_lpback 3\r\n");
            return;
        }
        poll_delay_1ms();
        elapsed++;
    }

    for (i = 0; i < sizeof pat; i++) {
        if (rxcap[i] != pat[i]) {
            u1_puts("ERR uart3_isr_lpback 4\r\n");
            return;
        }
    }

    if (tx_isr_writes < sizeof pat) {
        u1_puts("ERR uart3_isr_lpback 5\r\n");
        return;
    }

    u1_puts("OK uart3_isr_lpback\r\n");
}

int main(void)
{
    INTCONbits.MVEC = 1;

    u1_init();
    timer3_init_poll_1ms();

    __builtin_enable_interrupts();

    u1_puts("\r\nuart3_isr_lpback: starting\r\n");
    run_uart3_isr_lpback();

    for (;;) {
    }
}
