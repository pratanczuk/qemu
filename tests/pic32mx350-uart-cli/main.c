/*
 * PIC32MX350F256H — minimal multi-UART console + Timer1–Timer5 delay tests.
 * Built with XC32; intended for QEMU pic32mx3-generic and real silicon.
 */
#include <stdint.h>
#include <string.h>
#include <xc.h>
#include <sys/attribs.h>
#include <pic32m-libs/cp0defs.h>

/* PBCLK = SYSCLK = 80 MHz (matches QEMU default DEVCFG / OSCCON model). */
#define PBCLK_HZ        80000000u
#define UART_BAUD       115200u

#define LINE_MAX        64
#define NUM_UARTS       3
#define NUM_ADC_CH      16u

static char line[NUM_UARTS][LINE_MAX + 1];
static uint8_t pos[NUM_UARTS];

static void uart_putc(unsigned u, char c)
{
    volatile uint32_t *tx;

    switch (u) {
    case 0:
        tx = &U1TXREG;
        break;
    case 1:
        tx = &U2TXREG;
        break;
    default:
        tx = &U3TXREG;
        break;
    }
    /*
     * Do not spin on UTXBF: in LPBACK (uart3 isr test) a tight wait can deadlock
     * with QEMU's TX completion timer; the CLI prints status after restoring UART.
     */
    *tx = (uint32_t)(unsigned char)c;
}

static void uart_puts(unsigned u, const char *s)
{
    while (*s) {
        uart_putc(u, *s++);
    }
}

static int uart_getc_ready(unsigned u)
{
    volatile uint32_t *sta = (u == 0) ? &U1STA : (u == 1) ? &U2STA : &U3STA;
    return ((*sta) & 1u) != 0; /* URXDA */
}

static int uart_getc(unsigned u)
{
    volatile uint32_t *rx = (u == 0) ? &U1RXREG : (u == 1) ? &U2RXREG : &U3RXREG;
    return (int)(*rx & 0xFFu);
}

static void uart_hw_init(unsigned u)
{
    volatile uint32_t *mode;
    volatile uint32_t *sta;
    volatile uint32_t *brg;

    switch (u) {
    case 0:
        mode = &U1MODE;
        sta = &U1STA;
        brg = &U1BRG;
        break;
    case 1:
        mode = &U2MODE;
        sta = &U2STA;
        brg = &U2BRG;
        break;
    default:
        mode = &U3MODE;
        sta = &U3STA;
        brg = &U3BRG;
        break;
    }

    *mode = 0;
    *sta = 0;
    *brg = (PBCLK_HZ / (16u * UART_BAUD)) - 1u;

    /* 8N1, BRGH=0, UART ON (bit 15). */
    *mode = 0x8000u;
    /* UTXEN bit 10, URXEN bit 12 */
    *sta = (1u << 10) | (1u << 12);
}

static void print_help(unsigned u)
{
    uart_puts(u, "Commands:\r\n"
                 "  help        - this text\r\n"
                 "  uart        - which UART this session is (1-3 on MX350)\r\n"
                 "  switch N    - verify this line is UART N (1-3); not a mux —\r\n"
                 "                U2/U3 need their own host -serial / TCP port\r\n"
                 "  timer       - Timer3 polled delay ~10 s (100 x 100 ms)\r\n"
                 "  timer_ms N  - Timer3 delay ~N ms (prescale 256)\r\n"
                 "  timmer_ms N - same as timer_ms (alias)\r\n"
                 "  timer1_ms N - Timer1 delay ~N ms (Type A, /256)\r\n"
                 "  timer2_ms N - Timer2 delay ~N ms (Type B, /256)\r\n"
                 "  timer3_ms N - Timer3 delay ~N ms (same as timer_ms)\r\n"
                 "  timer4_ms N - Timer4 delay ~N ms (Type B, /256)\r\n"
                 "  timer5_ms N - Timer5 delay ~N ms (Type B, /256)\r\n"
                 "  adc all     - sample AN0-AN15 (10-bit), one line per ch\r\n"
                 "  ocmp all    - OC1-OC5 IF with T2, then OC1 with T3\r\n"
                 "  pmp         - PMP master PMDOUT->PMDIN (QEMU XOR stub)\r\n"
                 "  misc phys   - RCON/CVR regs; IC1+CMP stubs (IC/CMP QEMU-only)\r\n"
                 "  isr flags   - IFS0/1/2 SET/CLR each irq 0-75 (IEC masked)\r\n"
                 "  isr ct      - CP0 Count/Compare -> CTIF (IFS0 bit 0)\r\n"
                 "  i2c all     - I2C1/2 TRN stub RCV+XOR + I2CxMIF (QEMU)\r\n"
                 "  nvm all     - NVM page erase + row + word on last flash page\r\n"
                 "                (QEMU mips_pic32mx3 NVM model; needs ~1 KiB scratch)\r\n"
                 "  uart3 isr   - UART3 LPBACK + TX ring drained in ISR (grblHAL-style)\r\n"
                 "  echo X      - print X\r\n");
}

/* Max ~10 min; keeps (ms * 312500) in uint64_t with margin. */
#define TIMER_MS_MAX    600000u

static int parse_u32_dec(const char *s, uint32_t *out)
{
    uint32_t v = 0;

    while (*s == ' ' || *s == '\t') {
        s++;
    }
    if (*s < '0' || *s > '9') {
        return -1;
    }
    while (*s >= '0' && *s <= '9') {
        unsigned dig = (unsigned)(*s++ - '0');
        if (v > (0xFFFFFFFFu - dig) / 10u) {
            return -1;
        }
        v = v * 10u + dig;
    }
    while (*s == ' ' || *s == '\t') {
        s++;
    }
    if (*s != '\0') {
        return -1;
    }
    *out = v;
    return 0;
}

/*
 * Timers T1–T5: pic32mx3-generic models period match + TxIF (IFS0).
 * Type B (T2–T5): TCKPS = 111 -> /256. Type A (T1): TCKPS = 11 -> /256.
 */
#define TB_PRESCALE_256  (7u << 4)   /* Type B TCKPS */
#define T1_PRESCALE_256  (3u << 4)   /* Type A TCKPS 11 -> /256 */
#define PR3_100MS          31249u   /* 100 ms at PBCLK/256 */

static void timer3_sleep_10s(unsigned u)
{
    unsigned n;

    uart_puts(u, "Timer3: starting ~10 s (100 x 100 ms, prescale 256)...\r\n");

    T3CON = 0;
    TMR3 = 0;
    PR3 = PR3_100MS;
    IFS0CLR = _IFS0_T3IF_MASK;
    T3CON = TB_PRESCALE_256 | 0x8000u; /* TON */

    for (n = 0; n < 100u; n++) {
        while ((IFS0 & _IFS0_T3IF_MASK) == 0u) {
        }
        IFS0CLR = _IFS0_T3IF_MASK;
    }

    T3CONCLR = 0x8000u;
    uart_puts(u, "Timer3: done.\r\nOK timer\r\n");
}

static void uart_put_u32(unsigned u, uint32_t t)
{
    char buf[16];
    unsigned i = 0;

    if (t == 0u) {
        buf[i++] = '0';
    } else {
        char tmp[12];
        unsigned j = 0;

        while (t > 0u && j < sizeof tmp) {
            tmp[j++] = (char)('0' + (t % 10u));
            t /= 10u;
        }
        while (j > 0u) {
            buf[i++] = tmp[--j];
        }
    }
    buf[i] = '\0';
    uart_puts(u, buf);
}

/*
 * Arbitrary delay on timer tn (1..5): PBCLK/256 => 312500 Hz tick rate.
 * Match period = (PR+1) counts; max chunk 65536 counts per IRQ.
 */
static void timerN_sleep_ms(unsigned u, int tn, uint32_t ms, const char *err_tag,
                            const char *ok_line,
                            volatile uint32_t *tcon, volatile uint32_t *tmr,
                            volatile uint32_t *pr, uint32_t tcon_run,
                            uint32_t ifs_mask)
{
    uint64_t total;
    uint64_t rem;
    int first;

    if (ms == 0u) {
        uart_puts(u, err_tag);
        uart_puts(u, ": N must be > 0\r\n");
        return;
    }
    if (ms > TIMER_MS_MAX) {
        uart_puts(u, err_tag);
        uart_puts(u, ": N too large\r\n");
        return;
    }
    total = ((uint64_t)ms * 312500u + 999u) / 1000u;
    if (total == 0u) {
        total = 1u;
    }

    uart_puts(u, "Timer");
    uart_putc(u, (char)('0' + tn));
    uart_puts(u, ": delay ~");
    uart_put_u32(u, ms);
    uart_puts(u, " ms...\r\n");

    *tcon = 0;
    *tmr = 0;
    IFS0CLR = ifs_mask;

    rem = total;
    first = 1;
    while (rem > 0u) {
        uint32_t chunk = (rem > 65536u) ? 65536u : (uint32_t)rem;

        *pr = chunk - 1u;
        IFS0CLR = ifs_mask;
        if (first) {
            *tcon = tcon_run | 0x8000u;
            first = 0;
        }
        while ((IFS0 & ifs_mask) == 0u) {
        }
        IFS0CLR = ifs_mask;
        rem -= (uint64_t)chunk;
    }

    *tcon &= ~0x8000u; /* TON off */
    uart_puts(u, ok_line);
}

static void timer3_sleep_ms(unsigned u, uint32_t ms)
{
    timerN_sleep_ms(u, 3, ms, "ERR timer_ms", "OK timer_ms\r\n",
                    &T3CON, &TMR3, &PR3, TB_PRESCALE_256, _IFS0_T3IF_MASK);
}

static void adc_all(unsigned u)
{
    uint32_t ch;

    /*
     * Silicon: AD1CON3=0 can leave TAD out of spec so DONE never sets; set a
     * slow ADCS. Pins must be analog (ANSEL=1) + input (TRIS=1) per data sheet.
     * Boards that map UART onto AN-capable pins may need different ANSEL masks.
     */
    AD1CON1 = 0;
    AD1CON2 = 0x0000u; /* VCFG=AVdd/AVss */
    AD1CON3 = _AD1CON3_ADCS_MASK; /* ADCS=255 => long TAD at 80 MHz PBCLK */
    AD1CHS = 0;
    ANSELBSET = 0xFFFFu;
    ANSELCSET = 0xFFFFu;
    ANSELDSET = 0xFFFFu;
    ANSELESET = 0xFFFFu;
    /* Omit ANSELF/ANSELG: UART PPS often uses RF/RG; set those manually if needed. */

    AD1CON1 = _AD1CON1_ON_MASK; /* SSRC=0 manual */
    for (volatile unsigned w = 0; w < 8000u; w++) {
    }

    for (ch = 0u; ch < NUM_ADC_CH; ch++) {
        uint32_t v;
        uint32_t timeout;

        AD1CHS = (ch & 0x1Fu) << _AD1CHS_CH0SA_POSITION;
        AD1CON1SET = _AD1CON1_SAMP_MASK;
        for (volatile int i = 0; i < 200; i++) {
        }
        AD1CON1CLR = _AD1CON1_SAMP_MASK;
        timeout = 5000000u;
        while (timeout > 0u) {
            if ((AD1CON1 & _AD1CON1_DONE_MASK) != 0u) {
                break;
            }
            if ((IFS0 & _IFS0_AD1IF_MASK) != 0u) {
                break;
            }
            timeout--;
        }
        if (timeout == 0u) {
            uart_puts(u, "ERR adc all: timeout ch ");
            uart_put_u32(u, ch);
            uart_puts(u, "\r\n");
            return;
        }
        v = ADC1BUF0 & 0x3FFu;
        uart_puts(u, "ch ");
        uart_putc(u, (char)('0' + (ch / 10u) % 10u));
        uart_putc(u, (char)('0' + ch % 10u));
        uart_puts(u, " v ");
        uart_put_u32(u, v);
        uart_puts(u, "\r\n");
        AD1CON1CLR = _AD1CON1_DONE_MASK;
        IFS0CLR = _IFS0_AD1IF_MASK;
    }
    uart_puts(u, "OK adc all\r\n");
}

/* OCM != 0 enables QEMU OC IF on timer wrap; dual-compare continuous on silicon. */
#define OCMP_OCM        5U
#define OCMP_PR_SHORT   3124U /* ~10 ms tick @ PBCLK/256, same class as timer_ms */

static uint32_t ocmp_ifs_mask(int oc_num)
{
    switch (oc_num) {
    case 1:
        return _IFS0_OC1IF_MASK;
    case 2:
        return _IFS0_OC2IF_MASK;
    case 3:
        return _IFS0_OC3IF_MASK;
    case 4:
        return _IFS0_OC4IF_MASK;
    default:
        return _IFS0_OC5IF_MASK;
    }
}

static int ocmp_wait_flag(uint32_t m)
{
    uint32_t t;

    for (t = 5000000u; t > 0u; t--) {
        if ((IFS0 & m) != 0u) {
            return 0;
        }
    }
    return -1;
}

static void ocmp_all(unsigned u)
{
    int oc_num;

    /* Phase 1: OC1-OC5 use Timer2 (OCTSEL=0). */
    T2CON = 0;
    TMR2 = 0;
    PR2 = OCMP_PR_SHORT;
    OC1CON = 0;
    OC2CON = 0;
    OC3CON = 0;
    OC4CON = 0;
    OC5CON = 0;
    OC1R = 10U;
    OC1RS = 100U;
    OC2R = 10U;
    OC2RS = 100U;
    OC3R = 10U;
    OC3RS = 100U;
    OC4R = 10U;
    OC4RS = 100U;
    OC5R = 10U;
    OC5RS = 100U;
    IFS0CLR = _IFS0_T2IF_MASK | _IFS0_OC1IF_MASK | _IFS0_OC2IF_MASK
            | _IFS0_OC3IF_MASK | _IFS0_OC4IF_MASK | _IFS0_OC5IF_MASK;
    OC1CON = OCMP_OCM;
    OC2CON = OCMP_OCM;
    OC3CON = OCMP_OCM;
    OC4CON = OCMP_OCM;
    OC5CON = OCMP_OCM;
    T2CON = TB_PRESCALE_256 | 0x8000u;

    for (oc_num = 1; oc_num <= 5; oc_num++) {
        uint32_t m = ocmp_ifs_mask(oc_num);

        if (ocmp_wait_flag(m) != 0) {
            uart_puts(u, "ERR ocmp all: timeout OC");
            uart_putc(u, (char)('0' + oc_num));
            uart_puts(u, " (T2)\r\n");
            T2CONCLR = 0x8000u;
            OC1CON = 0;
            OC2CON = 0;
            OC3CON = 0;
            OC4CON = 0;
            OC5CON = 0;
            return;
        }
        IFS0CLR = m;
        IFS0CLR = _IFS0_T2IF_MASK;
        uart_puts(u, "oc ");
        uart_putc(u, (char)('0' + oc_num));
        uart_puts(u, " t2\r\n");
    }
    T2CONCLR = 0x8000u;
    OC1CON = 0;
    OC2CON = 0;
    OC3CON = 0;
    OC4CON = 0;
    OC5CON = 0;

    /* Phase 2: OC1 on Timer3 (OCTSEL=1). */
    T3CON = 0;
    TMR3 = 0;
    PR3 = OCMP_PR_SHORT;
    OC1CON = 0;
    IFS0CLR = _IFS0_T3IF_MASK | _IFS0_OC1IF_MASK;
    OC1CON = (1u << 3) | OCMP_OCM;
    T3CON = TB_PRESCALE_256 | 0x8000u;
    if (ocmp_wait_flag(_IFS0_OC1IF_MASK) != 0) {
        uart_puts(u, "ERR ocmp all: timeout OC1 (T3)\r\n");
        T3CONCLR = 0x8000u;
        OC1CON = 0;
        return;
    }
    IFS0CLR = _IFS0_OC1IF_MASK;
    IFS0CLR = _IFS0_T3IF_MASK;
    uart_puts(u, "oc 1 t3\r\n");
    T3CONCLR = 0x8000u;
    OC1CON = 0;

    uart_puts(u, "OK ocmp all\r\n");
}

/* Master mode 1 + IRQ at end of cycle; 16-bit uses XOR 0xA5A5 in QEMU PMP stub. */
#define PMP_PMM_MASTER1_16_IRQ    (0x0300u | 0x0400u | 0x2000u)
#define PMP_PMM_MASTER1_8_IRQ     (0x0300u | 0x2000u)

static int pmp_wait_ifs1(uint32_t m)
{
    uint32_t t;

    for (t = 5000000u; t > 0u; t--) {
        if ((IFS1 & m) != 0u) {
            return 0;
        }
    }
    return -1;
}

static void pmp_test(unsigned u)
{
    uint32_t d;

    PMCON = 0;
    PMMODE = 0;
    PMADDR = 0;
    PMDOUT = 0;
    PMDIN = 0;

    PMMODE = PMP_PMM_MASTER1_16_IRQ;
    PMCON = 0x8000u;
    IFS1CLR = _IFS1_PMPIF_MASK;
    PMDOUT = 0xCAFEu;
    if (pmp_wait_ifs1(_IFS1_PMPIF_MASK) != 0) {
        uart_puts(u, "ERR pmp: timeout PMPIF (16-bit)\r\n");
        PMCON = 0;
        PMMODE = 0;
        return;
    }
    IFS1CLR = _IFS1_PMPIF_MASK;
    d = PMDIN & 0xFFFFu;
    uart_puts(u, "pmp16 ");
    uart_put_u32(u, d);
    uart_puts(u, "\r\n");

    PMCON = 0;
    PMMODE = PMP_PMM_MASTER1_8_IRQ;
    PMCON = 0x8000u;
    IFS1CLR = _IFS1_PMPIF_MASK;
    PMDOUT = 0x0012u;
    if (pmp_wait_ifs1(_IFS1_PMPIF_MASK) != 0) {
        uart_puts(u, "ERR pmp: timeout PMPIF (8-bit)\r\n");
        PMCON = 0;
        PMMODE = 0;
        return;
    }
    IFS1CLR = _IFS1_PMPIF_MASK;
    d = PMDIN & 0xFFu;
    uart_puts(u, "pmp8 ");
    uart_put_u32(u, d);
    uart_puts(u, "\r\n");

    PMCON = 0;
    PMMODE = 0;
    uart_puts(u, "OK pmp\r\n");
}

static int misc_wait_ifs0(uint32_t m)
{
    uint32_t t;

    for (t = 5000000u; t > 0u; t--) {
        if ((IFS0 & m) != 0u) {
            return 0;
        }
    }
    return -1;
}

/*
 * RCON + CVRCON: register path (usable on silicon if RCON/CVR masks match).
 * IC1 + comparators: rely on QEMU stubs in mips_pic32mx3.c (instant IC1BUF, CMSTAT).
 */
static void misc_phys(unsigned u)
{
    uint32_t x;
    /* MX350: ON at bit 15, CVR<3:0> in low nibble (see __CVRCONbits_t). */
    uint32_t cvr_pat = 0x8000u | 15u;

    /* RCON: set/clear SWR (0x40) via SET/CLR aliases. */
    RCONSET = 0x40u;
    x = RCON;
    if ((x & 0x40u) == 0u) {
        uart_puts(u, "ERR misc phys: RCON SWR not set\r\n");
        return;
    }
    RCONCLR = 0x40u;
    x = RCON;
    if ((x & 0x40u) != 0u) {
        uart_puts(u, "ERR misc phys: RCON SWR not clear\r\n");
        return;
    }
    uart_puts(u, "rcon ok\r\n");

    CVRCON = 0;
    CVRCON = cvr_pat;
    x = CVRCON & 0xFFFFu;
    uart_puts(u, "cvr ");
    uart_put_u32(u, x);
    uart_puts(u, "\r\n");

    /* IC1: ICM=1 every edge, Timer2 source, ON — QEMU latches TMR2 into IC1BUF + IC1IF. */
    T2CON = 0;
    TMR2 = 0x55AAu;
    IC1CON = 0;
    IFS0CLR = _IFS0_IC1IF_MASK;
    IC1CON = 0x8000u | 1u; /* ON | ICM simple capture mode */
    if (misc_wait_ifs0(_IFS0_IC1IF_MASK) != 0) {
        uart_puts(u, "ERR misc phys: timeout IC1IF\r\n");
        IC1CON = 0;
        return;
    }
    IFS0CLR = _IFS0_IC1IF_MASK;
    x = IC1BUF & 0xFFFFu;
    uart_puts(u, "ic1 ");
    uart_put_u32(u, x);
    uart_puts(u, "\r\n");
    IC1CON = 0;

    /* Comparators: ON sets CxOUT in CMSTAT + CMPxIF (IFS1 bits 0/1) in QEMU. */
    CM1CON = 0;
    CM2CON = 0;
    IFS1CLR = _IFS1_CMP1IF_MASK | _IFS1_CMP2IF_MASK;
    CM1CON = 0x8000u;
    if (pmp_wait_ifs1(_IFS1_CMP1IF_MASK) != 0) {
        uart_puts(u, "ERR misc phys: timeout CMP1IF\r\n");
        CM1CON = 0;
        CM2CON = 0;
        return;
    }
    IFS1CLR = _IFS1_CMP1IF_MASK;
    x = CMSTAT & 3u;
    uart_puts(u, "cm ");
    uart_put_u32(u, x);
    uart_puts(u, " ");
    CM2CON = 0x8000u;
    if (pmp_wait_ifs1(_IFS1_CMP2IF_MASK) != 0) {
        uart_puts(u, "ERR misc phys: timeout CMP2IF\r\n");
        CM1CON = 0;
        CM2CON = 0;
        return;
    }
    IFS1CLR = _IFS1_CMP2IF_MASK;
    x = CMSTAT & 3u;
    uart_put_u32(u, x);
    uart_puts(u, "\r\n");
    CM1CON = 0;
    CM2CON = 0;

    uart_puts(u, "OK misc phys\r\n");
}

/* Matches mips_pic32mx3.c irq_to_vector[] length (IFS0+IFS1+MX350 IFS2 subset). */
#define PIC32MX350_N_IRQ  76u

static int ifs_bit_probe(volatile uint32_t *ifs, volatile uint32_t *ifsclr,
                         volatile uint32_t *ifsset, uint32_t bit)
{
    uint32_t m = 1u << bit;

    *ifsclr = m;
    if ((*ifs & m) != 0u) {
        return -1;
    }
    *ifsset = m;
    if ((*ifs & m) == 0u) {
        return -2;
    }
    *ifsclr = m;
    if ((*ifs & m) != 0u) {
        return -3;
    }
    return 0;
}

/*
 * Exercise every interrupt flag bit via SET/CLR aliases (IEC* = 0 so no CPU taken).
 * Silicon: reserved IFS bits may not exist; QEMU models a full 76-bit span.
 */
static void isr_flags_all(unsigned u)
{
    uint32_t save_iec0 = IEC0;
    uint32_t save_iec1 = IEC1;
    uint32_t save_iec2 = IEC2;
    uint32_t irq;

    IEC0 = 0;
    IEC1 = 0;
    IEC2 = 0;

    for (irq = 0u; irq < PIC32MX350_N_IRQ; irq++) {
        uint32_t bi = irq & 31u;
        int e;

        if (irq < 32u) {
            e = ifs_bit_probe(&IFS0, &IFS0CLR, &IFS0SET, bi);
        } else if (irq < 64u) {
            e = ifs_bit_probe(&IFS1, &IFS1CLR, &IFS1SET, bi);
        } else {
            e = ifs_bit_probe(&IFS2, &IFS2CLR, &IFS2SET, bi);
        }
        if (e != 0) {
            uart_puts(u, "ERR isr flags irq ");
            uart_put_u32(u, irq);
            uart_puts(u, "\r\n");
            goto restore_iec;
        }
    }
    uart_puts(u, "OK isr flags\r\n");

restore_iec:
    IEC0 = save_iec0;
    IEC1 = save_iec1;
    IEC2 = save_iec2;
}

/* Core timer: CP0 Compare match raises CTIF via QEMU cputimer + pic32_timer_irq. */
static void isr_ct(unsigned u)
{
    uint32_t c;
    uint32_t tmo;

    IFS0CLR = _IFS0_CTIF_MASK;
    c = _CP0_GET_COUNT();
    _CP0_SET_COMPARE(c + 200000u);
    for (tmo = 5000000u; tmo > 0u; tmo--) {
        if ((IFS0 & _IFS0_CTIF_MASK) != 0u) {
            break;
        }
    }
    IFS0CLR = _IFS0_CTIF_MASK;
    c = _CP0_GET_COUNT();
    _CP0_SET_COMPARE(c + 0x7FFFFFFFu);
    if (tmo == 0u) {
        uart_puts(u, "ERR isr ct: timeout CTIF\r\n");
        return;
    }
    uart_puts(u, "OK isr ct\r\n");
}

/* I2C: QEMU latches TRN^xor into RCV and sets I2CxMIF when I2CxCON ON. */
static void i2c_all(unsigned u)
{
    uint32_t v;

    I2C1CON = 0;
    I2C2CON = 0;
    IFS1CLR = _IFS1_I2C1MIF_MASK | _IFS1_I2C2MIF_MASK;

    I2C1CON = 0x8000u;
    I2C1TRN = 0x34u;
    if (pmp_wait_ifs1(_IFS1_I2C1MIF_MASK) != 0) {
        uart_puts(u, "ERR i2c all: timeout I2C1MIF\r\n");
        I2C1CON = 0;
        return;
    }
    IFS1CLR = _IFS1_I2C1MIF_MASK;
    v = I2C1RCV & 0xFFu;
    uart_puts(u, "i2c1 ");
    uart_put_u32(u, v);
    uart_puts(u, "\r\n");
    I2C1CON = 0;

    I2C2CON = 0x8000u;
    I2C2TRN = 0x34u;
    if (pmp_wait_ifs1(_IFS1_I2C2MIF_MASK) != 0) {
        uart_puts(u, "ERR i2c all: timeout I2C2MIF\r\n");
        I2C2CON = 0;
        return;
    }
    IFS1CLR = _IFS1_I2C2MIF_MASK;
    v = I2C2RCV & 0xFFu;
    uart_puts(u, "i2c2 ");
    uart_put_u32(u, v);
    uart_puts(u, "\r\n");
    I2C2CON = 0;

    uart_puts(u, "OK i2c all\r\n");
}

/*
 * NVM tests — PIC32MX NVMCON sequence (matches QEMU pic32mx3_nvm.c).
 * NVMCON_* / NVMOP values come from xc.h (ppic32mx.h).
 * Uses the last 1 KiB program page (0x1D03FC00); uart-cli.hex does not occupy it.
 */
#define NVMOP_WORD          NVMCON_NVMOP0                     /* word program */
#define NVMOP_ROW           (NVMCON_NVMOP0 | NVMCON_NVMOP1)   /* row program */
#define NVMOP_PAGE_ERASE    NVMCON_NVMOP2                     /* page erase */

/* Last page in 256 KiB program flash (1 KiB page, 512 B row). */
#define NVM_TEST_PHYS_PAGE  0x1D03FC00u

static uint8_t nvm_row_src[512] __attribute__((aligned(4)));

static void nvm_wrerr_clear(void)
{
    if (NVMCON & NVMCON_WRERR) {
        NVMCONCLR = NVMCON_WRERR;
    }
}

static int nvm_wait_done(unsigned u, const char *where)
{
    while ((NVMCON & NVMCON_WR) != 0u) {
    }
    if ((NVMCON & NVMCON_WRERR) != 0u) {
        uart_puts(u, where);
        uart_puts(u, ": WRERR\r\n");
        return -1;
    }
    return 0;
}

static int nvm_page_erase(unsigned u, uint32_t page_pa)
{
    nvm_wrerr_clear();
    /* Datasheet order: NVMCON (WREN|OP), NVMADDR, NVMKEY, WR */
    NVMCON = NVMCON_WREN | NVMOP_PAGE_ERASE;
    __asm__ volatile ("" ::: "memory");
    NVMADDR = page_pa;
    __asm__ volatile ("" ::: "memory");
    NVMKEY = 0xAA996655u;
    NVMKEY = 0x556699AAu;
    NVMCONSET = NVMCON_WR;
    return nvm_wait_done(u, "ERR nvm page_erase");
}

static int nvm_row_program(unsigned u, uint32_t row_pa, const void *src)
{
    nvm_wrerr_clear();
    NVMCON = NVMCON_WREN | NVMOP_ROW;
    __asm__ volatile ("" ::: "memory");
    NVMADDR = row_pa;
    __asm__ volatile ("" ::: "memory");
    NVMSRCADDR = (uint32_t)(uintptr_t)src;
    __asm__ volatile ("" ::: "memory");
    NVMKEY = 0xAA996655u;
    NVMKEY = 0x556699AAu;
    NVMCONSET = NVMCON_WR;
    return nvm_wait_done(u, "ERR nvm row");
}

static int nvm_word_program(unsigned u, uint32_t word_pa, uint32_t data)
{
    nvm_wrerr_clear();
    NVMCON = NVMCON_WREN | NVMOP_WORD;
    /* Barriers: XC32 may reorder SFR stores; NVM hardware is order-sensitive. */
    __asm__ volatile ("" ::: "memory");
    NVMADDR = word_pa;
    __asm__ volatile ("" ::: "memory");
    NVMDATA = data;
    __asm__ volatile ("" ::: "memory");
    NVMKEY = 0xAA996655u;
    NVMKEY = 0x556699AAu;
    NVMCONSET = NVMCON_WR;
    return nvm_wait_done(u, "ERR nvm word");
}

static void nvm_all(unsigned u)
{
    unsigned i;

    uart_puts(u, "nvm: test page ");
    uart_put_u32(u, NVM_TEST_PHYS_PAGE);
    uart_puts(u, "\r\n");

    if (nvm_page_erase(u, NVM_TEST_PHYS_PAGE) != 0) {
        uart_puts(u, "ERR nvm all: page_erase\r\n");
        return;
    }
    uart_puts(u, "nvm: page_erase ok\r\n");

    for (i = 0; i < sizeof nvm_row_src; i++) {
        nvm_row_src[i] = 0x5Au;
    }
    if (nvm_row_program(u, NVM_TEST_PHYS_PAGE, nvm_row_src) != 0) {
        uart_puts(u, "ERR nvm all: row\r\n");
        return;
    }
    uart_puts(u, "nvm: row ok\r\n");

    if (nvm_word_program(u, NVM_TEST_PHYS_PAGE + 512u + 16u, 0xDEADBEEFu) != 0) {
        uart_puts(u, "ERR nvm all: word\r\n");
        return;
    }
    uart_puts(u, "nvm: word ok\r\n");

    /*
     * We do not read flash back here: under QEMU the guest data path may not
     * yet see NVM-programmed words the same as silicon; WRERR/WR are enough
     * to validate pic32mx3_nvm.c + NVMKEY/NVMCON sequence.
     */
    uart_puts(u, "OK nvm all\r\n");
}

/* --- uart3 isr: grblHAL-style TX ring + U3TXIE; LPBACK; T4 polled 1 ms (keeps T3 for other cmds) --- */

#define U3ISR_TX_QSIZE  32u
#define U3ISR_TX_MASK   (U3ISR_TX_QSIZE - 1u)
#define U3ISR_RX_CAP    8u

static uint8_t u3isr_txdata[U3ISR_TX_QSIZE];
static volatile uint16_t u3isr_txhead;
static volatile uint16_t u3isr_txtail;
static uint8_t u3isr_rxcap[U3ISR_RX_CAP];
static volatile uint8_t u3isr_rxn;
static volatile uint32_t u3isr_tx_writes;

static const uint8_t u3isr_pat[] = { 0x55u, 0xAAu, 0x5Au, 0xA5u };

void __ISR(_UART_3_VECTOR, IPL2AUTO) uart3_isr_grblhal_test(void)
{
    if (IFS1bits.U3RXIF) {
        unsigned n;

        /* Fixed iteration cap: host serial + LPBACK can refill RX; never spin forever. */
        for (n = 0; n < 32u && U3STAbits.URXDA; n++) {
            uint8_t d = (uint8_t)U3RXREG;
            if (u3isr_rxn < U3ISR_RX_CAP) {
                u3isr_rxcap[u3isr_rxn++] = d;
            }
        }
        IFS1CLR = _IFS1_U3RXIF_MASK;
    }

    if (IFS2bits.U3TXIF && IEC2bits.U3TXIE) {
        uint_fast16_t tail = u3isr_txtail;
        if (tail != u3isr_txhead) {
            U3TXREG = u3isr_txdata[tail];
            u3isr_txtail = (uint16_t)((tail + 1u) & U3ISR_TX_MASK);
            u3isr_tx_writes++;
        }
        if (u3isr_txtail == u3isr_txhead) {
            IEC2CLR = _IEC2_U3TXIE_MASK;
        }
        IFS2CLR = _IFS2_U3TXIF_MASK;
    }
}

/*
 * Guest ~1 ms pacing for uart3 isr waits. T4IF polling can miss under QEMU when
 * the UART ISR runs often; a bounded busy loop matches wall time well enough for
 * the CLI timeout counters (silicon may run slightly faster).
 */
static void u3isr_poll_delay_1ms(void)
{
    volatile uint32_t k;

    for (k = 0; k < 500u; k++) {
        __asm__ __volatile__("" ::: "memory");
    }
}

static int u3isr_enqueue(uint8_t c)
{
    volatile uint32_t guard = 0;
    uint16_t next;

    for (;;) {
        uint16_t t = u3isr_txtail;
        next = (uint16_t)((u3isr_txhead + 1u) & U3ISR_TX_MASK);
        if (next != t) {
            break;
        }
        if (++guard > 5000000u) {
            return -1;
        }
    }
    u3isr_txdata[u3isr_txhead] = c;
    u3isr_txhead = next;
    IEC2SET = _IEC2_U3TXIE_MASK;
    return 0;
}

static void u3isr_hw_lpback_init(void)
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

    /* Bound flush: under QEMU a host -serial backend may refill faster than we
     * drain, which would spin forever here (not possible on bare silicon). */
    {
        unsigned nflush = 0;

        while (U3STAbits.URXDA && nflush < 64u) {
            (void)U3RXREG;
            nflush++;
        }
    }

    IFS1CLR = _IFS1_U3RXIF_MASK;
    IFS2CLR = _IFS2_U3TXIF_MASK;
    IEC1SET = _IEC1_U3RXIE_MASK;
    IEC2CLR = _IEC2_U3TXIE_MASK;
}

/*
 * XC32 -O2 miscompiles this routine (success path branches to function entry
 * instead of restore / OK print). Keep this unit at -O0.
 */
static void __attribute__((noinline, optimize("O0"))) uart3_isr_lpback_cmd(unsigned u)
{
    unsigned i;
    uint32_t elapsed;
    const char *result = NULL;

    uart_puts(u, "uart3 isr: LPBACK + TX ring (grblHAL-style)...\r\n");

    u3isr_txhead = 0;
    u3isr_txtail = 0;
    u3isr_rxn = 0;
    u3isr_tx_writes = 0;

    __builtin_disable_interrupts();
    IEC1CLR = _IEC1_U3RXIE_MASK;
    IEC2CLR = _IEC2_U3TXIE_MASK;
    IFS1CLR = _IFS1_U3RXIF_MASK;
    IFS2CLR = _IFS2_U3TXIF_MASK;
    U3MODEbits.ON = 0;

    u3isr_hw_lpback_init();

    __builtin_enable_interrupts();

    for (i = 0; i < sizeof u3isr_pat; i++) {
        if (u3isr_enqueue(u3isr_pat[i]) != 0) {
            result = "ERR uart3 isr 1\r\n";
            goto restore;
        }
    }

    elapsed = 0;
    while (u3isr_txtail != u3isr_txhead) {
        if (elapsed >= 200u) {
            result = "ERR uart3 isr 2\r\n";
            goto restore;
        }
        u3isr_poll_delay_1ms();
        elapsed++;
    }

    elapsed = 0;
    while (u3isr_rxn < sizeof u3isr_pat) {
        if (elapsed >= 200u) {
            result = "ERR uart3 isr 3\r\n";
            goto restore;
        }
        u3isr_poll_delay_1ms();
        elapsed++;
    }

    for (i = 0; i < sizeof u3isr_pat; i++) {
        if (u3isr_rxcap[i] != u3isr_pat[i]) {
            result = "ERR uart3 isr 4\r\n";
            goto restore;
        }
    }

    if (u3isr_tx_writes < sizeof u3isr_pat) {
        result = "ERR uart3 isr 5\r\n";
        goto restore;
    }

    result = "OK uart3 isr\r\n";

restore:
    if (result == NULL) {
        result = "ERR uart3 isr: internal\r\n";
    }
    __builtin_disable_interrupts();
    IEC1CLR = _IEC1_U3RXIE_MASK;
    IEC2CLR = _IEC2_U3TXIE_MASK;
    IFS1CLR = _IFS1_U3RXIF_MASK;
    IFS2CLR = _IFS2_U3TXIF_MASK;
    U3MODEbits.ON = 0;
    uart_hw_init(2);
    uart_puts(u, result);
    __builtin_enable_interrupts();
}

static void run_cmd(unsigned u, char *cmd)
{
    while (*cmd == ' ' || *cmd == '\t') {
        cmd++;
    }
    if (cmd[0] == '\0') {
        return;
    }
    if (strcmp(cmd, "help") == 0) {
        print_help(u);
        uart_puts(u, "OK help\r\n");
    } else if (strcmp(cmd, "uart") == 0) {
        uart_puts(u, "OK uart ");
        uart_putc(u, (char)('1' + (int)u));
        uart_puts(u, "\r\n");
    } else if (strncmp(cmd, "switch ", 7) == 0) {
        uint32_t n;

        if (parse_u32_dec(cmd + 7, &n) != 0) {
            uart_puts(u, "ERR switch: need integer N (1-3)\r\n");
        } else if (n < 1u || n > (uint32_t)NUM_UARTS) {
            uart_puts(u, "ERR switch: N must be 1-3 (MX350)\r\n");
        } else if ((unsigned)(n - 1u) != u) {
            uart_puts(u, "ERR switch: this is UART ");
            uart_putc(u, (char)('1' + (int)u));
            uart_puts(u, ", not ");
            uart_put_u32(u, n);
            uart_puts(u, "; open that UART on another host serial\r\n");
        } else {
            uart_puts(u, "OK switch ");
            uart_put_u32(u, n);
            uart_puts(u, "\r\n");
        }
    } else if (strcmp(cmd, "timer") == 0) {
        timer3_sleep_10s(u);
    } else if (strncmp(cmd, "timer_ms ", 9) == 0) {
        uint32_t ms;
        if (parse_u32_dec(cmd + 9, &ms) != 0) {
            uart_puts(u, "ERR timer_ms: need integer N\r\n");
        } else {
            timer3_sleep_ms(u, ms);
        }
    } else if (strncmp(cmd, "timmer_ms ", 10) == 0) {
        uint32_t ms;
        if (parse_u32_dec(cmd + 10, &ms) != 0) {
            uart_puts(u, "ERR timmer_ms: need integer N\r\n");
        } else {
            timer3_sleep_ms(u, ms);
        }
    } else if (strncmp(cmd, "timer", 5) == 0 && cmd[5] >= '1' && cmd[5] <= '5'
               && strncmp(cmd + 6, "_ms ", 4) == 0) {
        int tn = cmd[5] - '0';
        uint32_t ms;

        if (parse_u32_dec(cmd + 10, &ms) != 0) {
            uart_puts(u, "ERR timerN_ms: need integer N\r\n");
        } else {
            switch (tn) {
            case 1:
                timerN_sleep_ms(u, 1, ms, "ERR timer1_ms", "OK timer1_ms\r\n",
                                &T1CON, &TMR1, &PR1, T1_PRESCALE_256,
                                _IFS0_T1IF_MASK);
                break;
            case 2:
                timerN_sleep_ms(u, 2, ms, "ERR timer2_ms", "OK timer2_ms\r\n",
                                &T2CON, &TMR2, &PR2, TB_PRESCALE_256,
                                _IFS0_T2IF_MASK);
                break;
            case 3:
                timerN_sleep_ms(u, 3, ms, "ERR timer3_ms", "OK timer3_ms\r\n",
                                &T3CON, &TMR3, &PR3, TB_PRESCALE_256,
                                _IFS0_T3IF_MASK);
                break;
            case 4:
                timerN_sleep_ms(u, 4, ms, "ERR timer4_ms", "OK timer4_ms\r\n",
                                &T4CON, &TMR4, &PR4, TB_PRESCALE_256,
                                _IFS0_T4IF_MASK);
                break;
            default:
                timerN_sleep_ms(u, 5, ms, "ERR timer5_ms", "OK timer5_ms\r\n",
                                &T5CON, &TMR5, &PR5, TB_PRESCALE_256,
                                _IFS0_T5IF_MASK);
                break;
            }
        }
    } else if (strcmp(cmd, "adc all") == 0) {
        adc_all(u);
    } else if (strcmp(cmd, "ocmp all") == 0) {
        ocmp_all(u);
    } else if (strcmp(cmd, "pmp") == 0) {
        pmp_test(u);
    } else if (strcmp(cmd, "misc phys") == 0) {
        misc_phys(u);
    } else if (strcmp(cmd, "isr flags") == 0) {
        isr_flags_all(u);
    } else if (strcmp(cmd, "isr ct") == 0) {
        isr_ct(u);
    } else if (strcmp(cmd, "i2c all") == 0) {
        i2c_all(u);
    } else if (strcmp(cmd, "nvm all") == 0) {
        nvm_all(u);
    } else if (strcmp(cmd, "uart3 isr") == 0) {
        uart3_isr_lpback_cmd(u);
    } else if (strncmp(cmd, "echo ", 5) == 0) {
        uart_puts(u, cmd + 5);
        uart_puts(u, "\r\nOK echo\r\n");
    } else {
        uart_puts(u, "Unknown. Type help\r\n");
    }
}

static void poll_uart(unsigned u)
{
    int c;

    if (!uart_getc_ready(u)) {
        return;
    }
    c = uart_getc(u);
    if (c == '\r' || c == '\n') {
        line[u][pos[u]] = '\0';
        pos[u] = 0;
        uart_puts(u, "\r\n");
        run_cmd(u, line[u]);
        uart_puts(u, "\r\nU");
        uart_putc(u, (char)('1' + (int)u));
        uart_puts(u, "> ");
        return;
    }
    if (c == 127 || c == 8) { /* backspace */
        if (pos[u] > 0) {
            pos[u]--;
            uart_puts(u, "\b \b");
        }
        return;
    }
    if (c >= 32 && c < 127 && pos[u] < LINE_MAX) {
        line[u][pos[u]++] = (char)c;
        uart_putc(u, (char)c);
    }
}

int main(void)
{
    unsigned u;

    INTCONbits.MVEC = 1;

    for (u = 0; u < NUM_UARTS; u++) {
        uart_hw_init(u);
        pos[u] = 0;
        line[u][0] = '\0';
    }

    uart_puts(0, "\r\n*** PIC32MX350 UART CLI (U1-U3; no UART4 on MX350) ***\r\n");
    /* U2/U3: same CLI; no boot spam (keeps single-host-serial logs readable). */

    print_help(0);
    uart_puts(0, "\r\nU1> ");
    uart_puts(1, "\r\nU2> ");
    uart_puts(2, "\r\nU3> ");

    for (;;) {
        poll_uart(0);
        poll_uart(1);
        poll_uart(2);
    }
}
