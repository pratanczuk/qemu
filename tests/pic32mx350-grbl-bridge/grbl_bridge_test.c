/*
 * PIC32MX350F256H — minimal plotter-bridge (grbl_bridge) exercise for QEMU.
 *
 * Drives motor/PWM MMIO, toggles sensors + LEDs, uses vgpio for mapped motor
 * pins from contrib/qemu-grbl-plugin/plotter_bridge_config.example.json, and
 * prints pulse / encoder / vgpio_out state on UART1 for host + visualizer check.
 *
 * QEMU: PLOTTER_BRIDGE_SHM, PLOTTER_BRIDGE_CONFIG pointing at that JSON.
 * GPIO MMIO physical base 0x1F400040 → KSEG1 0xBF400040.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <stdint.h>
#include <xc.h>

#define PBCLK_HZ        80000000u
#define UART_BAUD       115200u

/* plotter-bridge GPIO region (second SysBus region); see MANUAL.md */
#define PLOTTER_GPIO_K1  0xBF400040u

#define OFF_MOTOR   0x00u
#define OFF_ENC     0x04u
#define OFF_SENSLED 0x08u
#define OFF_PULSE_X 0x10u
#define OFF_PULSE_Y 0x14u
#define OFF_PULSE_Z 0x18u
#define OFF_VGPIO_IN  0x1cu
#define OFF_VGPIO_OUT 0x20u
#define OFF_PWM     0x24u

/* plotter_bridge_config.example.json gpio_map (virtual pin = RBn→n, RDn→16+n) */
#define PIN_X_IN1   21u
#define PIN_X_IN2   24u
#define PIN_Y_IN1   16u
#define PIN_Y_IN2   17u
#define PIN_Z_IN1   5u

#define PLOTTER_LINE_A  0x01u
#define PLOTTER_LINE_B  0x02u

#define SENSOR_MECH0    0x01u
#define SENSOR_MECH1    0x02u
#define SENSOR_OPTICAL  0x04u
#define LED_RED         0x01u
#define LED_WHITE       0x02u

#define TB_PRESCALE_256  (7u << 4)
#define T3IFS_MASK      _IFS0_T3IF_MASK

static volatile uint32_t *const plotter_reg(unsigned off)
{
    return (volatile uint32_t *)(PLOTTER_GPIO_K1 + off);
}

static void uart1_putc(char c)
{
    U1TXREG = (uint32_t)(unsigned char)c;
}

static void uart1_puts(const char *s)
{
    while (*s) {
        uart1_putc(*s++);
    }
}

static void uart1_put_hex32(uint32_t v)
{
    static const char xd[] = "0123456789abcdef";
    int i;

    uart1_puts("0x");
    for (i = 28; i >= 0; i -= 4) {
        uart1_putc(xd[(v >> (unsigned)i) & 0xFu]);
    }
}

static void uart1_hw_init(void)
{
    U1MODE = 0;
    U1STA = 0;
    U1BRG = (PBCLK_HZ / (16u * UART_BAUD)) - 1u;
    U1MODE = 0x8000u;
    U1STA = (1u << 10) | (1u << 12);
}

/*
 * Timer3 delay (~ms) using PBCLK/256 — same math as uart-cli timerN_sleep_ms.
 */
static void delay_ms(uint32_t ms)
{
    uint64_t total;
    uint64_t rem;
    int first;

    total = ((uint64_t)ms * 312500u + 999u) / 1000u;
    if (total == 0u) {
        total = 1u;
    }
    T3CON = 0;
    TMR3 = 0;
    IFS0CLR = T3IFS_MASK;
    rem = total;
    first = 1;
    while (rem > 0u) {
        uint32_t chunk = (rem > 65536u) ? 65536u : (uint32_t)rem;

        PR3 = chunk - 1u;
        IFS0CLR = T3IFS_MASK;
        if (first) {
            T3CON = TB_PRESCALE_256 | 0x8000u;
            first = 0;
        }
        while ((IFS0 & T3IFS_MASK) == 0u) {
        }
        IFS0CLR = T3IFS_MASK;
        rem -= (uint64_t)chunk;
    }
    T3CONCLR = 0x8000u;
}

static uint32_t rd32(unsigned off)
{
    return *plotter_reg(off);
}

static void wr32(unsigned off, uint32_t v)
{
    *plotter_reg(off) = v;
}

int main(void)
{
    uint32_t px0, py0, pz0;
    uint32_t px1, py1, pz1;
    uint32_t enc0, enc1, vout0, vout1;
    uint32_t m;

    INTCONbits.MVEC = 1;
    uart1_hw_init();

    uart1_puts("\r\n*** grbl_bridge_test (PIC32MX350 + QEMU plotter-bridge) ***\r\n");

    /* --- Sensors + LEDs (host visualizer reads SHM) --- */
    wr32(OFF_SENSLED, ((uint32_t)LED_RED << 8) | (uint32_t)SENSOR_MECH0);
    uart1_puts("SENS/LED: mech0 + RED\r\n");
    delay_ms(5u);
    wr32(OFF_SENSLED, ((uint32_t)LED_WHITE << 8) | (uint32_t)(SENSOR_MECH0 | SENSOR_MECH1 | SENSOR_OPTICAL));
    uart1_puts("SENS/LED: all sensors + WHITE\r\n");
    delay_ms(5u);
    wr32(OFF_SENSLED, 0u);
    uart1_puts("SENS/LED: cleared\r\n");

    /* Baseline */
    px0 = rd32(OFF_PULSE_X);
    py0 = rd32(OFF_PULSE_Y);
    pz0 = rd32(OFF_PULSE_Z);
    enc0 = rd32(OFF_ENC);
    vout0 = rd32(OFF_VGPIO_OUT);

    uart1_puts("before: px=");
    uart1_put_hex32(px0);
    uart1_puts(" py=");
    uart1_put_hex32(py0);
    uart1_puts(" pz=");
    uart1_put_hex32(pz0);
    uart1_puts(" enc=");
    uart1_put_hex32(enc0);
    uart1_puts(" vgpio_out=");
    uart1_put_hex32(vout0);
    uart1_puts("\r\n");

    /* Full PWM + forward on all axes (motor MMIO): byte per axis = LINE_A */
    wr32(OFF_PWM, 0x00ffffffu);
    m = (uint32_t)PLOTTER_LINE_A | ((uint32_t)PLOTTER_LINE_A << 8)
        | ((uint32_t)PLOTTER_LINE_A << 16);
    wr32(OFF_MOTOR, m);

    uart1_puts("MOTOR MMIO forward all + PWM max; delay 120ms...\r\n");
    delay_ms(120u);

    px1 = rd32(OFF_PULSE_X);
    py1 = rd32(OFF_PULSE_Y);
    pz1 = rd32(OFF_PULSE_Z);
    enc1 = rd32(OFF_ENC);
    vout1 = rd32(OFF_VGPIO_OUT);

    uart1_puts("after:  px=");
    uart1_put_hex32(px1);
    uart1_puts(" py=");
    uart1_put_hex32(py1);
    uart1_puts(" pz=");
    uart1_put_hex32(pz1);
    uart1_puts(" enc=");
    uart1_put_hex32(enc1);
    uart1_puts(" vgpio_out=");
    uart1_put_hex32(vout1);
    uart1_puts("\r\n");

    /* Stop */
    wr32(OFF_MOTOR, 0u);

    /* --- vgpio motor path (matches JSON pins): IN1 only = forward --- */
    wr32(OFF_VGPIO_IN, 0u);
    wr32(OFF_PWM, 0x00ffffffu);
    /* X forward: pin IN1 only */
    wr32(OFF_VGPIO_IN, (1u << PIN_X_IN1));
    delay_ms(40u);
    uart1_puts("vgpio X fwd only: motor=");
    uart1_put_hex32(rd32(OFF_MOTOR));
    uart1_puts(" vout=");
    uart1_put_hex32(rd32(OFF_VGPIO_OUT));
    uart1_puts("\r\n");

    wr32(OFF_VGPIO_IN, (1u << PIN_Y_IN1));
    delay_ms(40u);
    uart1_puts("vgpio Y fwd only: motor=");
    uart1_put_hex32(rd32(OFF_MOTOR));
    uart1_puts("\r\n");

    /* Z: JSON has in2=255 — vgpio motor sync skips this axis (see plotter_bridge.c). */
    wr32(OFF_VGPIO_IN, (1u << PIN_Z_IN1));
    delay_ms(40u);
    uart1_puts("vgpio Z in1 only (in2 n/c in JSON): motor=");
    uart1_put_hex32(rd32(OFF_MOTOR));
    uart1_puts("\r\n");

    wr32(OFF_VGPIO_IN, 0u);
    wr32(OFF_PWM, 0u);

    /* Pass/fail heuristic: pulses or encoder or vgpio_out should change after motion */
    if (px1 != px0 || py1 != py0 || pz1 != pz0 || enc1 != enc0 || vout1 != vout0) {
        uart1_puts("OK grbl_bridge: motion or enc/vgpio activity observed\r\n");
    } else {
        uart1_puts("WARN grbl_bridge: no delta (stall at limit? try larger delay or max_pulses=0 JSON)\r\n");
    }

    uart1_puts("done.\r\n");
    for (;;) {
    }
    return 0;
}
