PIC32MX350F256H multi-UART CLI + Timer3 ~10 s test
===================================================

Build (XC32 v4.60 under /opt/microchip/xc32/v4.60):

  export PATH="/opt/microchip/xc32/v4.60/bin:$PATH"
  make

Outputs: uart-cli-mx350.elf, uart-cli-mx350.hex

Rebuilding QEMU when needed
-----------------------------
Any time you pull or change PIC32MX3 (or shared PIC32) *hardware models* under
this repo — especially hw/mips/*.c (UART, Timers, ADC, GPIO, etc.) — rebuild
qemu-system-mipsel here and run tests against *that* binary.  A distro-installed
QEMU does not include your tree’s behavior; stale builds cause failures such as
"adc all" timing out in simulation or mismatches with run-cli-test.expect.

After such changes, rebuild before re-running the expect script, for example:

  make -C mipsel-softmmu qemu-system-mipsel

To refresh the copy under build-pic32-qemu/ (used by some docs and expect defaults):

  bash scripts/build-pic32-qemu.sh

That script copies the linked binary to build-pic32-qemu/qemu-system-mipsel.
You can also copy manually: cp mipsel-softmmu/qemu-system-mipsel build-pic32-qemu/

Pass the resulting qemu-system-mipsel as the first argument to run-cli-test.expect
(or set the path inside the script).  Rebuild again whenever hw/mips or related
PIC32 code changes — do not assume an older binary is still valid.

After editing QEMU PIC32 models, rebuild in the same working tree before relying
on simulation results — stale binaries cause confusing test failures.

QEMU: host -serial order is UART3, UART1, UART2 (pic32mx3-generic).

  qemu-system-mipsel -M pic32mx3-generic -nographic -monitor none \
    -serial tcp:127.0.0.1:7710,server,nowait \
    -serial tcp:127.0.0.1:7711,server,nowait \
    -serial tcp:127.0.0.1:7712,server,nowait \
    -kernel uart-cli-mx350.hex

  TCP 7710 = guest UART3, 7711 = UART1, 7712 = UART2.

Commands (per UART): help | uart | switch N (N=1–3) | timer | timer_ms N |
  timmer_ms N | timer1_ms N … timer5_ms N | adc all | ocmp all | echo <text>

"adc all" needs a QEMU built from this tree (ADC1 model in mips_pic32mx3.c);
"ocmp all" needs the OC interrupt stub on T2/T3 wraps in mips_pic32mx3.c.
Rebuild as above after any hw/mips PIC32 change.

The part has three UARTs (U1–U3); there is no UART4.  On a single host serial you
are fixed to one guest UART; use multiple -serial backends (e.g. TCP ports above)
to “switch” by opening a different connection.  run-cli-test.expect exercises
stdio on U1, then connects to 7711/7712/7710 for U1/U2/U3.  Set PIC32_UART_TCP_BASE
if the default 7710..7712 ports are busy.

icount (optional wall-clock pacing)
------------------------------------
With -icount, QEMU_CLOCK_VIRTUAL follows the instruction counter, so a timer
scheduled on that clock would advance only with executed instructions and a
tight TxIF poll would not see wall-time progress.  pic32mx3-generic therefore
runs Timers T1–T5 period match on QEMU_CLOCK_VIRTUAL_RT whenever icount is
enabled (same as the prior Timer3-only behavior).  The TCG thread also runs
QEMU_CLOCK_VIRTUAL_RT timers after each icount batch (see cpus.c); otherwise a
busy-wait would starve the iothread and those timers would never fire.  Without
icount, these timers use the normal virtual clock.  Omit -icount for the fastest
runs (10 s guest time in a fraction of a second wall).
