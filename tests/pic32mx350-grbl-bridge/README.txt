PIC32MX350 grbl_bridge test (plotter-bridge + qemu_grbl_plugin visualizer)
==========================================================================

Firmware exercises the QEMU **plotter-bridge** device using the same GPIO map as
`contrib/qemu-grbl-plugin/plotter_bridge_config.example.json`: drives motor MMIO
and vgpio IN1/IN2 pins, full PWM, toggles **sensors** + **LEDs** (SHM bytes the
visualizer shows), then runs forward motion so **encoder** simulation and
**pulse** counts update (when not stalled at a limit).

Build (XC32 v4.x):

  export PATH="/opt/microchip/xc32/v4.60/bin:$PATH"
  make

Outputs: `grbl-bridge-test-mx350.elf`, `grbl-bridge-test-mx350.hex`

Run QEMU (from repo root; uses env vars wired in `mips_pic32mx3.c`):

  truncate -s 28 /tmp/plotter.shm
  export PLOTTER_BRIDGE_SHM=/tmp/plotter.shm
  export PLOTTER_BRIDGE_CONFIG="$PWD/contrib/qemu-grbl-plugin/plotter_bridge_config.example.json"

  ./mipsel-softmmu/qemu-system-mipsel -M pic32mx3-generic \
    -kernel tests/pic32mx350-grbl-bridge/grbl-bridge-test-mx350.hex \
    -nographic -monitor none -serial null -serial stdio -serial null \
    -icount shift=auto

In another terminal, visualizer (venv — see `contrib/qemu-grbl-plugin/MANUAL.md`):

  cd contrib/qemu-grbl-plugin/visualizer
  .venv/bin/python plotter_visualizer.py /tmp/plotter.shm \
    -c ../plotter_bridge_config.example.json

Expect script (smoke):

  ./run-grbl-bridge-test.expect [path/to/qemu-system-mipsel] [path/to.hex]

Success lines: `OK grbl_bridge:` or informative `WARN` if pulses did not move
(e.g. random reset at max travel). Sensor/LED lines print before motion so the UI
can show red/white and sensor buttons state.
