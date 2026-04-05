# Simulator for pic32

In this repository you will find a fork of the QEMU simulator for Microchip PIC32 processors.
The following PIC32 microcontrollers and boards are supported:

Machine selector    | Microcontroller and board
--------------------|------------------------------------
pic32mx3-generic    | Bare PIC32MX350F256H (no specific board wiring)
pic32mx7-explorer16 | PIC32MX7 on Microchip Explorer-16 board
pic32mx7-max32      | PIC32MX7 on chipKIT Max32 board
pic32mx7-maximite   | PIC32MX7 on Geoff's Maximite board
pic32mz-explorer16  | PIC32MZ on Microchip Explorer-16 board
pic32mz-meb2        | PIC32MZ on Microchip MEB-II board
pic32mz-wifire      | PIC32MZ on chipKIT WiFire board

## PIC32MX350F256H bare metal (`pic32mx3-generic`)

Build a little-endian MIPS softMMU QEMU binary (same as in [Building](#building) below), then run the `pic32mx3-generic` machine. It models a **PIC32MX350F256H**-class core and peripherals for bare-metal firmware (Intel HEX or ELF via `-kernel`).

### Serial port mapping

Host `-serial` backends are attached to guest UARTs in this **fixed order** (independent of pin remapping in the guest):

| Host `-serial` index | Guest UART | Typical use |
|---------------------|------------|-------------|
| #0 | UART3 | First `-serial` in command line |
| #1 | UART1 | Often used as console |
| #2 | UART2 | Optional second channel |

Example: UART1 on the terminal, no monitor:

```text
qemu-system-mipsel -M pic32mx3-generic -nographic -monitor none \
  -serial null -serial stdio -serial null \
  -kernel firmware.hex
```

The first `-serial` corresponds to UART3, the second to UART1, the third to UART2. Use `-serial null` for UARTs you do not need.

### Optional features

- **Instruction counting / pacing:** `-icount shift=auto` (or `NO_ICOUNT=1` in the test scripts for a fast run). With icount enabled, guest timers in this port are driven so that busy-waits still see time advance; see `tests/pic32mx350-uart-cli/README.txt`.
- **Flash images:** program and boot flash files can be passed with machine globals (see comments in `hw/mips/mips_pic32mx3.c` near `prog-flash` / `boot-flash`).

### CLI regression tests

The directory `tests/pic32mx350-uart-cli/` contains a small XC32 firmware and **expect** scripts that drive UARTs, timers, ADC, and output-compare stubs over QEMU.

1. **Toolchain:** Microchip XC32 (e.g. v4.60) on `PATH`, for example:

   ```text
   export PATH="/opt/microchip/xc32/v4.60/bin:$PATH"
   make -C tests/pic32mx350-uart-cli
   ```

2. **QEMU:** Build `qemu-system-mipsel` from this tree after any change under `hw/mips/` or related PIC32 code:

   ```text
   make -C mipsel-softmmu qemu-system-mipsel
   ```

   Optionally copy the binary (many scripts default to `build-pic32-qemu/qemu-system-mipsel`):

   ```text
   bash scripts/build-pic32-qemu.sh
   ```

3. **Run tests** (first argument = path to your `qemu-system-mipsel`):

   ```text
   /usr/bin/expect -f tests/pic32mx350-uart-cli/run-timer-ms-test.expect /path/to/qemu-system-mipsel
   /usr/bin/expect -f tests/pic32mx350-uart-cli/run-cli-test.expect /path/to/qemu-system-mipsel
   ```

   Optional second argument: path to `uart-cli-mx350.hex`. Environment variables such as `NO_ICOUNT=1`, `ICOUNT_SHIFT`, and `PIC32_UART_TCP_BASE` are documented in `tests/pic32mx350-uart-cli/README.txt`.

For the mainstream documentation on QEMU see [README.rst](https://github.com/qemu/qemu/blob/master/README.rst)
or visit [wiki.qemu-project.org](http://wiki.qemu-project.org).

# Building

The build is performed in four steps:

 * Install dependencies
 * Configure QEMU
 * Compile
 * Install

## Install dependencies

On Linux:

    sudo apt install libpixman-1-dev libfdt-dev zlib1g-dev libglib2.0-dev libsdl1.2-dev readline-dev libssl-dev
    curl -fsSL https://pyenv.run | bash
    ~/.pyenv/bin/pyenv install 2.7

On MacOS:

    brew install pyenv
    ~/.pyenv/bin/pyenv install 2.7

## Configure QEMU

    git clone git@github.com:sergev/qemu.git
    cd qemu
    ./configure --target-list=mipsel-softmmu --python=$HOME/.pyenv/versions/2.7.18/bin/python2 \
        --disable-werror --disable-opengl --disable-libnfs

## Compile

    make

## Install

Copy the resulting binary to a directory of your choice, with name `qemu-pic32`.
For example:

    cp mipsel-softmmu/qemu-system-mipsel ~/.local/bin/qemu-pic32

# Examples

 * [Run 'Hello World' demo on Max32 board](https://github.com/sergev/qemu/wiki/Max32-Hello-World)
 * [Run RetroBSD on Max32 board](https://github.com/sergev/qemu/wiki/RetroBSD-Example)
 * [Run LiteBSD on WiFire board](https://github.com/sergev/qemu/wiki/LiteBSD-Example)
 * [Full instruction trace of 'Hello World' demo](https://github.com/sergev/qemu/wiki/Example-of-instruction-trace)
