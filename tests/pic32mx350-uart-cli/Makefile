# PIC32MX350F256H — XC32 v4.x (default: /opt/microchip/xc32/v4.60)
#
# This Makefile only builds uart-cli-mx350.hex. It does NOT build QEMU.
# After changes under hw/mips/ (or other PIC32 QEMU sources), rebuild the
# emulator explicitly, e.g. from repo root:
#   make -C mipsel-softmmu qemu-system-mipsel
# or: bash scripts/build-pic32-qemu.sh
XC32DIR ?= /opt/microchip/xc32/v4.60
REPO_ROOT ?= ../..
CC      := $(XC32DIR)/bin/xc32-gcc
BIN2HEX := $(XC32DIR)/bin/xc32-bin2hex

PART    := 32MX350F256H
TARGET  := uart-cli-mx350.elf
HEX     := uart-cli-mx350.hex

# grblHAL-style UART3 ISR + TX ring + LPBACK (see PROPOSAL-grblhal-uart3-isr-test.md)
UART3_ISR_ELF  := uart3-isr-mx350.elf
UART3_ISR_HEX  := uart3-isr-mx350.hex

CFLAGS  := -mprocessor=$(PART) -O2 -Wall -std=c99 -ffunction-sections
LDFLAGS := -Wl,-Map=uart-cli-mx350.map

.PHONY: all clean qemu uart3-isr

all: $(HEX)

uart3-isr: $(UART3_ISR_HEX)

# Convenience: link qemu-system-mipsel from this tree (needed after hw/mips edits).
qemu:
	$(MAKE) -C $(REPO_ROOT)/mipsel-softmmu qemu-system-mipsel

$(TARGET): main.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ main.c

$(HEX): $(TARGET)
	$(BIN2HEX) $<

$(UART3_ISR_ELF): uart3_isr_lpback_main.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ uart3_isr_lpback_main.c

$(UART3_ISR_HEX): $(UART3_ISR_ELF)
	$(BIN2HEX) $<

clean:
	rm -f $(TARGET) $(HEX) uart-cli-mx350.map *.o \
		$(UART3_ISR_ELF) $(UART3_ISR_HEX)
