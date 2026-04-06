# PIC32MX350F256H — grbl_bridge_test (plotter-bridge exerciser for QEMU + visualizer)
XC32DIR ?= /opt/microchip/xc32/v4.60
REPO_ROOT ?= ../..
CC      := $(XC32DIR)/bin/xc32-gcc
BIN2HEX := $(XC32DIR)/bin/xc32-bin2hex

PART    := 32MX350F256H
TARGET  := grbl-bridge-test-mx350.elf
HEX     := grbl-bridge-test-mx350.hex

CFLAGS  := -mprocessor=$(PART) -O2 -Wall -std=c99 -ffunction-sections
LDFLAGS := -Wl,-Map=grbl-bridge-test-mx350.map

.PHONY: all clean qemu

all: $(HEX)

qemu:
	$(MAKE) -C $(REPO_ROOT)/mipsel-softmmu qemu-system-mipsel

$(TARGET): grbl_bridge_test.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ grbl_bridge_test.c

$(HEX): $(TARGET)
	$(BIN2HEX) $<

clean:
	rm -f $(TARGET) $(HEX) grbl-bridge-test-mx350.map *.o
