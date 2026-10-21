MCU := atmega328p
F_CPU := 16000000UL
CC := avr-gcc
OBJCOPY := avr-objcopy
SIZE := avr-size
BUILD ?= build
DEBUG_ALLOW_PC_AUTO_WITHOUT_ASMS ?= 0
PC_IP_LAST_OCTET ?= 1
CFLAGS := -mmcu=$(MCU) -DF_CPU=$(F_CPU) -DDEBUG_ALLOW_PC_AUTO_WITHOUT_ASMS=$(DEBUG_ALLOW_PC_AUTO_WITHOUT_ASMS) -DBAKU_PC_IP_LAST_OCTET=$(PC_IP_LAST_OCTET) -std=gnu11 -Os -Wall -Wextra -Werror -ffunction-sections -fdata-sections -fstack-usage
LDFLAGS := -mmcu=$(MCU) -Wl,--gc-sections,-Map,$(BUILD)/motor_controller.map
SOURCES := main.c board.c w5500.c mcp2515.c
OBJECTS := $(SOURCES:%.c=$(BUILD)/%.o)
.PHONY: all size clean
all: $(BUILD)/motor_controller.hex size
$(BUILD):
	mkdir -p $@
$(BUILD)/%.o: %.c hal.h spi_bus.h | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@
$(BUILD)/motor_controller.elf: $(OBJECTS)
	$(CC) $(LDFLAGS) $^ -o $@
$(BUILD)/motor_controller.hex: $(BUILD)/motor_controller.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@
size: $(BUILD)/motor_controller.elf
	$(SIZE) -C --mcu=$(MCU) $<
clean:
	rm -rf $(BUILD)
