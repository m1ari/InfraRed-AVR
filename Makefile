# MCU Selection
#MCU = atmega32u2
#MCU = atmega168a
#MCU = atmega164p
MCU = atmega328p

# Clock Speed
CLOCK = 8000000UL


## Objects that must be built in order to link

CC = avr-gcc
CPP = avr-g++


## Compile options common for all C compilation units.
CFLAGS = -Wall -gdwarf-2 -Os
#CFLAGS = -g	#Debug
#CFLAGS += -Os	# Optimize size
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d # Dependancy files
CFLAGS += -Iukhasnet-rfm69

## Linker flags
LDFLAGS = 

## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings




## Build
.PHONY: all
all:
	@echo "Use make PROJECT where project is one of:"
	@echo "  ir-sender"
	@echo "These will build and program the image"
	@echo "Add .hex to build the image to flash"

## Compile
%.o: %.c
	$(CC) -mmcu=$(MCU) -DF_CPU=$(CLOCK) $(CFLAGS) -c  $<

.PHONY: ir-sender
ir-sender: ir-sender.elf ir-sender.hex ir-sender.eep
	sudo avrdude -c avrispmkii -p $(MCU) -P usb -U flash:w:$@.hex
	avr-size -C --mcu=${MCU} ${@}.elf

.PHONY: ir-sender-asp
ir-sender-asp: ir-sender.elf ir-sender.hex ir-sender.eep
	sudo avrdude -c usbasp -p $(MCU) -P usb -U flash:w:ir-sender.hex
	avr-size -C --mcu=${MCU} ir-sender.elf

.PHONY: ir-receiver
ir-receiver: ir-receiver.elf ir-receiver.hex ir-receiver.eep
	sudo avrdude -c avrispmkii -p $(MCU) -P usb -U flash:w:$@.hex



##Link
%.elf: %.o
	 $(CC) -mmcu=$(MCU)  $(LDFLAGS) $^ -o $@

%.hex: %.elf
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: %.elf
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: %.elf
	avr-objdump -h -S $< > $@

#size: ${TARGET}
	#avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
#$(shell rm {ukhasnet-rf69,rfm-test}.{eep,elf,o,hex} dep/*)
.PHONY: clean
clean:
	-rm *.o
	-rm *.elf
	
.PHONY: fuses-328p
fuses-328p:
	sudo avrdude -c avrispmkii -p atmega328p -B 8 -P usb  -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0x07:m

# 328p Fuses
# (L:62, H:D9, E:07) - Default
# (L:E2, H:D9, E:07) - Remove CKDIV8
# (L:E2, H:DA, E:05) - Bootloader,   BoD@2.7
# http://www.engbedded.com/fusecalc/
# https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/boards.txt
# https://github.com/arduino/Arduino/tree/master/hardware/arduino/avr/bootloaders/atmega

.PHONY: boot-328p
boot-328p:
	sudo avrdude -c avrispmkii -p atmega328p -B 8 -P usb -U flash:w:ATmegaBOOT_168_atmega328_pro_8MHz.hex -U lfuse:w:0xe2:m -U hfuse:w:0xda:m -U efuse:w:0x05:m

## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

