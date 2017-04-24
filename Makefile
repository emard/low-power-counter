all: bin

SRCS=counter.c
PROJECT=counter

# programmer connection port
#AVRTTY=/dev/ttyS0
#AVRTTY=/dev/ttyUSB0
#AVRTTY=usbasp
AVRTTY=avrdoper
# smd led board
# AVRTTY=/dev/rfcomm25
# vuvuzelle
#AVRTTY=/dev/rfcomm26
# hjuston
# AVRTTY=/dev/rfcomm27
#AVRTTY=/tmp/COM1

# programmer type
#AVRPROG=usbasp
#AVRPROG=stk500
AVRPROG=stk500hvsp
#AVRPROG=dasa

AVRDUDE_MCU=$(MCU)
#AVRDUDE_OVERRIDE=-F
AVRDUDE_BAUD=-b 115200

# mcu model
# MCU=atmega162
# MCU=atmega64m1
# MCU=at90pwm316
MCU=attiny85

# CPU frequency in Hz
F_CPU = 1000000
# F_CPU = 16110000
F_PLL = 64000000

# normal application start at 0x0000
#  8k bootloader at 0x1800
# 16k bootloader at 0x3800
# 32k bootloader at 0x7800
START_ADDRESS=0x0

DEFINES = -DF_CPU=$(F_CPU) -DF_PLL=$(F_PLL) -DSTART_ADDRESS=$(START_ADDRESS)

# attention: writing wrong fuses or locks can prevent further programming of the device
# attention: different MCU models have different fuses and locks!
#     make fuse    # to set the clock generator, boot section size etc.
#     make lock    # to protect the boot loader from overwriting
# see page 231-232 for atmega162
# >8MHz ext osc, reset at 3800 lfuse:jtag,bootloader, efuse:brownout <2.9V, disable JTAG
# execucte bootloader, no jtag, brownout
#FUSEOPT_162 = -U hfuse:w:0xd8:m -U lfuse:w:0xee:m -U efuse:w:0xfb:m
# don't execute bootloader, no jtag, brownout (normal)
#FUSEOPT_162 = -U hfuse:w:0xd8:m -U lfuse:w:0xef:m -U efuse:w:0xfb:m
#FUSEOPT_32 = -U hfuse:w:0x82:m -U lfuse:w:0x9f:m
# RC oscillator (CPU will run at 16MHz, PLL at 64MHz)
FUSEOPT_316_RC = -U hfuse:w:0xd3:m -U lfuse:w:0xf3:m -U efuse:w:0xfa:m
# Crystal 8MHz  (CPU will run at 16MHz, PLL at 64MHz)
FUSEOPT_316_XTAL = -U hfuse:w:0xd3:m -U lfuse:w:0xf5:m -U efuse:w:0xfa:m
# Fuse for ATTINY85 internal RC oscillator 1MHz
FUSEOPT_T85_RC = -U hfuse:w:0xdf:m -U lfuse:w:0x62:m -U efuse:w:0xff:m
#FUSEOPT = -U hfuse:w:high.txt -U lfuse:w:low.txt
FUSEOPT=$(FUSEOPT_T85_RC)


READFUSEOPT = -U hfuse:r:high.txt -U lfuse:r:low.txt

# protect bootloader from overwriting (p230-231) (chip erase will reset this lock)
#LOCKOPT_BOOTLD = -U lock:w:0x2f:m
# protect flash and eeprom from programming and reading (p230-231) (chip erase will reset this lock)
LOCKOPT = -U lock:w:lock.txt
LOCKOPT = $(LOCKOPT_BOOTLD)

AVRDUDE_TERMINAL=
#AVRDUDE_TERMINAL=-t
#AVRDUDE_TERMINAL=-t -vvvv
AVRDUDE_COUNTER=
#AVRDUDE_COUNTER=-y
AVRDUDEPAR=-p $(AVRDUDE_MCU) -P $(AVRTTY) $(AVRDUDE_BAUD) $(AVRDUDE_OVERRIDE) -c $(AVRPROG) -u $(AVRDUDE_TERMINAL) $(AVRDUDE_COUNTER) 

OBJCOPY_FLASH_FLAGS = -R .eeprom
OBJCOPY_EEPROM_FLAGS = -j .eeprom
OBJCOPY_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
OBJCOPY_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings

# Intel HEX flash files
OBJCOPY_FORMAT=ihex
AVRDUDE_FORMAT=i

# Motorola S-Records flash files
#OBJCOPY_FORMAT=srec
#AVRDUDE_FORMAT=s

MAKEDEPEND=avr-gcc -M $(CPPFLAGS) $(DEFINES) -o $(PROJECT).d $(SRCS)
 
UPLOAD_COUNTER=~/src/avr/.upload_counter

OPTIMIZE=-Os -std=gnu99 -fpack-struct -fshort-enums -ffreestanding \
  -flto -fno-inline-small-functions -fno-split-wide-types \
  -fno-tree-scev-cprop -fwhole-program -fno-tree-scev-cprop \
  -mcall-prologues -Wl,--relax  -Wl,--section-start=.text=$(START_ADDRESS)
# 
# there some pointer casting in ds1wire.c that violate strict aliasing
#-funsigned-char -funsigned-bitfields \

# Minimalistic printf version
PRINTF_LIB_MIN = -Wl,-u,vfprintf -lprintf_min
# Floating point printf version (requires MATH_LIB = -lm below)
PRINTF_LIB_FLOAT = -Wl,-u,vfprintf -lprintf_flt

# If this is left blank, then it will use the Standard printf version.
#PRINTF_LIB = $(PRINTF_LIB_MIN)
#PRINTF_LIB = $(PRINTF_LIB_FLOAT)

ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs

# binary targets to build
BIN =	$(PROJECT).flash.$(OBJCOPY_FORMAT) $(PROJECT).eeprom.$(OBJCOPY_FORMAT)
#BIN =	$(PROJECT).flash.$(OBJCOPY_FORMAT) $(PROJECT).eeprom.$(OBJCOPY_FORMAT) ui

bin:	$(BIN)
upload: $(PROJECT).u
flash:  $(PROJECT).u
sflash:  $(PROJECT).su
bflash:  $(PROJECT).bu
eeprom: $(PROJECT).eu
verify: $(PROJECT).v

# generate dependencies using the compiler
$(PROJECT).d: $(SRCS)
	avr-gcc -mmcu=$(MCU) -M $(DEFINES) $(SRCS) | sed -e "s/^.*[.]o:/$(PROJECT).elf:/g" > $@
	avr-gcc -mmcu=$(MCU) -M $(DEFINES) $(SRCS) | sed -e "s/^.*[.]o:/$(PROJECT).s:/g" >> $@

# include dependency into the makefile
-include $(PROJECT).d

# compile into binary elf
$(PROJECT).elf: $(SRCS)
	avr-gcc -mmcu=$(MCU) -Wall $(OPTIMIZE) $(PRINTF_LIB) $(DEFINES) $(SRCS) -o $@
	avr-size $@
	@chmod -x $@ # remove executable flag

# make assembply output
$(PROJECT).s: $(SRCS)
	avr-gcc -mmcu=$(MCU) -Wall $(ASFLAGS)  $(OPTIMIZE) $(PRINTF_LIB) $(DEFINES) -S $(SRCS) -o $@

# convert binary elf to programmer file for upload
%.flash.$(OBJCOPY_FORMAT): %.elf
	avr-objcopy $(OBJCOPY_FLASH_FLAGS) -O $(OBJCOPY_FORMAT) $< $@
	avr-size $@
	@chmod -x $@

%.eeprom.$(OBJCOPY_FORMAT): %.elf
	avr-objcopy $(OBJCOPY_EEPROM_FLAGS) -O $(OBJCOPY_FORMAT) $< $@
	avr-size $@
	@chmod -x $@

# upload flash and eeprom files into the device
$(PROJECT).u: $(PROJECT).flash.$(OBJCOPY_FORMAT) $(PROJECT).eeprom.$(OBJCOPY_FORMAT)
	#expr `cat $(UPLOAD_COUNTER)` + 1 > $(UPLOAD_COUNTER)
	#cat < $(UPLOAD_COUNTER)
	# - killall cu
	# sleep 2
	avrdude $(AVRDUDEPAR) \
	  -U eeprom:w:$(PROJECT).eeprom.$(OBJCOPY_FORMAT):$(AVRDUDE_FORMAT) \
	  -U flash:w:$(PROJECT).flash.$(OBJCOPY_FORMAT):$(AVRDUDE_FORMAT)
	# echo -n -e "\\007"

$(PROJECT).us: $(PROJECT).flash.$(OBJCOPY_FORMAT) $(PROJECT).eeprom.$(OBJCOPY_FORMAT)

# blind upload flash and eeprom files into the device
$(PROJECT).bu: 
	#expr `cat $(UPLOAD_COUNTER)` + 1 > $(UPLOAD_COUNTER)
	#cat < $(UPLOAD_COUNTER)
	avrdude $(AVRDUDEPAR) \
	  -U flash:w:$(PROJECT).flash.$(OBJCOPY_FORMAT):$(AVRDUDE_FORMAT)
	# echo -n -e "\\007"

# ssbl bootloader upload
$(PROJECT).su: $(PROJECT).flash.$(OBJCOPY_FORMAT) $(PROJECT).eeprom.$(OBJCOPY_FORMAT)
	- killall cu
	sleep 2
	./ssbl.py $(AVRTTY) $(PROJECT).flash.$(OBJCOPY_FORMAT) $(PROJECT).eeprom.$(OBJCOPY_FORMAT)

# upload only eeprom files into the device
$(PROJECT).eu: $(PROJECT).eeprom.$(OBJCOPY_FORMAT)
	- killall cu
	sleep 2
	avrdude $(AVRDUDEPAR) \
	  -U eeprom:w:$(PROJECT).eeprom.$(OBJCOPY_FORMAT):$(AVRDUDE_FORMAT)
	# echo -n -e "\\007"

# read the eeprom	
eread:
	avrdude $(AVRDUDEPAR) \
	  -U eeprom:r:eeprom-read.hex:i

# verify flash and eeprom files in the device
$(PROJECT).v: $(PROJECT).flash.$(OBJCOPY_FORMAT) $(PROJECT).eeprom.$(OBJCOPY_FORMAT)
	avrdude $(AVRDUDEPAR) \
	  -U eeprom:v:$(PROJECT).eeprom.$(OBJCOPY_FORMAT):$(AVRDUDE_FORMAT) \
	  -U flash:v:$(PROJECT).flash.$(OBJCOPY_FORMAT):$(AVRDUDE_FORMAT)
	# echo -n -e "\\007"

ui:	ui.c unixavr.c
	gcc -o $@ ui.c unixavr.c

#%.w: %.m
#	expr `cat $(UPLOAD_COUNTER)` + 1 > $(UPLOAD_COUNTER)
#	cat < $(UPLOAD_COUNTER)
#	avrdude -u -p 8515 -P $(AVRTTY) -v -c stk500v2 -D -C /usr/local/etc/avrdude/avrdude.conf -U flash:w:$(PROJECT).m:s

#%.t: %.m
#	expr `cat $(UPLOAD_COUNTER)` + 1 > $(UPLOAD_COUNTER)
#	cat < $(UPLOAD_COUNTER)
#	avrdude -u -p 8515 -P $(AVRTTY) -v -c stk500v2 -D -C /usr/local/etc/avrdude/avrdude.conf -U flash:v:$(PROJECT).m:s

fuse:
	avrdude $(AVRDUDEPAR) $(FUSEOPT)

rfuse:
	avrdude $(AVRDUDEPAR) $(READFUSEOPT)

lock:
	avrdude $(AVRDUDEPAR) $(LOCKOPT)

run:
	stty speed 9600 raw -parenb < $(AVRTTY)
	cat $(AVRTTY)

clean:
	rm -f *.o *.s *.eeprom.$(OBJCOPY_FORMAT) *.flash.$(OBJCOPY_FORMAT) *.r *.elf *.d *.P DEADJOE octave-core $(BIN) *~ config/*~

#depend: $(SRCS)
#	makedepend $(INCLUDES) $^

# DO NOT DELETE THIS LINE -- make depend needs it
