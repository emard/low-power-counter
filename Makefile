all: counter.m

#AVRTTY=/dev/ttyS0
#AVRTTY=/dev/ttyUSB0
#AVRTTY=/dev/ttyUSB0
#AVRTTY=/dev/ttyACM0
AVRTTY=avrdoper

#MCU=at90s8515
#MCU=atmega8515
#MCU=atmega162
MCU=attiny84

FUSEOPT_T84 = -U hfuse:w:0xf0:m -U lfuse:w:0x60:m  -U efuse:w:0xfe:m
FUSEOPT=$(FUSEOPT_T84)
#FUSEOPT = -U hfuse:w:high.txt -U lfuse:w:low.txt
READFUSEOPT = -U hfuse:r:fuse-high.txt:h -U lfuse:r:fuse-low.txt:h -U efuse:r:fuse-extended.txt:h -U lock:r:fuse-lock.txt:h

# protect bootloader from overwriting (p230-231) (chip erase will reset this lock)
#LOCKOPT_BOOTLD = -U lock:w:0x2f:m
# protect flash and eeprom from programming and reading (p230-231) (chip erase will reset this lock)
LOCKOPT_BOOT_PROTECT = -U lock:w:0xfc:m
#LOCKOPT = -U lock:w:lock.txt
LOCKOPT = $(LOCKOPT_BOOT_PROTECT)

#$PRGPAR=-dprog=dasa -dlpt=$(AVRTTY) -dt_sck=0 -dt_wd_flash=3000 -dvoltage=3.0 -dt_reset=1500000
#PRGPAR=-dprog=dasa -dlpt=$(AVRTTY) -dt_sck=50 -dt_wd_flash=10000000 -dvoltage=3.0
#AVRDUDEPAR=-p $(MCU) -P $(AVRTTY) -c stk500v2 -y
#AVRDUDEPAR=-p $(MCU) -P $(AVRTTY) -c stk500 -y -u
AVRDUDEPAR=-p $(MCU) -P $(AVRTTY) -c stk500hvsp -y -u
#AVRDUDEPAR=-p $(MCU) -P $(AVRTTY) -c dasa -y -u

UPLOAD_COUNTER=~/src/avr/.upload_counter
#OPTIMIZE=-O6
#OPTIMIZE=-Os
#OPTIMIZE=-Os -fpack-struct -fshort-enums -ffreestanding --combine -fno-inline-small-functions -fno-split-wide-types -fno-tree-scev-cprop
#OPTIMIZE=-O0 -fpack-struct -fshort-enums -ffreestanding --combine -fno-inline-small-functions -fno-split-wide-types -fno-tree-scev-cprop
OPTIMIZE=-O0 -fpack-struct -fshort-enums -ffreestanding -fno-inline-small-functions -fno-split-wide-types -fno-tree-scev-cprop
ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs
OBJS=counter.o

flash: counter.au
verify: counter.av

counter.e: $(OBJS)
	avr-gcc -mmcu=$(MCU) $(OBJS) -o $@
#	ava $(AVALIB) -v --motorola -o $@ $(OBJS)

%.o: %.c
	avr-gcc -mmcu=$(MCU) -Wall  $(OPTIMIZE) -c $<

%.s: %.c
	avr-gcc -mmcu=$(MCU) -Wall $(ASFLAGS)  $(OPTIMIZE) -S $<

%.m: %.e
	avr-objcopy -O srec $< $@

%.u: %.m
	uisp $(PRGPAR) --erase
	expr `cat $(UPLOAD_COUNTER)` + 1 > $(UPLOAD_COUNTER)
	cat < $(UPLOAD_COUNTER)
	uisp $(PRGPAR) -v=3 --upload if=$<
	uisp $(PRGPAR) --verify if=$<
	echo -n -e "\\007"

%.v: %.m
	uisp $(PRGPAR) --verify if=$<

%.au: %.m
	expr `cat $(UPLOAD_COUNTER)` + 1 > $(UPLOAD_COUNTER)
	cat < $(UPLOAD_COUNTER)
	avrdude $(AVRDUDEPAR) -U flash:w:$<:s
	echo -n -e "\\007"

%.av: %.m
	avrdude $(AVRDUDEPAR) -U flash:v:$<:s
	echo -n -e "\\007"

%.ar:
	avrdude $(AVRDUDEPAR) -U flash:r:read.m:s
	echo -n -e "\\007"

%.ar:
	avrdude $(AVRDUDEPAR) -U flash:r:read.m:s
	echo -n -e "\\007"

fuse:
	avrdude $(AVRDUDEPAR) $(FUSEOPT)

rfuse:
	avrdude $(AVRDUDEPAR) $(READFUSEOPT)
	cat fuse-low.txt fuse-high.txt fuse-extended.txt fuse-lock.txt

lock:
	avrdude $(AVRDUDEPAR) $(LOCKOPT)


term:
	avrdude $(AVRDUDEPAR) -t
	echo -n -e "\\007"

#%.w: %.m
#	expr `cat $(UPLOAD_COUNTER)` + 1 > $(UPLOAD_COUNTER)
#	cat < $(UPLOAD_COUNTER)
#	avrdude -u -p 8515 -P $(AVRTTY) -v -c stk500v2 -D -C /usr/local/etc/avrdude/avrdude.conf -U flash:w:ds182x.m:s

#%.t: %.m
#	expr `cat $(UPLOAD_COUNTER)` + 1 > $(UPLOAD_COUNTER)
#	cat < $(UPLOAD_COUNTER)
#	avrdude -u -p 8515 -P $(AVRTTY) -v -c stk500v2 -D -C /usr/local/etc/avrdude/avrdude.conf -U flash:v:ds182x.m:s

run:
	stty speed 9600 raw -parenb < $(AVRTTY)
	cat $(AVRTTY)

clean:
	rm -f *.o *.s *.m *.r *.e *~ fuse-*.txt
