
all:  adc-module.hex

adc-module.o:  adc-module.asm ../ds1wire.asm ../ds1_address.inc
	/sw/bin/gpasm -p 12F683 -c $<

adc-module.hex:  adc-module.o
	/sw/bin/gplink -o adc-module.hex adc-module.o

