
all:  adc-module.hex

adc-module.o:  adc-module.asm ../ds1wire-1pin.asm ../ds1_address.inc
	/sw/bin/gpasm -p 12F675 -e ON -c $<

adc-module.hex:  adc-module.o
	/sw/bin/gplink -o adc-module.hex adc-module.o

