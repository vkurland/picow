
all:  adc-module.hex

../ds1wire-1pin.o: ../ds1wire-1pin.asm ../ds1_address.inc ../ds1.inc
	/sw/bin/gpasm -p 12F675  -I.. -c $<

adc-module.o:  adc-module.asm
	/sw/bin/gpasm -p 12F675 -I.. -c $<

adc-module.hex:  adc-module.o   ../ds1wire-1pin.o
	/sw/bin/gplink -o adc-module.hex adc-module.o ../ds1wire-1pin.o

clean:
	rm -f *.o *.cod *.hex *.lst ../ds1wire-1pin.o

pic:    adc-module.hex
	/sw/bin/usb_pickit adc-module.hex
