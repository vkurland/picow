
all:  adc-module.hex

../ds1wire-1pin.o: ../ds1wire-1pin.asm ../ds1_address.inc ../ds1.inc
	gpasm -p 12F675  -I.. -c $<

adc-module.o:  adc-module.asm
	gpasm -p 12F675 -I.. -c $<

adc-module.hex:  adc-module.o   ../ds1wire-1pin.o
	gplink -o adc-module.hex adc-module.o ../ds1wire-1pin.o

clean:
	rm -f *.o *.cod *.hex *.lst ../ds1wire-1pin.o

pic:    adc-module.hex
	usb_pickit adc-module.hex
