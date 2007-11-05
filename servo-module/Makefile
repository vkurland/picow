
all:  servo-module.hex

../ds1wire-1pin.o: ../ds1wire-1pin.asm ../ds1_address.inc ../ds1.inc
	/sw/bin/gpasm -p 12F683  -I.. -c $<

servo-module.o:  servo-module.asm
	/sw/bin/gpasm -p 12F683 -I.. -c $<

servo-module.hex:  servo-module.o   ../ds1wire-1pin.o
	/sw/bin/gplink -o servo-module.hex servo-module.o ../ds1wire-1pin.o

clean:
	rm -f *.o *.cod *.hex *.lst

pic:    servo-module.hex
	/sw/bin/usb_pickit servo-module.hex
