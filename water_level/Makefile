
all:  water_level.hex

../ds1wire-1pin.o: ../ds1wire-1pin.asm ../ds1_address.inc ../ds1.inc
	/sw/bin/gpasm -p 12F675  -I.. -c $<

water_level.o:  water_level.asm
	/sw/bin/gpasm -p 12F675 -I.. -c $<

water_level.hex:  water_level.o   ../ds1wire-1pin.o
	/sw/bin/gplink -o water_level.hex water_level.o ../ds1wire-1pin.o

clean:
	rm -f *.o *.cod *.hex *.lst ../ds1wire-1pin.o

pic:    water_level.hex
	/sw/bin/usb_pickit water_level.hex
