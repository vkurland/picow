
all:  feeder.hex feeder.hex

../ds1wire-1pin.o: ../ds1wire-1pin.asm ../ds1_address.inc ../ds1.inc
	/sw/bin/gpasm -p 12F675  -I.. -c $<

feeder.o:  feeder.asm
	/sw/bin/gpasm -p 12F675  -I.. -c $<

feeder.hex:  feeder.o ../ds1wire-1pin.o
	/sw/bin/gplink -o feeder.hex feeder.o ../ds1wire-1pin.o

clean:
	rm -f *.o *.cod *.hex *.lst ../ds1wire-1pin.o

pic:	feeder.hex
	/sw/bin/usb_pickit feeder.hex

1:      
	../crc8.py FE.000000000001 > ../ds1_address.inc
	make pic

c:      
	../crc8.py FE.00000000000C > ../ds1_address.inc
	make pic

