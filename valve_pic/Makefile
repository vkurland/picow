
all:  valve_pic.hex valve_pic.hex

../ds1wire-1pin.o: ../ds1wire-1pin.asm ../ds1_address.inc ../ds1.inc
	/sw/bin/gpasm -p 12F683 -I.. -c $<

valve_pic.o:  valve_pic.asm
	/sw/bin/gpasm -p 12F683 -I.. -c $<

valve_pic.hex:  valve_pic.o ../ds1wire-1pin.o
	/sw/bin/gplink -o valve_pic.hex valve_pic.o ../ds1wire-1pin.o

clean:
	rm -f *.o *.cod *.hex *.lst

pic:	valve_pic.hex
	/sw/bin/usb_pickit valve_pic.hex
