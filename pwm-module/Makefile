
all:  pwm-module.hex

../ds1wire-1pin.o: ../ds1wire-1pin.asm ../ds1_address.inc ../ds1.inc
	/sw/bin/gpasm -p 12F683  -I.. -c $<

pwm-module.o:  pwm-module.asm
	/sw/bin/gpasm -p 12F683 -I.. -c $<

pwm-module.hex:  pwm-module.o   ../ds1wire-1pin.o
	/sw/bin/gplink -o pwm-module.hex pwm-module.o ../ds1wire-1pin.o

clean:
	rm -f *.o *.cod *.hex *.lst

pic:    pwm-module.hex
	/sw/bin/usb_pickit pwm-module.hex
