
all:  pwm-module.hex

pwm-module.o:  pwm-module.asm ../ds1wire.asm ../ds1_address.inc
	/sw/bin/gpasm -p 12F683 -c $<

#ds1wire.o:  ../ds1wire.asm ds1_address.inc
#	/sw/bin/gpasm -p 12F683 -c $<

pwm-module.hex:  pwm-module.o   #ds1wire.o
	/sw/bin/gplink -o pwm-module.hex pwm-module.o #ds1wire.o

