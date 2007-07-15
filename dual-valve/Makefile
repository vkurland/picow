
all:  dual-valve.hex

dual-valve.o:  dual-valve.asm ../ds1wire.asm ../ds1_address.inc
	/sw/bin/gpasm -p 12F675 -c $<

dual-valve.hex:  dual-valve.o
	/sw/bin/gplink -o dual-valve.hex dual-valve.o

