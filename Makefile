
all: picow.hex feeder.hex dual-valve.hex

picow.o:  picow.asm
	/sw/bin/gpasm -p 12F675 -c $<

feeder.o:  feeder.asm
	/sw/bin/gpasm -p 12F675 -c $<

dual-valve.o:  dual-valve.asm
	/sw/bin/gpasm -p 12F675 -c $<

ds1wire.o:  ds1wire.asm ds1_address.inc
	/sw/bin/gpasm -p 12F675 -c $<

picow.hex:  picow.o ds1wire.o
	/sw/bin/gplink -o picow.hex picow.o ds1wire.o

feeder.hex:  feeder.o ds1wire.o
	/sw/bin/gplink -o feeder.hex feeder.o ds1wire.o

dual-valve.hex:  dual-valve.o ds1wire.o
	/sw/bin/gplink -o dual-valve.hex dual-valve.o ds1wire.o

