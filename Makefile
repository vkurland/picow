
all: picow.hex

picow.o:  picow.asm
	/sw/bin/gpasm -p 12F675 -c $<

ds1wire.o:  ds1wire.asm ds1_address.inc
	/sw/bin/gpasm -p 12F675 -c $<


picow.hex:  picow.o ds1wire.o
	/sw/bin/gplink -o picow.hex picow.o ds1wire.o

