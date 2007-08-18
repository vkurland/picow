
all:  ph-meter-controller.hex ph-meter-voltmeter.hex

../ds1wire-1pin.o: ../ds1wire-1pin.asm ../ds1_address.inc ../ds1.inc
	/sw/bin/gpasm -p 12F683 -c $<

ph-meter-controller.o:  ph-meter-controller.asm
	/sw/bin/gpasm -p 12F683 -c $<

ph-meter-controller.hex:  ph-meter-controller.o ../ds1wire-1pin.o
	/sw/bin/gplink -o ph-meter-controller.hex ph-meter-controller.o ../ds1wire-1pin.o



ph-meter-voltmeter.o:   ph-meter-voltmeter.asm
	/sw/bin/gpasm -p 12F683 -c $<

ph-meter-voltmeter.hex:  ph-meter-voltmeter.o ../ds1wire-1pin.o
	/sw/bin/gplink -o ph-meter-voltmeter.hex ph-meter-voltmeter.o ../ds1wire-1pin.o

