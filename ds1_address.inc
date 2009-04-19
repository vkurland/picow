
	;; 1-wire address fe.00.00.00.00.00.17.25 

	GLOBAL	get_addr_byte

DS1W_addr  CODE  0x40

;-----------------------------------------------------------------------------
;  Note that bytes of the address are in reverse order
; this is so we can use index as a counter and iterate using decfsz
get_addr_byte:	addwf   PCL,f
		retlw   0
ds1_addr7	retlw	0x25
ds1_addr6	retlw	0x17
ds1_addr5	retlw	0x0
ds1_addr4	retlw	0x0
ds1_addr3	retlw	0x0
ds1_addr2	retlw	0x0
ds1_addr1	retlw	0x0
ds1_addr0	retlw	0xfe
	return
