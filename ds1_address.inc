
	;; 1-wire address fe.01.02.03.04.05.06.bb 

	GLOBAL	get_addr_byte

DS1W_addr  CODE  0x40

;-----------------------------------------------------------------------------
;  Note that bytes of the address are in reverse order
; this is so we can use index as a counter and iterate using decfsz
get_addr_byte:	addwf   PCL,f
		retlw   0
ds1_addr7	retlw	0xbb
ds1_addr6	retlw	0x6
ds1_addr5	retlw	0x5
ds1_addr4	retlw	0x4
ds1_addr3	retlw	0x3
ds1_addr2	retlw	0x2
ds1_addr1	retlw	0x1
ds1_addr0	retlw	0xfe
	return
