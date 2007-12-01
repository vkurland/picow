        
        ;; ****************************************************************
        ;;
        ;; 1-wire master primitives
        ;; 
        ;; ****************************************************************

;;------------------------------------------------------------------------------
;; sends reset pulse
        
ds1rst: 
        call	owout_line_low  ; dq low
        movlw   0x37            ; start 400 us timer (200 counts)
        movwf   TMR0
        bcf     INTCON,T0IF
        btfss   INTCON,T0IF     
        goto    $-1
        call    owin

        return
        
        ;; write time slot
        ;; bit to send is in C
dm1wr:  
        call    owout_line_low
        movlw   0x03
        movwf   dlyctr          ; wait 10us
        decfsz  dlyctr,f
        goto    $-1
        btfsc   STATUS,C
        call    owin            ; release the line
        movlw   0x14            ; wait 60 us
        movwf   dlyctr
        decfsz  dlyctr,f
        goto    $-1
        call    owin
        return

        ;; read time slot
        ;; return bit in C
dm1rd:
        call    owout_line_low
        call    delay4us          ; wait 5us
        nop
        call    owin
        call    delay4us          ; wait 5us
        nop
        bcf     STATUS,C
        TEST1WSC
        bsf     STATUS,C
        movlw   0x17            ; wait 70 us
        movwf   dlyctr
        decfsz  dlyctr,f
        goto    $-1
        return

        ;; send reset, wait for presence pulse
        ;; if presence received, return with C cleared
dm1res: 
        call    owout_line_low
        movlw   0xb7            ; wait 550 us
        movwf   dlyctr
        decfsz  dlyctr,f
        goto    $-1
        call    owin
        movlw   0x17            ; wait 70 us
        movwf   dlyctr
        decfsz  dlyctr,f
        goto    $-1
        bsf     STATUS,C
        TEST1WSS
        bcf     STATUS,C        ; line low, detected presence pulse
        movlw   0x95            ; wait 450 us
        movwf   dlyctr
        decfsz  dlyctr,f
        goto    $-1
        return

        ;; send a byte, byte is in outdat
dm1sen: movlw   0x08
        movwf   bitctr
dm1sen1:
        rrf     outdat,f        
        call    dm1wr
        decfsz  bitctr,f
        goto    dm1sen1
        return                  

        ;; receive a byte, return it in indat
dm1rec: movlw   0x08
        movwf   bitctr
        clrf    indat
dm1rec1:        
        call    dm1rd
        rrf     indat,f
        decfsz  bitctr,f
        goto    dm1rec1
        return
