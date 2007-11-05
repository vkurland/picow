
        ;; ############################################################
        ;; 
        ;; Dallas 1-wire routines
        ;;
        ;; Requirements:
        ;;
        ;; Timer 0 (TMR0) should be configured to run
        ;; of the internal clock with 2:1 prescaler
        ;;
        ;; Using interrupt-on-change, this feature is not available
        ;; in the code using these 1-wire routines
        ;;
        ;; 1-wire bus is connected to a single GPIO port,
        ;; which is dynamically switched between input and output
        ;; modes. Port number should be passed in W in call to ds1init
        ;;
        ;; To initialize:
        ;;
        ;; movlw   GPIO4
        ;; call    ds1init
        ;; 
        ;; ds1wait blocks and waits for 1wire reset pulse, then sends
        ;; presence pulse
        ;; 
        ;; How to put device to 'sleep' while wating for the reset pulse:
        ;; (note that ds1init enables interrupt-on-change - sets IOC
        ;; and clears GPIF bit)
        ;;
        ;  movlw   GPIO4
        ;  call    ds1init
        ;  bsf     INTCON,GPIE     ; enable GPIO change interrupt
        ;  sleep
        ;  nop
        ;  bcf     INTCON,GPIE     ; disable Interrupt on GPIO port change
        ;  call    ds1wait
        ;
        ;;
        ;; typical sequence:
        ;;
        ;; main_loop:
        ;;      bsf     INTCON, GIE     ; enable interrupts
        ;;      call    ds1wait         ; wait for 'reset' pulse
        ;;                              ; and send 'presence' pulse
        ;;      call    ds1rec_open_ended ; read command (disables interrupts)
        ;;      btfsc   dsstat,1
        ;;      goto    main_loop
        ;;      ;  command in indat
        ;;
        ;; 
        ;; ############################################################

        include p12f683.inc

        errorlevel  -302               ; suppress message 302 from list file

DS1WIRE_CODE    set    1
        
        include ds1_address.inc
        include ds1.inc
        
DS1W_VARS   UDATA       0x40
dsstat          RES     1
bitctr          RES     1
indat           RES     1
indat1          RES     1
indat2          RES     1
indat3          RES     1
indat4          RES     1
outdat          RES     1
addr_idx        RES     1
byte_true       RES     1
byte_compl      RES     1
bcntr           RES     1
tmpbit          RES     1
tmpind          RES     1
long_timeout1   RES     1
rx_byte_count   RES     1
ds1iobit        RES     1
ds1iobit_c      RES     1
dlyctr          RES     1

DS1W_C  CODE
        DA      "Copyright 2007, Vadm Kurland"
        DA      "v1.1"
        
        ;; check 1-wire line and skip next if line is low
        ;; note that since we ue bit Z, we actually skip when
        ;; Z is set (hence btfss)
TEST1WSC  macro
        movfw   GPIO
        andwf   ds1iobit,w
        btfss   STATUS,Z
        endm
        
        ;; check 1-wire line and skip next if line is high
TEST1WSS  macro
        movfw   GPIO
        andwf   ds1iobit,w
        btfsc   STATUS,Z
        endm



SR_1_BIT_OP macro
        TEST1WSS
        goto    $-3
        ;;  send the bit
        rrf     byte_true,f
        call    ds1wr
        ;; ~32us from this moment till the beginning of the next slot
        ;;  send the same bit, complemented, bit is still in C
        call    ds1wr_r
        btfsc   dsstat,dareset
        ;goto    wait_line_high_final
        goto    wait_line_high_long
        ;; read bit back from the master
        ;; 
        ;; bit GPIF indicates mismatch between current value of
        ;; the GPIO bit and its value on the last read.
        ;; Last read was inside ds1wr where we waited for the line to
        ;; become high. Now we wait for the line to come low. No need
        ;; to read GPIO to clear mismatch, in fact, if for some reason
        ;; line has already come low and mismatch occured, we need to know.
        ;; In that case bit GPIF will be set immediately after we clear it,
        ;; and that is good for us.
        bcf     INTCON,GPIF
        btfss   INTCON,GPIF
        goto    $-1
        ;; the line is low at this time
        rlf     byte_true,f     ; put the bit back
        bcf     STATUS,C
        call    ds1ret          ; 4us
        call    ds1ret          ; 4us
        TEST1WSC
        bsf     STATUS,C        ; line is high == '1'
        rlf     tmpbit,w
        ;; bit that was sent by the master is in W (W=0 or W=1)
        ;; my own bit is in byte_true (also as 0 or 1)
        xorwf   byte_true,f
        btfsc   byte_true,0
        goto    sr_no_match       ; *** NO MATCH ***
        rrf     byte_true,f
        endm


MR_1_BIT_OP macro
        TEST1WSS
        goto    $-3
        bcf     INTCON,GPIF
        btfss   INTCON,GPIF
        goto    $-1
        bcf     STATUS,C
        call    ds1ret          ; 4us
        call    ds1ret          ; 4us
        call    ds1ret          ; 4us
        TEST1WSC
        bsf     STATUS,C        ; line is high == '1'
        rrf     tmpbit,f
        endm

owin:
        ;BANKSEL TRISIO        
        ;bsf     TRISIO,4       ; win/wout is input
        movfw   ds1iobit
        BANKSEL TRISIO        
        iorwf   TRISIO,f
        BANKSEL GPIO
        return

owout_line_low: 
        ;BANKSEL TRISIO        
        ;bcf     TRISIO,4       ; win/wout is output
        ;BANKSEL GPIO
        ;bcf     GPIO,4         ; dq low
        movfw   ds1iobit_c
        BANKSEL TRISIO
        andwf   TRISIO,f
        BANKSEL GPIO
        andwf   GPIO,f
        return
        
        
        ;; ################################################################
        
        ;;------------------------------------------------------
        ;; We use interrupt-on-change to quickly detect changes in
        ;; 1-wire line state. We enable interrupt-on-change only
        ;; for the GPIO port used for 1-wire communication. This
        ;; feature is disabled for all other GPIO ports to avoid
        ;; interference. 
ds1init:
        movwf   tmpbit
        incf    tmpbit,f
        bsf     STATUS,C
        clrf    ds1iobit
        rlf     ds1iobit,f
        decfsz  tmpbit,f
        goto    $-2
        comf    ds1iobit,w
        movwf   ds1iobit_c
        movfw   ds1iobit
        BANKSEL IOC
        movwf   IOC             ; enable interrupt-on-change for the given gpio
        BANKSEL GPIO
ds1close:
        call    owin
        movfw   GPIO            ; clear GPIF mismatch condition, if any
        bcf     INTCON,GPIF
        return
        
        ;; block and wait for 1wire reset pulse, then send presence pulse
        ;; 
        ;; How to put device to 'sleep' while wating for the reset pulse:
        ;; 
        ;; (note that ds1init enables interrupt-on-change - sets IOC
        ;; and clears GPIF bit)
        ;;
        ;  movlw   GPIO4
        ;  call    ds1init
        ;  bsf     INTCON,GPIE     ; enable GPIO change interrupt
        ;  sleep
        ;  nop
        ;  bcf     INTCON,GPIE     ; disable Interrupt on GPIO port change
        ;  call    ds1wait
        ;  
ds1wait:
        movlw   0x37            ; start 400 us timer (200 counts)
        goto    actual_ds1_wait

        ;; wait for the reset pulse but start with line low
        ;; and use short timeout. Use after ds1rec_open_ended
        ;; detected start of the reset pulse (hense short timeout)
ds1wait_short:
        movlw   0xb4            ; start 150 us timer
        ;; wait for the line to go low, then wait for it to go high,
        ;; making sure it stays low long enough to be reset.
        ;; Timer constant for the timeout is in W on entry
        ;; 
actual_ds1_wait:
        call    ds1_wait_reset
        btfsc   STATUS,C
        goto    ds1wait
        call    ds1pres         ; got reset, send presence        
        return

;; Just block and wait for reset, then return
;; in case of error returns with bit C set        
ds1_wait_reset:
        movwf   tmpbit
        bcf     STATUS,C
        TEST1WSC
        goto    $-3

_ds1wai3:
        bcf     dsstat,dareset  ; dq is low, clear reset flag
        movfw   tmpbit
        movwf   TMR0
        bcf     INTCON,T0IF

_ds1wai1:
        TEST1WSS                ; test line, skip if high
        goto    _line_still_low
        bsf     STATUS,C        ; dq high too soon
        return
        
_line_still_low:        
        ;; check if timer timed out (long enough to be reset)
        btfss   INTCON,T0IF
        goto    _ds1wai1        ; No timeout yet, loop and wait again
        TEST1WSS                ; Timeout, line stayed low long enough.
                                ; Wait till it is high again.
        goto    $-3
        return
        
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
        
;;------------------------------------------------------------------------------
;; sends presence pulse
        
ds1pres movlw   0xf5            ; wait ~20 us
        movwf   TMR0
        bcf     INTCON,T0IF
        btfss   INTCON,T0IF     
        goto    $-1

        call	owout_line_low  ; dq low
        movlw   0xaf            ; wait ~160 us
        movwf   TMR0
        bcf     INTCON,T0IF
        btfss   INTCON,T0IF     
        goto    $-1
        call    owin

        return
        
;; ****************************************************************
;; Receive one byte, check for timeout while waiting for line to go high
;; In the end wait for the line to come high
ds1rec: movlw   0x08
        movwf   bitctr
        bcf     dsstat,dareset
        call    ds1rec1
        TEST1WSS
        goto    $-3
        return

;; ****************************************************************
;; Receive one byte, check for timeout while waiting for line to go high
;; Do not wait for  the line to become high in the end (regardless of timeout).
;; If timeout occurs, return with dareset bit set, this is usually an indication
;; that master issued bus reset
ds1rec_open_ended:
        movlw   0x08
        movwf   bitctr
        bcf     dsstat,dareset
        call    ds1rec1
        return
        
;; ****************************************************************
;; Core routine to read 1 byte:
;; waits for the line to go low, reads 8 bits, in the end
;; does not wait for the line to go high after 8 bits have been read
;; disables interrupts as soon as line goes low in the beginning
ds1rec1:
        movfw   GPIO
        bcf     INTCON,GPIF
        btfss   INTCON,GPIF
        goto    $-1
        
        bcf     INTCON, GIE     ; disable all interrupts
        
get_bit:        
        call    ds1recx         ; 4us
        call    ds1recx         ; 4us
        ;call    ds1recx         ; 4us
        TEST1WSC                ; test line, skip if low
        goto    got_1           ; line is high == '1'

        bcf     STATUS,C        ; line is low == '0'
        rrf     indat,f         

        decfsz  bitctr,f        
        goto    ds1rec_wait_high
        return
        
        ;;  wait till line goes high or timeout
ds1rec_wait_high:       
        movlw   0x80            ; 250 us - timeout
        movwf   TMR0
        bcf     INTCON,T0IF
wait_0:
        TEST1WSC                ; test line, skip if low
        goto    ds1rec1
        btfss   INTCON,T0IF     
        goto    wait_0

        ;;  timeout, set error bit and exit
        bsf     dsstat,dareset
        return

got_1:  bsf     STATUS,C        
        rrf     indat,f         

        decfsz  bitctr,f        
        goto    ds1rec1         
ds1recx:
        return
        

;; ****************************************************************
;; read 2 bytes, put them in indat1, indat2
;;        
;; Using delayed initialization to conserve time at the beginning
;; This routine may be used when there is very little time gap,
;; next to none, between chip-specific subcommand and following
;; data bytes. E.g. this is the case for subcommand 0x5A for DS2413
;; (command MATCH_ROM, subsommand 0x5A "PIO access write")
;; This command sends two bytes from the master immediately after
;; the command code. We need to move into waiting for the line to
;; go low as soon as possible after we matched command code.
;; ****************************************************************
ds1_rx2:
        clrf    bitctr
        
_rx_rec1:
        ;; wait for the line to go low
        movfw   GPIO
        bcf     INTCON,GPIF
        btfss   INTCON,GPIF
        goto    $-1

        movf    bitctr,f
        btfss   STATUS,Z
        goto    _rx_cont

        ;; bitctr==0, delayed initialization
        movlw   2
        movwf   rx_byte_count
        movlw   indat1
        movwf   FSR
        movlw   0x08
        movwf   bitctr
        bcf     dsstat,dareset

_rx_cont:      
        call    ds1recx         ; 4us
        call    ds1recx         ; 4us
        call    ds1recx         ; 4us
        
        TEST1WSC
        goto    _rx_got_1      ; line is high == '1'

        bcf     STATUS,C        ; line is low == '0'
        rrf     INDF,f         

        decfsz  bitctr,f        
        goto    _rx_wait_high
        goto    _rx_ds1rec_byte_done

        ;;  wait till line goes high or timeout
_rx_wait_high:
        movlw   0x80            ; 250 us - timeout
        movwf   TMR0
        bcf     INTCON,T0IF
_rx_wait_0:
        TEST1WSC
        goto    _rx_rec1
        btfss   INTCON,T0IF     
        goto    _rx_wait_0

        ;;  timeout, set error bit and exit
        bsf     dsstat,dareset
        return

_rx_got_1:
        bsf     STATUS,C        
        rrf     INDF,f         

_rx_ds1rec_loop:       
        decfsz  bitctr,f        
        goto    _rx_rec1

_rx_ds1rec_byte_done:
        incf    FSR,f
        movlw   0x08
        movwf   bitctr
        decfsz  rx_byte_count,f
        goto    _rx_wait_high
        
        return

;; ****************************************************************
;; read 3 bytes, put them in indat1, indat2, indat3
;; Just like ds1_rx2 but three bytes
;; ****************************************************************
ds1_rx3:
        clrf    bitctr

        ;; wait for the line to go low
        movfw   GPIO
        bcf     INTCON,GPIF
        btfss   INTCON,GPIF
        goto    $-1

        movf    bitctr,f
        btfss   STATUS,Z
        goto    _rx_cont

        ;; bitctr==0, delayed initialization
        movlw   3
        movwf   rx_byte_count
        movlw   indat1
        movwf   FSR
        movlw   0x08
        movwf   bitctr
        bcf     dsstat,dareset
        
        goto    _rx_cont
        
;; ****************************************************************
;; read 4 bytes, put them in indat1, indat2, indat3, indat4
;; Just like ds1_rx2 but four bytes
;; ****************************************************************
ds1_rx4:
        clrf    bitctr

        ;; wait for the line to go low
        movfw   GPIO
        bcf     INTCON,GPIF
        btfss   INTCON,GPIF
        goto    $-1

        movf    bitctr,f
        btfss   STATUS,Z
        goto    _rx_cont

        ;; bitctr==0, delayed initialization
        movlw   4
        movwf   rx_byte_count
        movlw   indat1
        movwf   FSR
        movlw   0x08
        movwf   bitctr
        bcf     dsstat,dareset
        
        goto    _rx_cont
        
        
        
;;-----------------------------------------------------------------------------
;; sends 1 bit
;;  bit to be sent is passed via carry
;;  assumes master has sent reset pulse

ds1wr:  bcf     dsstat,dareset
        btfss   STATUS,C        
        goto    ds1wr_0        ; send '0'

        ;; sending 1
ds1wr_1:        
        ;; wait for the line to go low
        movfw   GPIO
        bcf     INTCON,GPIF
        btfss   INTCON,GPIF
        goto    $-1
        goto    ds1wr_hold
       
        ;; sending 0
ds1wr_0:        
        ;; wait for the line to go low
        movfw   GPIO
        bcf     INTCON,GPIF
        btfss   INTCON,GPIF
        goto    $-1
        call	owout_line_low
ds1wr_hold:     
        call    ds1wrx          ; 4us
        call    ds1wrx
        call    ds1wrx
        call    ds1wrx
        call    ds1wrx          ; 4us
        ;call    ds1wrx         ; 4us
        bcf     INTCON,T0IF     
        ;;  prepare to wait for line to go high or timeout
        movlw   0xcd            ; 100 us (slot=120us, but we have
                                ; spent ~20us already) 
        movwf   TMR0
        call	owin

        ;; this spot is very time critical. Master drives line
        ;; high and then very quickly starts new slot, so the line
        ;; goes low really fast. We should detect this short pulse
        ;; of when line is high and return.
wait_1:
        TEST1WSC
        return
        btfss   INTCON,T0IF     
        goto    wait_1

        ;;  timeout, set error bit and still wait for the line to go high
        bsf     dsstat,dareset
        TEST1WSS
        goto    $-3
ds1wrx: return

        
        ;;----------------------------------------------------
        ;; sends 1 bit, reversed
        ;; bit to be sent is passed via carry
        ;; assumes master has sent reset pulse
        ;;
        ;; do not clear error status bit to save a little
        ;; time. We check for errors in search_rom after send both
        ;; true and reverse bit
ds1wr_r:
        btfsc   STATUS,C        
        goto    ds1wr_0
        goto    ds1wr_1
       
        
        ;; ***************************************************
        ;; Send one byte 
ds1sen: movlw   0x08
        movwf   bitctr
ds1sen1:
        rrf     outdat,f        
        call    ds1wr           
        btfsc   dsstat,dareset  
        goto    ds1senx         
        decfsz  bitctr,f
        goto    ds1sen1
ds1senx:
        return                  

;; ****************************************************************
;; Send one byte but check for the timeout while waiting
;; for the line to go low (at th beginning of the process). 
;; If timeout occurs, return with dareset bit set, this is an indication
;; that master issued bus reset
        
ds1sen_with_timeout:
        movlw   0x08
        movwf   bitctr
        bcf     dsstat,dareset
        
        movlw   D'200'           ; 200*256*2 us
        call    wait_line_low_with_timeout
        btfsc   dsstat,dareset
        return
        goto    _sen_loop0

_sen_loop1:
        TEST1WSC
        goto    $-3
        
_sen_loop0:
        rrf     outdat,f
        btfsc   STATUS,C        
        goto    _sen_hold_line       ; send '1'
        ;;  send '0'
        call	owout_line_low  ; drive line low
_sen_hold_line: 
        call    ds1wrx          ; 4us
        call    ds1wrx
        call    ds1wrx
        call    ds1wrx
        call    ds1wrx          ; 4us
        call    ds1wrx
        call    ds1wrx
        call    ds1wrx
;        call    ds1wrx
;        call    ds1wrx        
        call	owin
        TEST1WSS
        goto    $-3
        
        decfsz  bitctr,f
        goto    _sen_loop1

_sen_ext:       
        return
        
;; ****************************************************************
;; SEARCH_ROM
;; Performs SEARCH_ROM command (0xF0)
;; ****************************************************************

ds1_search_rom:
        clrf    tmpbit
        movlw   D'8'
        movwf   addr_idx
sr_loop:       
        movfw   addr_idx
        call    get_addr_byte
        ;; byte of the address is in W
        movwf   byte_true

        SR_1_BIT_OP
        SR_1_BIT_OP
        SR_1_BIT_OP
        SR_1_BIT_OP
        
        SR_1_BIT_OP
        SR_1_BIT_OP
        SR_1_BIT_OP
        SR_1_BIT_OP

        ;; done with one byte, need to continue
        decfsz  addr_idx,f
        goto    sr_loop
        ;; THE END, *** ALL BYTES MATCHED ***
        
sr_no_match:
        ;goto    wait_line_high_final
        goto    wait_line_high_long

        
        
;; ****************************************************************
;; MATCH_ROM
;; returns with bit dareset cleared in dsstat if address matches
;; ****************************************************************
ds1_match_rom
        bsf     dsstat,dareset
        movlw   D'8'
        movwf   addr_idx
mr_loop:
        movfw   addr_idx
        call    get_addr_byte
        ;; byte of the address is in W
        movwf   byte_true
        clrf    tmpbit

        MR_1_BIT_OP
        MR_1_BIT_OP
        MR_1_BIT_OP
        MR_1_BIT_OP

        MR_1_BIT_OP
        MR_1_BIT_OP
        MR_1_BIT_OP
        MR_1_BIT_OP
        
        ;; complete byte of the address is assembled in tmpbit
        movfw   byte_true       ; expected byte of the address
        xorwf   tmpbit,w
        btfss   STATUS,Z
        ;; byte does not match
        goto    wait_line_high_final
        ;; byte matches, move to the next
        decfsz  addr_idx,f
        goto    mr_loop
        ;; all bytes matched
        bcf     dsstat,dareset
        goto    wait_line_high_final

;; ****************************************************************
;; Waits with timeout
;; ****************************************************************
        
        ;;  wait till line goes high or timeout
        ;;  timer constant for the timeout is passed in W
wait_line_high_with_timeout:
        movwf   TMR0
        bcf     INTCON,T0IF
_wait_line_high:
        TEST1WSC
        return
        btfss   INTCON,T0IF     
        goto    _wait_line_high
        ;;  timeout
        bsf     dsstat,dareset
wait_line_high_final:  
        TEST1WSS
        goto    $-3
ds1ret  return

        ;; wait till line goes high and stays high at least 400us
wait_line_high_long:    
        TEST1WSS
        goto    $-3
        ;; line went high
        movlw   0x37            ; 400 us
        movwf   TMR0
        bcf     INTCON,T0IF
_wait_while_high:       
        TEST1WSS
        goto    wait_line_high_long ; again low
        btfss   INTCON,T0IF     
        goto    _wait_while_high
        ;; line is still high and timer rolled
        return
        
        ;;  wait till line goes low or timeout
        ;;  timeout constant is passed in W
        ;;  timeout is long, approximately W*256*2 us
wait_line_low_with_timeout:
        movwf   long_timeout1
_wllwt1:
        movlw   1
        movwf   TMR0
        bcf     INTCON,T0IF
_wait_line_low:
        TEST1WSS
        return
        btfss   INTCON,T0IF     
        goto    _wait_line_low
        decfsz  long_timeout1,f
        goto    _wllwt1
        ;;  timeout
        bsf     dsstat,dareset
        return

        ;; ****************************************************************
        ;;
        ;; 1-wire master primitives
        ;; 
        ;; ****************************************************************

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
        call    ds1ret          ; wait 5us
        nop
        call    owin
        call    ds1ret          ; wait 5us
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
        
        end                             
