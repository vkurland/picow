
        ;; ########################################
        ;; 
        ;; Dallas 1-wire routines
        ;; Assuming 1-wire is attached to GPIO3 (in) and GPIO4 (out)
        ;;
        ;;  Timer 0 (TMR0) should be configured to run
        ;;  of the internal clock with 2:1 prescaler
        ;; 
        ;; ########################################

        include ../ds1_address.inc
        
DS1W_VARS   UDATA       0x40
tmstmp          RES     1
dsstat          RES     1
bitctr          RES     1
indat           RES     1
indat1          RES     1
indat2          RES     1
indat3          RES     1
indat4          RES     1
outdat          RES     1
saddctr         RES     1
addr_idx        RES     1
byte_true       RES     1
byte_compl      RES     1
bcntr           RES     1
savebit         RES     1
tmpbit          RES     1
tmpind          RES     1
long_timeout1   RES     1
long_timeout2   RES     1
rx_byte_count   RES     1
        
;----- GPIO Bits --------------------------------------------------------

GP5                          EQU     H'0005'
GPIO5                        EQU     H'0005'
GP4                          EQU     H'0004'
GPIO4                        EQU     H'0004'
GP3                          EQU     H'0003'
GPIO3                        EQU     H'0003'
GP2                          EQU     H'0002'
GPIO2                        EQU     H'0002'
GP1                          EQU     H'0001'
GPIO1                        EQU     H'0001'
GP0                          EQU     H'0000'
GPIO0                        EQU     H'0000'

        GLOBAL  dsstat, ds1init, ds1wait, ds1wait_short, ds1sen
        GLOBAL  ds1rec, ds1rec_open_ended, ds1rec_enable_int
        GLOBAL  ds1_rx2, ds1_rx3, ds1_rx4
        GLOBAL  indat, indat1, indat2, indat3, indat4, outdat
        GLOBAL  ds1_search_rom, ds1_match_rom
        
        EXTERN  indicator,inddelay
        EXTERN  app_match_rom

win     equ     GPIO3           ; 1-wire in
wout    equ     GPIO4           ; 1-wire out, connected via transistor
dareset equ     1               ; reset flag bit in dsstat

DS1W_C  CODE

;-----------------------------------------------------------------------------
ds1init
        bcf     GPIO,wout       ; dq high (connected via transistor)

;; send address to indicator, byte by byte
;;         movlw   D'8'
;;         movwf   addr_idx
;; i_loop:
;;         movfw   addr_idx
;;         call    get_addr_byte
;;         call    indicator
;;         call    inddelay
;;         clrf    bcntr
;;         call    inddelay
;;         decfsz  bcntr,f
;;         goto    $-2
;;         decfsz  addr_idx,f
;;         goto    i_loop

        return
        
ds1wait:
        movlw   0x37            ; start 400 us timer (200 counts)
        goto    actual_ds1_wait

        ;; wait for the reset pulse but start with line low
        ;; and use short timeout. Use after ds1rec_open_ended
        ;; detected start of the reset pulse (hense short timeout)
ds1wait_short:
        movlw   0xb4            ; start 150 us timer
        goto    actual_ds1_wait
        
        ;; wait for the line to go low, then wait for it to go high,
        ;; making sure it stays low long enough to be reset.
        ;; Timer constant for the timeout is in W on entry
        ;; 
actual_ds1_wait:
        bsf     INTCON, GIE     ; enable interrupts
        btfss   GPIO,win        ; if line is high, sleep
        goto    ds1wai3         ; if line is low, proceed

;;  This fragment implements power saving mode (sleep) for the time when
;;  1-wire bus is inactive. PIC is woken up by "port change interrupt" on GPIO3
;;        
;;     line is high, enable interrups and sleep
;;        
;;         movf    GPIO,f
;;         BANK1
;;         bcf     INTCON,GPIF     ;Clear port change Interrupt Flag
;;         bsf     INTCON,GPIE     ;Interrupt on GPIO port change
;;         bsf     IOC,IOC3        ; enable interrupt on GPIO3 state change
;;         bsf     INTCON, GIE     ; enable interrupts
;;         BANK0
;;         sleep
;;         nop
;;         bcf     INTCON,GPIE       ; disable Interrupt on GPIO port change
        
        btfsc   GPIO,win        ; wait till dq goes low
        goto    $-1

ds1wai3:
        bcf     INTCON, GIE     ; disable all interrupts
                                ; because 1-wire is very time critical
        
        bcf     dsstat,dareset  ; dq is low, clear reset flag
        movwf   TMR0
        bcf     INTCON,T0IF
ds1wai1:
        btfsc   GPIO,win        ; is dq still low?
        goto    ds1wait         ; no, start again
        btfss   INTCON,T0IF     ; check if timer timed out (long enough to be reset)
        goto    ds1wai1         ; no, loop and wait again
ds1wai2:
        btfss   GPIO,win        ; is dq released ?
        goto    $-1             ; wait till dq goes high

        call    ds1pres         ; got reset, send presence
        return

        
;;------------------------------------------------------------------------------
;; sends presence pulse
        
ds1pres movlw   0xf5            ; wait ~20 us
        movwf   TMR0
        bcf     INTCON,T0IF
        btfss   INTCON,T0IF     
        goto    $-1

        bsf     GPIO,wout       ; dq low
        movlw   0xaf            ; wait ~160 us
        movwf   TMR0
        bcf     INTCON,T0IF
        btfss   INTCON,T0IF     
        goto    $-1
        bcf     GPIO,wout       ; dq high

        return
        
;; ****************************************************************
;; Receive one byte, check for timeout while waiting for line to go high
;; In the end wait for the line to come high
ds1rec: movlw   0x08
        movwf   bitctr
        bcf     dsstat,dareset
        call    ds1rec1
        btfss   GPIO,win
        goto    $-1
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
;; Receive one byte, in the beginning keep interrupts enabled
;; until line goes low   
ds1rec_enable_int:
        bsf     INTCON, GIE     ; enable all interrupts
        movlw   0x08
        movwf   bitctr
        bcf     dsstat,dareset
        call    ds1rec1
        return

;; ****************************************************************
ds1rec1:
        btfsc   GPIO,win       
        goto    $-1
        
        bcf     INTCON, GIE     ; disable all interrupts
        
get_bit:        
        call    ds1recx         ; 4us
        call    ds1recx         ; 4us
        call    ds1recx         ; 4us
        btfsc   GPIO,win       
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
wait_0: btfsc   GPIO,win       
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
        btfsc   GPIO,win       
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
        
        btfsc   GPIO,win       
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
        btfsc   GPIO,win       
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
        
        btfsc   GPIO,win       
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
        
        btfsc   GPIO,win       
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
;;
;; NOTE: as of 06/09/2007 using ds1wr_with_timeout
;;  instead of ds1wr breaks ROM SEARCH. DO NOT USE!
;; 
ds1wr_with_timeout:
        btfss   GPIO,win       
        goto    actual_ds1_wr    ; line is already low
        movlw   D'200'           ; 200*256*2 us ~ 100ms
        movwf   long_timeout1
        btfss   GPIO,win       
        goto    actual_ds1_wr    ; line is already low
_ds1wr_sw0:      
        movlw   D'20'             ; 20 times 200*256*2 us ~ 2 sec
        movwf   long_timeout2
        btfss   GPIO,win       
        goto    actual_ds1_wr    ; line is already low
_ds1wr_sw1:
        movlw   1
        movwf   TMR0
        bcf     INTCON,T0IF     
_ds1wr_wait_1:
        btfss   GPIO,win       
        goto    actual_ds1_wr   ; line went low
        btfss   INTCON,T0IF     
        goto    _ds1wr_wait_1
        decfsz  long_timeout2,f
        goto    _ds1wr_sw1
        decfsz  long_timeout1,f
        goto    _ds1wr_sw0
        ;;  timeout
        bsf     dsstat,dareset
        return
        
ds1wr:
        btfsc   GPIO,win       
        goto    $-1

actual_ds1_wr:    
        bcf     dsstat,dareset
        btfsc   STATUS,C        
        goto    hold_line       ; send '1'
        ;;  send '0'
        bsf     GPIO,wout       ; drive line low (transistor)
hold_line:
        call    ds1wrx          ; 4us
        call    ds1wrx
        call    ds1wrx
        call    ds1wrx
        call    ds1wrx          ; 4us
        call    ds1wrx
        
        bcf     GPIO,wout       ; release the line

        ;;  wait for line to go high or timeout
        movlw   0xcd            ; 100 us (slot=120us, but we have
                                ; spent ~20us already) 
        movwf   TMR0
        bcf     INTCON,T0IF     
wait_1  btfsc   GPIO,win       
        return
        btfss   INTCON,T0IF     
        goto    wait_1

        ;;  timeout, set error bit and still wait for the line to go high
        bsf     dsstat,dareset  
        btfss   GPIO,win       
        goto    $-1
ds1wrx  return

        
;;-----------------------------------------------------------------------------
;; sends 1 bit, reversed
;;  bit to be sent is passed via carry
;;  assumes master has sent reset pulse
        
ds1wr_r:
        bcf     dsstat,dareset
        
        btfsc   GPIO,win       
        goto    $-1
        btfss   STATUS,C        
        goto    hold_line       ; send '1' (revers)
        ;;  send '0'
        bsf     GPIO,wout       ; drive line low (transistor)
        goto    hold_line
        
;; ****************************************************************
;; Send one byte 
ds1sen: 
        movlw   0x08
        movwf   bitctr
ds1sen1 rrf     outdat,f        
        call    ds1wr           
        btfsc   dsstat,dareset  
        goto    ds1senx         
        decfsz  bitctr,f
        goto    ds1sen1
ds1senx return                  

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
        btfsc   GPIO, win
        goto    $-1

_sen_loop0:
        rrf     outdat,f
        btfsc   STATUS,C        
        goto    _sen_hold_line       ; send '1'
        ;;  send '0'
        bsf     GPIO,wout       ; drive line low (transistor)
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
        
        bcf     GPIO,wout       ; release the line
        btfss   GPIO, win
        goto    $-1

;;         ;;  wait for line to go high or timeout
;;         movlw   0xcd            ; 100 us (slot=120us, but we have spent ~20us already)
;;         movwf   TMR0
;;         bcf     INTCON,T0IF     
;; _sen_wait_1:
;;         btfsc   GPIO,win       
;;         goto    _sen_next_bit
;;         btfss   INTCON,T0IF     
;;         goto    _sen_wait_1

;;         ;;  timeout, set error bit and still wait for the line to go high
;;         bsf     dsstat,dareset  
;;         btfss   GPIO,win       
;;         goto    $-1
;;         return
;; _sen_next_bit:
        
        decfsz  bitctr,f
        goto    _sen_loop1

_sen_ext:       
        return
        
;; ****************************************************************
;; SEARCH_ROM
;; Performs SEARCH_ROM command (0xF0)
;; returns with bit dareset cleared if search matched our address
;; ****************************************************************
ds1_search_rom: 
        movlw   D'8'
        movwf   addr_idx
        movfw   addr_idx
        call    get_addr_byte
        ;; byte of the address is in W
        movwf   byte_true
        movwf   savebit
        movlw   D'8'
        movwf   bcntr
sb_loop0:       
        ;;  now send this bit
        rrf     byte_true,f
        call    ds1wr
; ~32us from this moment till the beginning of the next slot
        btfsc   dsstat,dareset
        goto    wait_line_high_final
        ;;  send the same bit, complemented, bit is still in C
        call    ds1wr_r
        btfsc   dsstat,dareset
        goto    wait_line_high_final
        ;; ----------------------------------------------------------------
        ;; read bit back from the master
        ;; process the bit and prepare for the next cycle
        ;; before wating for the line to come high (== end of a slot)
        bcf     STATUS,C
        
        btfsc   GPIO,win
        goto    $-1

        call    ds1ret          ; 4us
        call    ds1ret          ; 4us
        call    ds1ret          ; 4us
        
        btfss   GPIO,win       
        goto    _got_bit        ; line is low == '0'
        bsf     STATUS,C        ; line is high == '1'
_got_bit:
        rlf     tmpbit,w
        ;; bit that was sent by the master is in W (W=0 or W=1)
        ;; my own bit is in savebit (also as 0 or 1)
        xorwf   savebit,f
        btfsc   savebit,0
        goto    _no_match       ; *** NO MATCH ***
        ;; if rightmost bit is 0 after xor, then bits were the same
        ;; continue search
        ;; however the line might still be low
        decfsz  bcntr,f
        goto    _next_bit
        ;; done with one byte, need to continue
        decfsz  addr_idx,f
        goto    _next_byte
        ;; THE END, *** ALL BYTES MATCHED ***
        bcf     dsstat,dareset
        goto    wait_line_high_final

_next_byte:
        movfw   addr_idx
        call    get_addr_byte
        ;; byte of the address is in W
        movwf   byte_true
        movlw   D'8'
        movwf   bcntr
_next_bit:      
        ;;  save so we can check rightmost bit when master sends it back
        movfw   byte_true
        movwf   savebit
        movlw   0x80            ; 250 us
        call    wait_line_high_with_timeout
        btfss   dsstat,1
        goto    sb_loop0
        return                  ; error bit set, abort

_no_match:
        bsf     dsstat,dareset
        return
        
;; ****************************************************************
;; MATCH_ROM
;; ****************************************************************
ds1_match_rom
        movlw   D'8'
        movwf   addr_idx
        call    get_addr_byte
        ;; byte of the address is in W
        movwf   byte_true
        clrf    tmpbit
        movlw   D'8'
        movwf   bcntr
mr_loop0:       
        movfw   byte_true       ; next (expected) byte of the address
mr_loop1:       
        ;; ----------------------------------------------------------------
        ;; read back from the master
        ;; process the bit and prepare for the next cycle
        ;; before wating for the line to come high (== end of a slot)
        bcf     STATUS,C
        
        btfsc   GPIO,win
        goto    $-1

        call    ds1ret          ; 4us
        call    ds1ret          ; 4us
        call    ds1ret          ; 4us
        
        btfss   GPIO,win       
        goto    _mr_got_bit     ; line is low == '0'
        bsf     STATUS,C        ; line is high == '1'

_mr_got_bit:
        rrf     tmpbit,f

        decfsz  bcntr,f
        goto    _mr_next_bit

        ;; complete byte of the address is assembled in tmpbit
        ;; byte we expect is still in W
        xorwf   tmpbit,w
        btfss   STATUS,Z
        ;; byte does not match
        goto    wait_line_high_final
        ;; byte matches, move on to next
        decfsz  addr_idx,f
        goto    _mr_next_byte
        ;; all bytes matched
_mr_run_app:
        bcf     dsstat,dareset
        goto    wait_line_high_final

_mr_next_bit:
        btfss   GPIO,win
        goto    $-1
        goto    mr_loop1
        
_mr_next_byte:
        movfw   addr_idx
        call    get_addr_byte
        ;; byte of the address is in W
        movwf   byte_true
        clrf    tmpbit
        movlw   D'8'
        movwf   bcntr
        movlw   0x80            ; 250 us
        call    wait_line_high_with_timeout
        btfss   dsstat,1
        goto    mr_loop0
        return                  ; error, abort
        
;; ****************************************************************
;; Waits with timeout
;; ****************************************************************
        
        ;;  wait till line goes high or timeout
        ;;  timer constant for the timeout is passed in W
wait_line_high_with_timeout:
        movwf   TMR0
        bcf     INTCON,T0IF
_wait_line_high:
        btfsc   GPIO,win       
        return
        btfss   INTCON,T0IF     
        goto    _wait_line_high
        ;;  timeout
        bsf     dsstat,dareset
wait_line_high_final:  
        btfss   GPIO,win       
        goto    $-1
ds1ret  return



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
        btfss   GPIO,win       
        return
        btfss   INTCON,T0IF     
        goto    _wait_line_low
        decfsz  long_timeout1,f
        goto    _wllwt1
        ;;  timeout
        bsf     dsstat,dareset
        return

        
;; end                             ;
