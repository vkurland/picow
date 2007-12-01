
        ;; ############################################################
        ;; 
        ;; Dallas 1-wire routines
        ;;
        ;; Requirements:
        ;;
        ;; Uses Timer 0 (TMR0), configred with no prescaler (running
        ;; at 1:1 freq). User program can utilize TMR0 in hook functions
        ;; to measure short intervals of time but should not change
        ;; prescaler and other settings of TMR0.
        ;;
        ;; Uses interrupt-on-change, this feature is not available
        ;; in the code using these 1-wire routines
        ;;
        ;; Uses WDT, WDT should be enabled in configuration word
        ;; Assigns prescaler to WDT, sets it to 1:32 which yields WDT
        ;; timeout of 0.57 sec
        ;;
        ;; Completely owns OPTION_REG register and resets it periodically
        ;; to restore WDT prescaler. Specifically controls bits
        ;; PS0-2, PSA, T0SE,  T0CS of OPTION_REG . Two higher bits
        ;; (GPPU and INTEDG) can be controlled by user program; control
        ;; word can be passed via set_option_reg_bits function.
        ;; 
        ;; 1-wire bus is connected to a single GPIO port,
        ;; which is dynamically switched between input and output
        ;; modes. Port number should be passed in W in call to ds1init
        ;;
        ;; To initialize:
        ;;
        ;; movlw   GPIO4
        ;; call    ds1init
        ;; clrw                    ; OPTION_REG bits GPPU and INTEDG == 0
        ;; call    set_option_reg_bits
        ;; movlw   GPIO5
        ;; call    set_activity_led_port
        ;; movlw   GPIO2
        ;; call    set_error_led_port
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
        ;; ============= API sunctions ==================================
        ;;
        ;;  Setup:
        ;; 
        ;; ds1init:             pass GPIO port used for 1-wire in W
        ;; 
        ;; set_option_reg_bits: pass bits for OPTION_REG in W
        ;; ds1wire module controls bits PS0-2, PSA, T0SE,  T0CS
        ;; other bits are passed by user program using set_option_reg_bits
        ;;
        ;; set_activity_led_port: pass GPIO port used for activity LED in W
        ;; 
        ;; set_error_led_port:    pass GPIO port used for error indic. LED in W
        ;; 
        ;; 
        ;; User functions are implemeted in three 'hook' functions:
        ;; read_register_hook, write_to_register_hook, idle_hook
        ;;
        ;; read_register_hook:  called in MATCH_ROM, function 0xF5 right before
        ;;                      register contents is sent to the master
        ;;
        ;; write_register-hook: called in MATCH_ROM, function 0x5A after
        ;;                      data has been written to the register
        ;;
        ;; idle_hook:           called while waiting for reset pulse
        ;;
        ;;  Controlling indicator LEDs:
        ;; 
        ;; actled_on, actled_off: turn activity LED on / off
        ;; errled_on, errled_off: turn error indicator LED on / off
        ;;
        ;;  Main loop:
        ;; 
        ;; ds1main:             main loop
        ;;
        ;; ############################################################

        include p12f675.inc

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
long_timer_val  RES     1
long_timer_cntr RES     1


;;; 1-wire I/O bit
ds1iobit        RES     1
ds1iobit_c      RES     1
;;; activity LED bit
actledbit       RES     1
actledbit_c     RES     1
;;; Error indicator bit
errledbit       RES     1
errledbit_c     RES     1          

option_reg_bits RES     1
        
dlyctr          RES     1

REGISTERS       RES     8       ; 8 1-byte registers

#define OPTION_BITS     b'00001101' ; assign prescaler to WDT, prescaler 1:32
                                    ; prescaler 1:32 -> WDT timeout 0.57sec

DS1W_C  CODE
        DA      "Copyright 2007, Vadm Kurland"
        DA      "v1.2"
        
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
        call    delay4us          ; 4us
        call    delay4us          ; 4us
        call    delay4us          ; 4us
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
        call    delay4us          ; 4us
        call    delay4us          ; 4us
        call    delay4us          ; 4us
        TEST1WSC
        bsf     STATUS,C        ; line is high == '1'
        rrf     tmpbit,f
        endm

owin:
        movfw   ds1iobit
        BANKSEL TRISIO        
        iorwf   TRISIO,f
        BANKSEL GPIO
delay4us:
        return

owout_line_low: 
        movfw   ds1iobit_c
        BANKSEL TRISIO
        andwf   TRISIO,f
        BANKSEL GPIO
        andwf   GPIO,f
        return
        
actled_on:
        ;; 'activity' LED on
        movfw   actledbit
        BANKSEL GPIO
        iorwf   GPIO,f
        return

actled_off:
        ;; 'activity' LED off
        movfw   actledbit_c
        BANKSEL GPIO
        andwf   GPIO,f
        return


errled_on:
        ;; 'error' LED on
        movfw   errledbit
        BANKSEL GPIO
        iorwf   GPIO,f
        return

errled_off:
        ;; 'error' LED off
        movfw   errledbit_c
        BANKSEL GPIO
        andwf   GPIO,f
        return


        ;; ################################################################

        ;; convert bit number to bit mask
        ;; bit number passed in W, bit mask returned in W
getbitmask: 
        movwf   tmpind
        incf    tmpind,f
        bsf     STATUS,C
        clrf    tmpbit
        rlf     tmpbit,f
        decfsz  tmpind,f
        goto    $-2
        movfw   tmpbit
        return

reg_init:       
        movlw   REGISTERS
        movwf   FSR
        movlw   D'8'
        movwf   bcntr
        clrf    INDF
        incf    FSR,f
        decfsz  bcntr,f
        goto    $-3
        return
        
        ;; Clear WDT and reset prescaler (prescaler is cleared
        ;; by clrwdt command)      
clearwdt:
        BANKSEL OPTION_REG
        clrwdt
        movfw   option_reg_bits
        movwf   OPTION_REG
        BANKSEL GPIO
        return
        
        
        ;;------------------------------------------------------
        ;; We use interrupt-on-change to quickly detect changes in
        ;; 1-wire line state. We enable interrupt-on-change only
        ;; for the GPIO port used for 1-wire communication. This
        ;; feature is disabled for all other GPIO ports to avoid
        ;; interference. 
ds1init:
        call    getbitmask
        movwf   ds1iobit
        movwf   ds1iobit_c
        comf    ds1iobit_c,f

        BANKSEL IOC
        movwf   IOC             ; enable interrupt-on-change for the given gpio
        BANKSEL GPIO

        clrf    actledbit
        clrf    actledbit_c
        comf    actledbit_c,f

        clrf    errledbit
        clrf    errledbit_c
        comf    errledbit_c,f

        call    clearwdt

        BANKSEL GPIO
        ;; if NOT_POR bit is '0', this means  WDT timeout occured
        btfsc   STATUS, NOT_TO
        call    reg_init        ; clear registers on power-on
        btfss   STATUS, NOT_TO
        call    errled_on       ; err led on if WDT reset
        
ds1close:
        call    actled_off
        call    owin
        movfw   GPIO            ; clear GPIF mismatch condition, if any
        bcf     INTCON,GPIF
        return

        ;; pass control word for the OPTION_REG
        ;; ds1wire module controls bits PS0-2, PSA, T0SE,  T0CS
        ;; other bits are passed by user program using set_option_reg_bits
        ;; 
set_option_reg_bits:
        movwf   option_reg_bits
        movlw   b'11000000'
        andwf   option_reg_bits,f
        movlw   OPTION_BITS
        iorwf   option_reg_bits,f
        return
        
        ;; use GPIO channel passed in W as 'activity' indicator
set_activity_led_port:
        call    getbitmask
        movwf   actledbit
        movwf   actledbit_c
        comf    actledbit_c,f
        return

        ;; use GPIO channel passed in W as 'error' indicator
set_error_led_port:
        call    getbitmask
        movwf   errledbit
        movwf   errledbit_c
        comf    errledbit_c,f
        return
        
        ;;------------------------------------------------------
        ;; ds1 main loop

wait_reset_end:
        call	clearwdt
        call    ds1wait_short
        goto    wait_cmd

gen_error:
        call    errled_on

ds1main:
        call    ds1wait

wait_cmd:
;;; Need to wait a little bit for slow chips to get in sync with protocol.
;;; Since different devices can generate presence pulse of different length,
;;; line can possibly still be low when we get to this point. Usually there
;;; is a pause of 1-3 ms between reset-presence pulses and the command
;;; sent by the master so it is safe to wait a little before we start reading
;;; command.
        movlw   0xCD             ; 50 us
        movwf   TMR0
        bcf     INTCON,T0IF
        btfss   INTCON,T0IF     
        goto    $-1

        call    delay4us
        call    delay4us
        call    delay4us
        call    delay4us

        call    ds1rec_open_ended
        btfsc   dsstat,1
        goto    wait_reset_end

cmd:
        movlw   SEARCH_ROM
        subwf   indat,w
        btfss   STATUS,Z
        goto    mr

        ;; SEARCH ROM command
        call    actled_on
        call    ds1_search_rom
        call    actled_off
        ;; we do not support any subcommands after SEARCH_ROM at this time
        goto    ds1main
        
mr:     movlw   MATCH_ROM
        subwf   indat,w
        btfss   STATUS,Z
        goto    gen_error

        ;;  MATCH ROM command
        call    ds1_match_rom
        btfsc   dsstat,1
        goto    ds1main          ; match_rom did not match our address
        
        call    actled_on
        ;; Perform operations specific to MATCH_ROM
        call    ds1rec
        call    actled_off
        
        movlw   0xF5
        subwf   indat,w
        btfss   STATUS,Z
        goto    reg_write

        ;; Command 0xF5: read content of the register N
        ;; register number follows (1 byte)
        call    ds1rec

        ;; call hook so that user's program can make changes in registers
        ;; right before they are read
        call    read_register_hook
        
        movfw   indat
        addlw   REGISTERS
        movwf   FSR
        movfw   INDF
        movwf   outdat
        call    ds1sen

        goto    ds1main

reg_write:
        movlw   0x5A
        subwf   indat,w
        btfss   STATUS,Z
        goto    gen_error         ; illegal command

        ;; Command 0x5A: write two bytes into the register N
        ;; register number follows (1 byte)
        ;; receive 3 bytes from the master (indat1, indat2, indat3)
        call    ds1_rx3
        movfw   indat1
        addlw   REGISTERS
        movwf   FSR
        ;; check data integrity
        movlw   0xAA
        movwf   outdat
        comf    indat3,w
        xorwf   indat2,w
        btfss   STATUS,Z
        goto    reg_wr_err
        movfw   indat2
        movwf   INDF
        call    ds1sen
send_reg:
        movfw   INDF
        movwf   outdat
        call    ds1sen

        call    write_to_register_hook
        
        goto    ds1main

reg_wr_err:
        movlw   0xA0
        movwf   outdat
        call    ds1sen
        clrf    outdat
        call    ds1sen
        goto    ds1main


        
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


;;; Wait for the line to go low, then wait for it to go high,
;;; making sure it stays low long enough to be reset.
;;; Timer constant for the timeout is in W on entry
;;; 
;;; Just block and wait for reset, then return
;;; in case of error returns with bit C set        
ds1wait:
        movlw   d'100'            ; start 400 us timer
        movwf   tmpbit
        call    idle_loop

measure_reset_pulse:    
        movfw   tmpbit
        call    long_timer_init
_while_line_low:
        TEST1WSC                ; test line, skip if low
        goto    ds1wait         ; line went high too soon
        call    long_timer_check
        btfss   INTCON,T0IF     
        goto    _while_line_low

        TEST1WSS                ; Timeout, line was low long enough.
        goto    $-3             ; Wait till it is high again.

        call    ds1pres         ; got reset, send presence        
        return  

;;; Wait for the reset pulse but start with line low
;;; and use short timeout. Use after ds1rec_open_ended
;;; detected start of the reset pulse (hense short timeout)
ds1wait_short:
        movlw   d'40'            ; start 150 us timer
        movwf   tmpbit
        goto    measure_reset_pulse
        
idle_loop:      
        call	clearwdt
        call    actled_off
        call    errled_off
        call    idle_hook
        TEST1WSC
        goto    idle_loop
        return
        
;;-------------------------------------------------------
;; sends presence pulse
        
ds1pres:
        movlw   0xEB            ; wait ~20 us
        movwf   TMR0
        bcf     INTCON,T0IF
        btfss   INTCON,T0IF     
        goto    $-1

        call	owout_line_low  ; dq low
        movlw   0x5F            ; wait ~160 us
        movwf   TMR0
        bcf     INTCON,T0IF
        btfss   INTCON,T0IF     
        goto    $-1
        call    owin
        return
        
;; ****************************************************************
;; Receive one byte. In the end wait for the line to come high
;;; 
;;; This function is used to read command sent by the master after
;;; reset pulse. Problem: master sends reset pulse when SEARCH_ROM sequence
;;; is complete, we detect this pulse in the main loop and proceed waiting
;;; for the command in ds1rec, which never comes. Instead, another reset pulse
;;; comes some time later. This means it is possible that this function
;;; can receive reset pulse instead of a byte of data. If this happens,
;;; return with bit dsstat set in dareset variable.
ds1rec: movlw   0x08
        movwf   bitctr
        bcf     dsstat,dareset
        call    ds1rec1
        TEST1WSS
        goto    $-3
        return

;;; ****************************************************************
;;; Receive one byte.
;;; Do not wait for  the line to become high in the end.
;;; Note that timeout measured while waiting for the line to be released
;;; after each bit still applies. If this fuction gets control after
;;; reset pulse sent after the SEARCH_ROM command is completed, it will
;;; time out after the first attempt to read a bit and return with
;;; bit dsstat set in dareset variable.
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
;;;
ds1rec1:
        movfw   GPIO
        bcf     INTCON,GPIF
        btfss   INTCON,GPIF
        goto    $-1

        ;bsf     GPIO, GPIO2
        
ds1rec2:        
get_bit:        
        ;bsf     GPIO, GPIO5

        call    delay4us         ; 4us
        call    delay4us         ; 4us
        call    delay4us         ; 4us
        TEST1WSC                ; test line, skip if low
        goto    got_1           ; line is high == '1'

        bcf     STATUS,C        ; line is low == '0'
        rrf     indat,f         
        ;bcf     GPIO, GPIO5

        decfsz  bitctr,f        
        goto    ds1rec_wait_high
        return
        
        ;;  wait till line goes high or timeout
ds1rec_wait_high:       
        movlw   0x5             ; 250 us - timeout
        movwf   TMR0
        bcf     INTCON,T0IF
wait_0:
        TEST1WSC                ; test line, skip if low
        goto    ds1rec1
        btfss   INTCON,T0IF     
        goto    wait_0

        ;;  timeout, set error bit and exit
        bsf     dsstat,dareset
        ;bcf     GPIO, GPIO5
        ;bcf     GPIO, GPIO2
        return

got_1:  bsf     STATUS,C        
        rrf     indat,f         
        ;bcf     GPIO, GPIO5

        decfsz  bitctr,f        
        goto    ds1rec1         
        ;bcf     GPIO, GPIO2
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
        call    delay4us         ; 4us
        call    delay4us         ; 4us
        call    delay4us         ; 4us
        
        TEST1WSC
        goto    _rx_got_1      ; line is high == '1'

        bcf     STATUS,C        ; line is low == '0'
        rrf     INDF,f         

        decfsz  bitctr,f        
        goto    _rx_wait_high
        goto    _rx_ds1rec_byte_done

        ;;  wait till line goes high or timeout
_rx_wait_high:
        movlw   0x5             ; 250 us - timeout
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
        
        
        
;;;-----------------------------------------------------
;;; sends 1 bit
;;; bit to be sent is passed via carry
;;; assumes master has sent reset pulse
;;; This function is used only in SEARCH_ROM (macro SR_1_BIT_OP)
;;;
;;; In the end waits for the line to go high with no timeout.
;;; WDT provides protection against situations when line
;;; gets stuck low.
;;; 
ds1wr:  bcf     dsstat,dareset
        movfw   GPIO
        bcf     INTCON,GPIF
        btfss   STATUS,C        
        goto    ds1wr_0        ; send '0'
ds1wr_1:        
        ;; wait for the line to go low
        btfss   INTCON,GPIF
        goto    $-1
        goto    ds1wr_hold
       
;;;----------------------------------------------------
;;; second entry point: send 1 bit, reversed
;;; bit to be sent is passed via carry
;;; assumes master has sent reset pulse
;;; This function is used only in SEARCH_ROM (macro SR_1_BIT_OP)
;;;
ds1wr_r:
        movfw   GPIO
        bcf     INTCON,GPIF
        btfss   STATUS,C        
        goto    ds1wr_1

        ;; sending 0
ds1wr_0:        
        ;; wait for the line to go low
        btfss   INTCON,GPIF
        goto    $-1
        call	owout_line_low

ds1wr_hold:     
        call    delay4us          ; 4us
        call    delay4us
        call    delay4us
        call    delay4us
        call    delay4us          ; 4us
        ;call    delay4us          ; 4us

        call	owin

        TEST1WSS
        goto    $-3
        return

        
;;; -----------------------------------------------------
;;; Send one byte
;;;
;;; Master drives line low to indicate beginning of a slot, however
;;; line stays low for just a few microseconds so TEST1WSC is too slow.
;;; Use interrupt on change bit.
;;; 
ds1sen: movlw   0x08
        movwf   bitctr

        movfw   GPIO
        bcf     INTCON,GPIF

;;; master drives line low and then releases it right away, line stays
;;; low for only about 3us. It then senses the line level to check what
;;; is being transmitted by slave, 0 or 1. It senses the line
;;; within about 10us after it releases the line. Prepare the bit to be
;;; sent before we go into the loop waiting for the beginning of the slot.
;;; Splitting the loop into two parts (for '0' and '1') allows for minimal
;;; delay between detection of the beginning of the slot and setting line
;;; level low to send '0'.

ds1sen1:
        rrf     outdat,f        
        btfsc   STATUS,C        
        goto    ds1sen_1

ds1sen_0:       
        btfss   INTCON,GPIF
        goto    $-1
        call	owout_line_low

ds1sen_hold:    
        call    delay4us          ; 4us
        call    delay4us
        call    delay4us
        call    delay4us

        call	owin            ; does not touch GPIO
                                ; so it does not clear IOC condition

        movfw   GPIO            ; to clear IOC condition
        bcf     INTCON,GPIF
        
        decfsz  bitctr,f
        goto    ds1sen1
        return
                
ds1sen_1:       
        btfss   INTCON,GPIF
        goto    $-1
        goto    ds1sen_hold

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
        movlw   d'100'
        call    long_timer_init
_wait_line_high_long_loop:      
        TEST1WSS
        goto    wait_line_high_long ; again low
        call    long_timer_check
        btfss   INTCON,T0IF     
        goto    _wait_line_high_long_loop
        ;; line is still high and timer rolled
        return

        ;; ----------------------------------------------------------------
        ;; Long duration timer
        ;; interval passed in W is in 4us increments
        ;; Thus can support up to 4*256 = 1024 us intervals
long_timer_init:
        movwf   long_timer_cntr
        movlw   0xFF
        movwf   long_timer_val
        movfw   long_timer_cntr
        subwf   long_timer_val,f
        movlw   4
        movwf   long_timer_cntr

_long_timer_do_init:    
        movfw   long_timer_val
        movwf   TMR0
        bcf     INTCON,T0IF
        return

long_timer_check:
        btfss   INTCON,T0IF
        return
        ;; timer overflowed
        decfsz  long_timer_cntr,f
        goto    _long_timer_do_init
        ;; timer overflowed 4 times. Long timeout has occured
        return
        
        
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
        
        end                             
