
        ;processor p12f675
        
        include p12f675.inc
        errorlevel  -302               ; suppress message 302 from list file

        ;__config (_WDT_OFF & _INTRC_OSC_NOCLKOUT)
	__CONFIG  _CP_OFF & _WDT_OFF & _BODEN_ON & _PWRTE_ON & _INTRC_OSC_NOCLKOUT & _MCLRE_OFF & _CPD_OFF

;******************************************************************************
;Defines
;******************************************************************************

#define TRISIO_BITS     B'11001000' ; GPIO 0,1,2,4,5 are outputs
#define CMCON_BITS	B'00000111' ; configure comparator inputs as digital I/O
#define OPTION_BITS	b'10000000' ; assign TMR0 prescaler 1:2 for TMR0
#define T1CON_BITS      b'00110001' ; TMR1ON, 1:8 prescaler

;******************************************************************************
; 1-wiire commands
;******************************************************************************
#define SEARCH_ROM      0xF0
#define COND_SEARCH     0xEC
#define READ_ROM        0x33
#define MATCH_ROM       0x55
#define READ_MEMORY     0xAA
#define SKIP_ROM        0xCC


        
BANK0:  MACRO
	bcf     STATUS,RP0	; change to PORT memory bank
        ENDM

BANK1:  MACRO
	bsf     STATUS,RP0	; change to memory bank 1
        ENDM

        GLOBAL  indicator,inddelay

        EXTERN  dsstat, ds1init, ds1wait, ds1wait_short, ds1sen, ds1sen_detect_reset
        EXTERN  ds1rec, ds1rec_open_ended, ds1rec_detect_reset
        EXTERN  ds1_rx2, ds1_rx3
        EXTERN  indat, indat1, indat2, indat3, outdat
        EXTERN  ds1_search_rom, ds1_match_rom

;******************************************************************************
;General Purpose Registers (GPR's) 
;******************************************************************************

MAIN_VARS       UDATA   0x20
;; temp variables to save state on interrupt entry
WTEMP           RES      1
STATUSTEMP      RES      1
PCLATHTEMP      RES      1
FSRTEMP         RES      1

_delay          RES     1
_pause          RES     1
bcntr           RES     1
tmpbit          RES     1
tmpind          RES     1
tmpb            RES     1
        
REGISTERS       RES     8       ; 8 1-byte registers

bcntr1          RES     1
sec_cntr        RES     1
        
;******************************************************************************
;Reset Vector 
;******************************************************************************
RESET_V	CODE    0x000         	; processor reset vector
	nop			; required by in circuit debugger  
	goto    Init            ; go to beginning of program

;******************************************************************************
;interrupt vector
;******************************************************************************
IRQ_V   CODE    0x004

        movwf   WTEMP           ;Save off current W register contents
        movf    STATUS,w
        clrf    STATUS                  ;Force to page0
        movwf   STATUSTEMP
        movf    PCLATH,w
        movwf   PCLATHTEMP              ;Save PCLATH
        movf    FSR,w
        movwf   FSRTEMP                 ;Save FSR

        bcf     INTCON,GPIF       ; Clear GPIO Interrupt Flag
        btfss   PIR1, TMR1IF
        goto    intext
        bcf     PIR1, TMR1IF

        movf    REGISTERS+1,f
        btfsc   STATUS,Z
        goto    r1_off          ; register1 == 0
        decfsz  REGISTERS+1,f
        goto    r1_on
r1_off: bcf     GPIO, GPIO1
        goto    r2
r1_on:  bsf     GPIO, GPIO1

r2:     movf    REGISTERS+2,f
        btfsc   STATUS,Z
        goto    r2_off          ; register2 == 0
        decfsz  REGISTERS+2,f
        goto    r2_on
r2_off: bcf     GPIO, GPIO2
        goto    intext
r2_on:  bsf     GPIO, GPIO2
        
intext: call    tmr1_one_tenth_sec

        clrf    STATUS            ; Select Bank0
        movf    FSRTEMP,w
        movwf   FSR               ; Restore FSR
        movf    PCLATHTEMP,w
        movwf   PCLATH            ; Restore PCLATH
        movf    STATUSTEMP,w
        movwf   STATUS            ; Restore STATUS
        swapf   WTEMP,f                   
        swapf   WTEMP,w           ; Restore W without corrupting STATUS bits
        retfie
 
;******************************************************************************
;Initialization
;******************************************************************************
MAIN    CODE

        ;; initialize TMR1 for 0.1 sec count
        ;; in the end of which it generates interrupt
        ;; using prescaler 1:8
        ;; timer counts every 8us
        ;; need 100,000us, counting backwards to 0
        ;; 100,000us corresponds to 100000/8=12500 counts
        ;; preload timer counter with 65536-12500 = 53036 = 0xCF2C
        ;; 
tmr1_one_tenth_sec:
        movlw   0xCF
        movwf   TMR1H
        movlw   0x2C
        movwf   TMR1L
        movlw   T1CON_BITS
        movwf   T1CON           ; enable timer and set prescaler to 1:8
        ;; enable interrupt on roll-over
        ;; tmr1 interrupt bit PIE1:0    (bank1)
        ;; PEIE bit INTCON:6            (bank0)
        ;; GIE bit INTCON:7             (bank0)
        ;; clear bit TMR1IF in PIR1
        BANK1
        bsf     PIE1, TMR1IE
        BANK0
        bcf     PIR1, TMR1IF
        bsf     INTCON, PEIE
        bsf     INTCON, GIE
        return
        
v1:     bsf     GPIO,GPIO1
        call    v_pause         ; delay in sec is in W
        bcf     GPIO,GPIO1
        return

v2:     bsf     GPIO,GPIO2
        call    v_pause         ; delay in sec is in W
        bcf     GPIO,GPIO2
        return
        
        ;; ################################################################
        ;; Init
        ;; ################################################################
Init
	call    0x3FF      ; retrieve factory calibration value
        BANK1
	movwf   OSCCAL          ; update register with factory cal value 
	movlw	TRISIO_BITS
	movwf	TRISIO
        movlw   OPTION_BITS
	movwf	OPTION_REG
	clrf	ANSEL		; configure A/D I/O as digital

	BANK0

;;         ;;  interrupts
;;         ;;  GPIO state change interrupt
;;         ;;  first, read from GPIO to clear mismatches
;;         movfw   GPIO
;;         bsf     INTCON,GPIE     ;Interrupt on GPIO port change
;;         bcf     INTCON,GPIF     ;Clear port change Interrupt Flag
;;         bsf     INTCON,GIE      ;Turn on Global Interrupts
        
        movlw   CMCON_BITS
	movwf	CMCON		;
        
	clrf	TMR0
        clrf    TMR1L
        clrf    TMR1H

        bcf     GPIO, GPIO0
        bcf     GPIO, GPIO1     ; valve 1
        bcf     GPIO, GPIO2     ; valve 2

        bcf     GPIO, GPIO5     ; led

        movlw   REGISTERS
        movwf   FSR
        movlw   D'8'
        movwf   bcntr
        clrf    INDF
        incf    FSR,f
        decfsz  bcntr,f
        goto    $-3
        
        call    ds1init
        call    tmr1_one_tenth_sec
        
        goto    main_loop

wait_reset_end:
        call    ds1wait_short
        goto    wait_cmd

main_loop:      
        bsf     INTCON, GIE     ; enable interrupts
        call    ds1wait

wait_cmd:
        call    ds1rec_open_ended
        btfsc   dsstat,1
        goto    wait_reset_end

cmd:    movlw   SEARCH_ROM
        subwf   indat,w
        btfss   STATUS,Z
        goto    mr

        ;; Master issued search ROM command
        call    ds1_search_rom
        ;; we do not support any subcommands after SEARCH_ROM at this time
        goto    main_loop

        
        btfsc   dsstat,1
        goto    main_loop       ; search did not match our address
        ;; Master may issue chip-specific command after SEARCH_ROM

        bsf     GPIO,GPIO5
        call    ds1rec_open_ended
        bcf     GPIO,GPIO5
        
        btfsc   dsstat,1
        goto    wait_reset_end  ; if timeout occured, this is reset, line still low
        ;; Process subcommand
        
        goto    main_loop
        
mr:
;;          movfw   indat
;;          call    indicator
;;          goto    main_loop

        movlw   MATCH_ROM
        subwf   indat,w
        btfss   STATUS,Z
        goto    main_loop
        ;;  Match ROM command
        call    ds1_match_rom
        btfsc   dsstat,1
        goto    main_loop       ; match_rom did not match our address
        
        bsf     GPIO,GPIO5
        ;; Perform operations specific to MATCH_ROM
        call    ds1rec
        bcf     GPIO,GPIO5
        
        movlw   0xF5
        subwf   indat,w
        btfss   STATUS,Z
        goto    reg_write

        ;; Command 0xF5: read content of the register N
        ;; register number follows (1 byte)
        call    ds1rec
        movfw   indat
        addlw   REGISTERS
        movwf   FSR
        movfw   INDF
        movwf   outdat
        call    ds1sen
        goto    main_loop

reg_write:
;;         movfw   indat
;;         call    indicator
;;         goto    main_loop
       
        movlw   0x5A
        subwf   indat,w
        btfss   STATUS,Z
        goto    main_loop       ; illegal command

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

        ; call    open_valve      
        
        goto    main_loop

reg_wr_err:
        movlw   0xA0
        movwf   outdat
        call    ds1sen
        clrf    outdat
        call    ds1sen
        goto    main_loop


        ;; ################################################################
        ;; Valve1 is controlled by register 1
        ;; Valve2 is controlled by register 2
        ;; Writing a number into a register causes corresponding
        ;; valve to open. The number defines how long (in sec)
        ;; it stays open
        
open_valve:
        movf    REGISTERS+1,w
        btfsc   STATUS,Z
        goto    t_v2
        call    v1
        clrf    REGISTERS+1
        return

t_v2:   movf    REGISTERS+2,w
        btfsc   STATUS,Z
        return
        call    v2
        clrf    REGISTERS+2
        return

        ;; pause for W seconds
v_pause: 
        movwf   sec_cntr
v_loop: call    one_sec
        decfsz  sec_cntr,f
        goto    v_loop
        return

        ;;  one second delay
one_sec:
        movlw   D'40'
        movwf   bcntr1
_v_0:   movlw   D'50'
        movwf   bcntr
_v_1:   movlw   5               ; ~500 us
        movwf   TMR0
        bcf     INTCON,T0IF
        btfss   INTCON,T0IF
        goto    $-1
        decfsz  bcntr,f
        goto    _v_1
        decfsz  bcntr1,f
        goto    _v_0
        return

        ;; ################################################################
        ;; send byte passed in W to indicator
        ;; use GPIO0 as strobe and GPIO1 as data line
indicator
        movwf   tmpind
        movlw   D'8'
        movwf   bcntr
        
        bcf     GPIO,GPIO0      ; strobe
        call    inddelay
        call    inddelay
        bsf     GPIO,GPIO0
        
ind1    bcf     GPIO,GPIO1      ; data
        rrf     tmpind,f
        btfsc   STATUS,C
        bsf     GPIO,GPIO1      ; data        
        call    inddelay
        decfsz  bcntr,f
        goto    ind1
        return

inddelay
        movlw   0xFF
        movwf   _delay
dloop   call    _dd
        decfsz  _delay,f
        goto    dloop
_dd     return

pause   movlw   D'100'
        movwf   _pause
ploop   call    inddelay
        decfsz  _pause,f
        goto    ploop
        return
        
        end
        
