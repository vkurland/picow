
        include p12f675.inc
        errorlevel  -302               ; suppress message 302 from list file

        ;__config (_WDT_OFF & _INTRC_OSC_NOCLKOUT)
	__CONFIG  _CP_OFF & _WDT_OFF & _BODEN_ON & _PWRTE_ON & _INTRC_OSC_NOCLKOUT & _MCLRE_OFF & _CPD_OFF


        include "../ds1wire.asm"
        

;******************************************************************************
;
;  pins:
;  GPIO0 - button (turns channel 1 on/off manually)
;  GPIO1 - channel 1
;  GPIO2 - channel 2
;  GPIO3 - 1-wire signal
;  GPIO4 - to the gate of n-channel MOSFET transistor, D connected to 1-wire
;  GPIO5 - "activity" LED
;******************************************************************************
        
;******************************************************************************
;Defines
;******************************************************************************

#define TRISIO_BITS     B'11001001' ; GPIO 1,2,4,5 are outputs, GPIO 0,3 - inputs
#define WPU_BITS        B'00000001' ; weak pull-up for GPIO0
#define CMCON_BITS	B'00000111' ; configure comparator inputs as digital I/O
#define OPTION_BITS	b'00000000' ; assign TMR0 prescaler 1:2 for TMR0,
                                    ; GPIO pull-ups enabled
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

#define BTN             GPIO0
#define CH1             GPIO1
#define CH2             GPIO2
#define ACTIVITY        GPIO5
        

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

        bcf     INTCON,GPIF     ; Clear GPIO Interrupt Flag
        btfss   PIR1, TMR1IF
        goto    intext          ; not tmr1 interrupt
        bcf     PIR1, TMR1IF

        bsf     GPIO, ACTIVITY  ; "activity" LED

        ;; check button
        btfss   GPIO,BTN
        ;; user pressed the button, activate channel 1
        goto    r1_on
        
r1:     movf    REGISTERS+1,f
        btfsc   STATUS,Z
        goto    r1_off          ; register1 == 0
        decfsz  REGISTERS+1,f
        goto    r1_on
r1_off: bcf     GPIO, CH1
        goto    r2
r1_on:  bsf     GPIO, CH1

r2:     movf    REGISTERS+2,f
        btfsc   STATUS,Z
        goto    r2_off          ; register2 == 0
        decfsz  REGISTERS+2,f
        goto    r2_on
r2_off: bcf     GPIO, CH2
        goto    restart_tmr1
r2_on:  bsf     GPIO, CH2
      
restart_tmr1:
        call    tmr1_one_tenth_sec

        bcf     GPIO, ACTIVITY       ; "activity" LED
        
intext:
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
        BANKSEL PIE1
        bsf     PIE1, TMR1IE
        BANKSEL PIR1
        bcf     PIR1, TMR1IF
        bsf     INTCON, PEIE
        bsf     INTCON, GIE
        return
        
        ;; ################################################################
        ;; Init
        ;; ################################################################
Init
	call    0x3FF      ; retrieve factory calibration value
        BANKSEL OSCCAL
	movwf   OSCCAL          ; update register with factory cal value 
	movlw	TRISIO_BITS
	movwf	TRISIO
        movlw   WPU_BITS
        movwf   WPU
        movlw   OPTION_BITS
	movwf	OPTION_REG
	clrf	ANSEL		; configure A/D I/O as digital

	BANKSEL GPIO

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

;        bcf     GPIO, BTN
        bcf     GPIO, CH1     ; valve 1
        bcf     GPIO, CH2     ; valve 2

        bcf     GPIO, ACTIVITY     ; "activity" led

        movlw   REGISTERS
        movwf   FSR
        movlw   D'8'
        movwf   bcntr1
        clrf    INDF
        incf    FSR,f
        decfsz  bcntr1,f
        goto    $-3
        
        call    ds1init
        call    tmr1_one_tenth_sec
        
        goto    main_loop

wait_reset_end:
        call    ds1wait_short
        goto    wait_cmd

main_loop:      
        call    ds1wait
        goto    wait_cmd

        bsf     GPIO,ACTIVITY     ; "activity" led
        call    ds1rec_open_ended
        bcf     GPIO,ACTIVITY     ; "activity" led
        btfsc   dsstat,1
        goto    wait_reset_end

wait_cmd:
        bsf     GPIO,ACTIVITY     ; "activity" led
        call    ds1rec_enable_int
        bcf     GPIO,ACTIVITY     ; "activity" led
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

        bsf     GPIO,ACTIVITY     ; "activity" led
        call    ds1rec_open_ended
        bcf     GPIO,ACTIVITY     ; "activity" led
        
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
        
        bsf     GPIO,ACTIVITY     ; "activity" led
        ;; Perform operations specific to MATCH_ROM
        call    ds1rec
        bcf     GPIO,ACTIVITY     ; "activity" led
        
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




        
        end
        
