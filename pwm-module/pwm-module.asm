
        
        include p12f683.inc
        errorlevel  -302               ; suppress message 302 from list file

        ;; Note: code protection is on
        ;; 
        __CONFIG  _CP_ON & _WDT_OFF & _PWRTE_ON & _INTRC_OSC_NOCLKOUT & _MCLRE_OFF & _CPD_OFF


        include "../ds1wire.asm"
        
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

TRISIO2                      EQU     H'0002'

;******************************************************************************
;
;  pins:
;  GPIO0 - 
;  GPIO1 - 
;  GPIO2 - PWM output
;  GPIO3 - 1-wire signal
;  GPIO4 - to the gate of n-channel MOSFET transistor, D connected to 1-wire
;  GPIO5 - "activity" LED
;
;  Controlling the servo:
;
;  register1 - PWM duty cycle 8 MSB
;        
;  register2 - PWM duty cycle 2 LSB
;        
;  register3 - code that defines timer1 interval
;   1 -> 4096 usec
;   2 -> 40960 usec
;   3 -> 0.1 sec
;        
;  register4 - mode of operation:
;              0 - off, 1 - linear, 2 - slow start/stop
;        
;  register5 - index for the slow start sequence
;        
;  register6 - current value of 8 MSB of duty cycle code
;              should be equal to register1
;        
;  register7 - current value of 2 LSB of duty cycle code
;              should be equal to register2
;
;
;
;
;
;
;       
;******************************************************************************
        
;******************************************************************************
;Defines
;******************************************************************************

#define TRISIO_BITS     B'11001100' ; GPIO 0,1,4,5: out, GPIO 3: in, 2: PWM
#define WPU_BITS        B'00000000' ; weak pull-ups off
#define OPTION_BITS	b'10000000' ; assign TMR0 prescaler 1:2 for TMR0,
                                    ; GPIO pull-ups disabled
#define T1CON_BITS      b'00110001' ; TMR1ON, 1:8 prescaler
#define CCP1CON_BITS    b'00001100' ; DC1B1,DC1B0=0, PWM mode active high

;******************************************************************************
; 1-wire commands
;******************************************************************************
#define SEARCH_ROM      0xF0
#define COND_SEARCH     0xEC
#define READ_ROM        0x33
#define MATCH_ROM       0x55
#define READ_MEMORY     0xAA
#define SKIP_ROM        0xCC

#define PWM             GPIO2
#define ACTIVITY        GPIO5
        
;; BANK0:  MACRO
;; 	bcf     STATUS,RP0	; change to PORT memory bank
;;         ENDM

;; BANK1:  MACRO
;; 	bsf     STATUS,RP0	; change to memory bank 1
;;         ENDM

;;         EXTERN  dsstat, ds1init, ds1wait, ds1wait_short, ds1sen
;;         EXTERN  ds1rec, ds1rec_open_ended, ds1rec_detect_reset, ds1rec_enable_int
;;         EXTERN  ds1_rx2, ds1_rx3
;;         EXTERN  indat, indat1, indat2, indat3, outdat
;;         EXTERN  ds1_search_rom, ds1_match_rom

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

tmp1            RES     1
offset          RES     1
cruizing        RES     1
skip_counter:   RES     1
delta16         RES     1
        
#define register0 REGISTERS
#define register1 REGISTERS+1
#define register2 REGISTERS+2
#define register3 REGISTERS+3
#define register4 REGISTERS+4
#define register5 REGISTERS+5
#define register6 REGISTERS+6
#define register7 REGISTERS+7
        
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
        call    run_pwm
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

        ;; initialize timer1
        ;; W: a code that defines timer1 interval
        ;; 
        ;; 1 -> 4096 usec
        ;; 2 -> 40960 usec
        ;; 3 -> 0.1 sec
        ;;
        ;; *** Keep this in this code block to ensure
        ;; it ends up in the lower part of address space. This
        ;; simplifies calculated jump code.
tmr1_init:
        addwf   PCL,f
        retlw   0
        goto    tmr1_4096_usec
        goto    tmr1_40960_usec
        goto    tmr1_one_tenth_sec

        
        
;******************************************************************************
;Initialization
;******************************************************************************
MAIN    CODE

        ;;  register1 -- PWM duty cycle code, 8 MSB
        ;;  register2 -- PWM duty cycle code, 2 LSB
        ;;  register3 -- code that defines timer1 interval
        ;;  register4 -- mode of operation:
        ;;               0 - off, 1 - linear, 2 - slow start/stop
        ;;
        ;;  register5 -- index for the slow start sequence
        ;;  register6 -- current value of 8 MSB of duty cycle code
        ;;               should be equal to register1
        ;;  register7 -- current value of 2 LSB of duty cycle code
        ;;               should be equal to register2
        
run_pwm:        
        movf    register4,f
        btfsc   STATUS,Z
        goto    r4_0
        call    pwm_enable      ; register4 != 0
        goto    _cont_pwm_run
r4_0:   
        call    pwm_disable     ; register4 == 0
        goto    _ext_pwm_run

_cont_pwm_run:
        call    adjust_duty_cycle_code
        btfsc   STATUS,C
        call    pwm_change_duty_cycle
_ext_pwm_run:   
        movfw   register3
        call    tmr1_init
        return

        ;; increment or decrement code in reg6/reg7
        ;; Eventually register6 should become equal to register1
        ;; and register7 to register2
        ;; Return with bit C set if we need to push updated
        ;; duty cycle code to PWM module
        ;; Keeping this funtion in the main code block
        ;; to minimize amount of memory used in IRQ service routine
        
adjust_duty_cycle_code:
        clrf    delta16
        movfw   register6
        subwf   register1,w     ; w = register1 - register6
        ;; if register1 == register6 then check register7
        btfsc   STATUS,Z
        goto    check_reg7
        movwf   delta16
        ;; if register1 > register6 then goto incr_duty_cycle_code
        btfss   STATUS,C
        goto    decr_duty_cycle_code
        goto    incr_duty_cycle_code

check_reg7:
        movfw   register7
        subwf   register2,w     ; w = register2 - register7
        ;; if register2 == register7 then we are done
        btfsc   STATUS,Z
        goto    no_change
        ;; if register2 > register7 then goto incr_duty_cycle_code
        btfss   STATUS,C
        goto    decr_duty_cycle_code
        
incr_duty_cycle_code:
        decfsz  skip_counter,f
        return
        call    get_skip_counter
        movwf   skip_counter
        
        incf    register7,f
        btfss   register7,2
        goto    return_with_change
        ;; register7 > 3
        clrf    register7
        incf    register6,f
        goto    return_with_change

decr_duty_cycle_code:
        ;; register1 < register6
        ;; delta16 = -delta16
        comf    delta16,f
        incf    delta16,f
        decfsz  skip_counter,f
        return
        call    get_skip_counter
        movwf   skip_counter
        
        decf    register7,f   ; register7 = register7 - 1
        btfss   register7,7
        goto    return_with_change
        ;; register7 < 0
        movlw   3
        movwf   register7
        decf    register6,f
        
return_with_change:     
        bsf     STATUS,C
        return

no_change:
        ;; r1,2 == r6,7 - servo is in requested position
        ;; clear slow start sequence counter to prepare for
        ;; the next move
        clrf    register5
        clrf    cruizing
        clrf    delta16
        clrf    offset
        bcf     STATUS,C
        return

get_skip_counter:
        btfss   register4,0
        goto    $+2
        retlw   1               ; reg4:0 == 1, always use skip 1
        ;; check bit 1
        btfss   register4,1
        retlw   1               ; unknown bits in reg4
        ;; reg4:1 == 1, slow start/stop
        movfw   register5
        btfsc   cruizing,0
        ;; delta16 = abs(register1 - register6)
        ;; if we are cruizing, use delta16 as an index
        ;; is slow start/stop table. Most of the table is
        ;; populated with '1' so if we are far from the end of the transfer,
        ;; we get '1' from it. But as we get close to the target, we
        ;; get other values that make us break slowly
        movfw   delta16
        call    get_seq_code
        ;; important !!! PCLATH is used in get_seq_code because it
        ;; is located in the page 3
        clrf    PCLATH
        btfsc   cruizing,0
        return             ; no need to increment reg5 if already cruizing
        
        movwf   tmp1
        decf    tmp1,f
        btfsc   STATUS,Z
        ;; got 1, which means no skipping - cruizing mode
        bsf     cruizing,0
        incf    register5,f
        return                  ; code is in W
 
        ;; -------------------------------------------------------------
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
tmr1_start:     
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
        
        ;; initialize TMR1 for 4.096 msec count
        ;; in the end of which it generates interrupt
        ;; using prescaler 1:8
        ;; timer counts every 8us
        ;; need 4096us, counting backwards to 0
        ;; 4096us corresponds to 4096/8=512 counts
        ;; preload timer counter with 65536-512 = 65024 = 0xFE00
        ;; 
tmr1_4096_usec:
        movlw   0xFE
        movwf   TMR1H
        movlw   0
        movwf   TMR1L
        goto    tmr1_start
        
        ;; initialize TMR1 for 40.96 msec count
        ;; in the end of which it generates interrupt
        ;; using prescaler 1:8
        ;; timer counts every 8us
        ;; need 40960us, counting backwards to 0
        ;; 40960us corresponds to 40960/8=5120 counts
        ;; preload timer counter with 65536-5120 = 60416 = 0xEC00
        ;; 
tmr1_40960_usec:
        movlw   0xEC
        movwf   TMR1H
        movlw   0
        movwf   TMR1L
        goto    tmr1_start
        
        ;; ################################################################
        ;; Init
        ;; ################################################################
Init
        BANKSEL TRISIO
	movlw	TRISIO_BITS
	movwf	TRISIO
        movlw   WPU_BITS
        movwf   WPU
        movlw   OPTION_BITS
	movwf	OPTION_REG
	clrf	ANSEL		; configure A/D I/O as digital

	BANKSEL TMR0

;;         ;;  interrupts
;;         ;;  GPIO state change interrupt
;;         ;;  first, read from GPIO to clear mismatches
;;         movfw   GPIO
;;         bsf     INTCON,GPIE     ;Interrupt on GPIO port change
;;         bcf     INTCON,GPIF     ;Clear port change Interrupt Flag
;;         bsf     INTCON,GIE      ;Turn on Global Interrupts
        
	clrf	TMR0
        clrf    TMR1L
        clrf    TMR1H

        bcf     GPIO, PWM
        bcf     GPIO, ACTIVITY     ; "activity" led

        ;; clear all registers
        movlw   REGISTERS
        movwf   FSR
        movlw   D'8'
        movwf   bcntr
        clrf    INDF
        incf    FSR,f
        decfsz  bcntr,f
        goto    $-3
        
        call    ds1init
        movlw   1                 ; start with fastest speed
        movwf   register3
        call    tmr1_init
        call    pwm_init

        clrf    offset
        clrf    cruizing
        clrf    delta16
        clrf    skip_counter
        
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

        goto    main_loop

reg_wr_err:
        movlw   0xA0
        movwf   outdat
        call    ds1sen
        clrf    outdat
        call    ds1sen
        goto    main_loop

pwm_init:
        ;; 1. Disable CCP1 pin by clearing TRIS bit
        call    pwm_disable
        
        ;; 2. set PWM period by loading PR2 register
        ;; 
        ;; according to the formula (page 79 of data sheet)
        ;; if oscillator frequency is 4MHz and using prescaler 4, we
	;; need to load PR2 with D'203' to get 1.2KHz PWM freq.
        ;;
        ;; For lowest PWM frequency load PR2 with 255 and use
        ;; prescaler 16. This corrsponds to 245Hz (period 4096us)
        ;; With the latter setup servo reacts to the range of
        ;; duty cycle codes 30-150, which corresponds to the range
        ;; of pulse width 0.4ms - 2.4ms
        
        BANKSEL PR2
        movlw   D'255'
        movwf   PR2

        ;; 3. configure CCP module for PWM mode by loading
        ;; the CCP1CON register with the approp. val.
        BANKSEL CCP1CON
        movlw   CCP1CON_BITS
        movwf   CCP1CON

        ;; 4. setthe PWM duty cycle by loading the CCPR1L
        ;; register and DC1B bits of the CCP1CON register
        BANKSEL CCPR1L
        movfw   register1
        movwf   CCPR1L

        ;; 5. Configure and start Timer2:
        ;;  • Clear the TMR2IF interrupt flag bit of the 
        ;;    PIR1 register.          
        ;;  • Set the Timer2 prescale value by loading the
        ;;    T2CKPS bits of the T2CON register. 
        ;;  • Enable Timer2 by setting the TMR2ON bit of 
        ;;    the T2CON register.
        BANKSEL PIR1
        bcf     PIR1,TMR2IF
        BANKSEL T2CON
        movlw   b'10'           ; prescaler 4 : b'01'; 16 : b'10'
        movwf   T2CON
        bsf     T2CON,TMR2ON    ; tmr2 on

        ;; 6. Enable PWM output after a new PWM cycle has 
        ;;    started: 
        ;;  • Wait until Timer2 overflows (TMR2IF bit of 
        ;;    the PIR1 register is set). 
        ;;  • Enable the CCP1 pin output driver by 
        ;;    clearing the associated TRIS bit.
        BANKSEL PIR1
        btfss   PIR1,TMR2IF
        goto    $-1

        ;; do not enable pwm at the beginning, wait for the user
        ;; to set bit 0 or 1 in register4 instead
        ;call    pwm_enable
        return

pwm_enable:
        BANKSEL TRISIO
        bcf     TRISIO, TRISIO2
        BANKSEL GPIO
        return
        
pwm_disable:
        BANKSEL TRISIO
        bsf     TRISIO, TRISIO2
        BANKSEL GPIO
        return
        
        ;; Change PWM duty cycle to the code in register7 and register6
        ;;  register7 holds 8 MSB and register6 holds 2 LSB
        ;; 
pwm_change_duty_cycle:  
        BANKSEL CCPR1L
        movfw   register6
        movwf   CCPR1L

        BANKSEL CCP1CON

        bcf     CCP1CON,DC1B0
        btfsc   register7,0
        bsf     CCP1CON,DC1B0

        bcf     CCP1CON,DC1B1
        btfsc   register7,1
        bsf     CCP1CON,DC1B1
        
        BANKSEL GPIO
        return

        ;; #############################################################
TBL1_PAGE   CODE    0x300

        ;; sequence for slow start
        ;; index is in W on entry
get_seq_code:
        movwf   offset
        movlw   HIGH seq_code_tbl
        movwf   PCLATH
        movfw   offset
        addwf   PCL,f
seq_code_tbl:   
        retlw   5
        retlw   5
        retlw   5
        retlw   5
        retlw   5
        retlw   4
        retlw   5
        retlw   4
        retlw   5
        retlw   4
        retlw   4
        retlw   4
        retlw   3
        retlw   4
        retlw   3
        retlw   3
        retlw   3
        retlw   2
        retlw   3
        retlw   2
        retlw   3
        retlw   2
        retlw   2
        retlw   2
        retlw   2
        retlw   2
        retlw   1
        retlw   2
        retlw   1
        retlw   2
        retlw   1
        retlw   2
        retlw   1
        retlw   2
        retlw   1
        retlw   2
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        retlw   1
        

        
        end
        
