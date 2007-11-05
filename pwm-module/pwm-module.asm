
        include p12f683.inc
        errorlevel  -302               ; suppress message 302 from list file

        ;; Note: code protection is on
        ;; 
        __CONFIG  _WDT_ON & _PWRTE_OFF & _INTRC_OSC_NOCLKOUT & _MCLRE_OFF & _CPD_ON & _CP_ON

        CODE
        DA      "Copyright 2007, Vadm Kurland"

        include ds1.inc

 
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
;  GPIO3 - 
;  GPIO4 - 1-wire
;  GPIO5 - "activity" LED
;
;  Controlling PWM output:
;
;  register0 - status and control register. Bits:
;
;    0 - R   0: target code reached; 1: begin PWM sequence change
;    1 - 
;    2 - 
;    3 - 
;    4 - 
;    5 - 
;    6 - 
;    7 - 
;        
;  register1 - PWM pulse 8 MSB
;  register2 - PWM pulse 2 LSB
;
;  register6 - current value of 8 MSB of duty cycle code
;              should be equal to register1
;  register7 - current value of 2 LSB of duty cycle code
;              should be equal to register2
;
;
;  Using WDT to avoid deadlocks. Reset WDT in interrupt routine
;
;
;
;       
;******************************************************************************
        
;******************************************************************************
;Defines
;******************************************************************************

#define TRISIO_BITS     B'11011101' ; GPIO 1,5: out, GPIO 0,3,4: in, 2: PWM
#define WPU_BITS        B'00000000' ; weak pull-ups off
#define OPTION_BITS	b'10000000' ; assign TMR0 prescaler 1:2 for TMR0,
                                    ; GPIO pull-ups disabled
#define T1CON_BITS      b'00110001' ; TMR1ON, 1:8 prescaler
#define CCP1CON_BITS    b'00001100' ; DC1B1,DC1B0=0, PWM mode active high

;******************************************************************************
; pin assignment
;******************************************************************************

#define PWM             GPIO2
#define ACTIVITY        GPIO5

;******************************************************************************
; register0 bits        
;
#define BEGIN_TRANSFER   0
#define JUMP_MODE        1
#define LINEAR_MODE      2
#define FAST_LINEAR_MODE 3
#define SLOW_START_MODE  4
#define TIMER_SPEED_BIT  7
#define ALL_MODES        b'11110'
        
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

bcntr           RES     1        
tmp1            RES     1
tmp2            RES     1
offset          RES     1
skip_counter:   RES     1
delta           RES     1
r1r6neg         RES     1
skip_for_adc    RES     1
start_seq_idx   RES     1
cruizing        RES     1
        
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
        call    tmr1_init

        BANKSEL GPIO
        bcf     GPIO, ACTIVITY    ; "activity" LED

intext:
        clrwdt                    ; clear watchdog timer

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

        ;;  register1 -- PWM duty cycle code, 8 MSB
        ;;  register2 -- PWM duty cycle code, 2 LSB
        ;;
        ;;  register6 -- current value of 8 MSB of duty cycle code
        ;;               should be equal to register1
        ;;  register7 -- current value of 2 LSB of duty cycle code
        ;;               should be equal to register2

run_pwm:        
        movfw   register6
        subwf   register1,w     ; w = register1 - register6
        ;; if register1 == register6 then check register7
        btfsc   STATUS,Z
        goto    check_reg7
        ;; if register1 != register6 then goto change_pwm
        goto    change_pwm

check_reg7:
        movfw   register7
        subwf   register2,w     ; w = register2 - register7
        ;; if register2 == register7 then we are done
        btfsc   STATUS,Z
        return
        ;; if register2 != register7 then goto change_pwm

change_pwm:     
        movfw   register1
        movwf   register6
        movfw   register2
        movwf   register7
        call    pwm_change_duty_cycle
        return

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
        ;; initialize timer1. 
       
tmr1_init:
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
Init:   
        BANKSEL TRISIO
	movlw	TRISIO_BITS
	movwf	TRISIO
        movlw   WPU_BITS
        movwf   WPU
        movlw   OPTION_BITS
	movwf	OPTION_REG

        BANKSEL ANSEL
        movlw   b'00010001'     ; Fosc/8, GPIO0 is analog input
        movwf   ANSEL

        BANKSEL ADCON0
        movlw   b'00000001'     ; left justify, AN0, ADC on
        movwf   ADCON0

	BANKSEL TMR0

;;         ;;  interrupts
;;         ;;  GPIO state change interrupt
;;         ;;  first, read from GPIO to clear mismatches
;;         movfw   GPIO
;;         bsf     INTCON,GPIE     ;Interrupt on GPIO port change
;;         bcf     INTCON,GPIF     ;Clear port change Interrupt Flag
;;         bsf     INTCON,GIE      ;Turn on Global Interrupts
        
        clrf    skip_for_adc
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
_clr_reg_loop:  
        clrf    INDF
        incf    FSR,f
        decfsz  bcntr,f
        goto    _clr_reg_loop

        movlw   GPIO4
        call    ds1init
        
        clrf    register0       ; pwm off, fastest period
        call    tmr1_init
        call    pwm_init

        clrf    offset
        clrf    delta
        clrf    start_seq_idx
        clrf    cruizing
        movlw   1
        movwf   skip_counter
        
        goto    main_loop

wait_reset_end:
        clrwdt                    ; clear watchdog timer
        call    ds1wait_short
        goto    wait_cmd

main_loop:      
        bsf     INTCON, GIE       ; enable all interrupts
        clrwdt                    ; clear watchdog timer
        call    ds1wait

wait_cmd:
        bsf     GPIO,ACTIVITY     ; "activity" led
        call    ds1rec_open_ended
        bcf     GPIO,ACTIVITY     ; "activity" led
        btfsc   dsstat,1
        goto    wait_reset_end

cmd:
        clrwdt                    ; clear watchdog timer
        movlw   SEARCH_ROM
        subwf   indat,w
        btfss   STATUS,Z
        goto    mr

        ;; Master issued search ROM command
        call    ds1_search_rom
        ;; we do not support any subcommands after SEARCH_ROM at this time
        goto    main_loop

mr:     movlw   MATCH_ROM
        subwf   indat,w
        btfss   STATUS,Z
        goto    main_loop
        ;;  Match ROM command
        clrwdt                    ; clear watchdog timer
        call    ds1_match_rom
        btfsc   dsstat,1
        goto    main_loop       ; match_rom did not match our address
        
        clrwdt                    ; clear watchdog timer
        bsf     GPIO,ACTIVITY     ; "activity" led
        ;; Perform operations specific to MATCH_ROM
        call    ds1rec
        bcf     GPIO,ACTIVITY     ; "activity" led
        
        movlw   0xF5
        subwf   indat,w
        btfss   STATUS,Z
        goto    reg_write

        clrwdt                    ; clear watchdog timer
        
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

        clrwdt                    ; clear watchdog timer
        
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
        movfw   register6
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
        call    wait_tmr2

        ;; Enable pwm right away
        call    pwm_enable
        return

wait_tmr2:
        BANKSEL PIR1
        btfss   PIR1,TMR2IF
        goto    $-1
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
        ;; wait till Timer2 overflows - beginning of pwm pulse
        call    wait_tmr2
        
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
   
        end
        
