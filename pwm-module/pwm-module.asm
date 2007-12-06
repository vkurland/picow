
        include p12f683.inc
        errorlevel  -302               ; suppress message 302 from list file

        ;; Note: code protection is on
        ;; 
        __CONFIG  _WDT_OFF & _PWRTE_OFF & _INTRC_OSC_NOCLKOUT & _MCLRE_OFF & _CPD_ON & _CP_ON

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
;  GPIO0 - button
;  GPIO1 - 
;  GPIO2 - PWM output
;  GPIO3 - 
;  GPIO4 - 1-wire
;  GPIO5 - "activity" LED
;
;  Controlling PWM output:
;
;  register0 - status and control register. Not in use.
;  register1 - PWM pulse 8 MSB
;  register2 - PWM pulse 2 LSB
;
;  register6 - current value of 8 MSB of duty cycle code
;              should be equal to register1
;  register7 - current value of 2 LSB of duty cycle code
;              should be equal to register2
;
;******************************************************************************
        
;******************************************************************************
;Defines
;******************************************************************************

#define TRISIO_BITS     B'11011101' ; GPIO 1,5: out, GPIO 0,3,4: in, 2: PWM
#define WPU_BITS        B'00000001' ; weak pull-up for GPIO0 (button)
#define T1CON_BITS      b'00110001' ; TMR1ON, 1:8 prescaler
#define CCP1CON_BITS    b'00001100' ; DC1B1,DC1B0=0, PWM mode active high
#define CMCON0_BITS     b'00000111' ; pins are I/O, comparator is off.
;******************************************************************************
; pin assignment
;******************************************************************************

#define BTN             GPIO0
#define PWM             GPIO2
#define ACTIVITY        GPIO5

;******************************************************************************
;General Purpose Registers (GPR's) 
;******************************************************************************

MAIN_VARS       UDATA   0x20

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
        retfie

;******************************************************************************
;Initialization
;******************************************************************************
MAIN    CODE

        ;; -------------------------------------------------------------
        ;; initialize TMR1 for 0.1 sec count
        ;; in the end of which it generates interrupt
        ;; using prescaler 1:8
        ;; timer counts every 8us
        ;; need 100,000us, counting backwards to 0
        ;; 100,000us corresponds to 100000/8=12500 counts
        ;; preload timer counter with 65536-12500 = 53036 = 0xCF2C
        ;; 
tmr1_init:
tmr1_one_tenth_sec:
        movlw   0xCF
        movwf   TMR1H
        movlw   0x2C
        movwf   TMR1L
tmr1_start:     
        movlw   T1CON_BITS
        movwf   T1CON           ; enable timer and set prescaler to 1:8
        BANKSEL PIR1
        bcf     PIR1, TMR1IF
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
        bcf     OPTION_REG, 7   ; enable WPU
	clrf	ANSEL		; configure A/D I/O as digital

        ;; must initialize CMCON0 register because button is
        ;; connected to GPIO0 which is a comparator input.
        BANKSEL CMCON0
        movlw   CMCON0_BITS
        movwf   CMCON0

	BANKSEL TMR0
	clrf	TMR0
        clrf    TMR1L
        clrf    TMR1H

        bcf     GPIO, PWM
        bcf     GPIO, ACTIVITY     ; "activity" led

        movlw   GPIO4
        call    ds1init
        movlw   ACTIVITY
        call    set_activity_led_port
        
        call    tmr1_init
        call    pwm_init

        call    ds1main         ; loops forever 

        ;; ################################################################
        ;; hooks ds1wire-1pin calls
        ;; ################################################################

read_register_hook:
        return

write_to_register_hook:
        return

idle_hook:
        BANKSEL PIR1
        btfss   PIR1, TMR1IF
        goto    intext          ; timer has not overflown yet
        bcf     PIR1, TMR1IF

        call    actled_on
        call    change_pwm
        
        ;; check button
        btfss   GPIO, BTN
        ;; user pressed the button, switch to calibration mode
        call    full_scale_pwm

        call    tmr1_init
intext:
        BANKSEL GPIO
        return
 

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
   
change_pwm:     
        movfw   register1
        movwf   register6
        movfw   register2
        movwf   register7
        call    pwm_change_duty_cycle
        return

full_scale_pwm:     
        movlw   0xFF
        movwf   register6
        movlw   0x02
        movwf   register7
        call    pwm_change_duty_cycle
        return

        
        end
        
