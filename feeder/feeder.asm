
        ;processor p12f675
        
        include p12f675.inc
        errorlevel  -302               ; suppress message 302 from list file

        __CONFIG  _WDT_ON & _PWRTE_OFF & _INTRC_OSC_NOCLKOUT & _MCLRE_OFF & _CPD_ON & _CP_ON

TOP     CODE
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
;;;
;;;  pins:
;;;  GPIO0 - battery voltage metering (via divider 1:2)
;;;  GPIO1 - voltage reference for ADC (2.5V MCP1525)
;;;  GPIO2 - motor control
;;;  GPIO3 - 
;;;  GPIO4 - 1-wire
;;;  GPIO5 - "activity" LED
;;;
;;;
;;;  Controlling valves:
;;;
;;;  register0 - status register. Bits:
;;;            0 - 1: perform feeding cycle
;;;            1 - 
;;;            2 -
;;;            3 -
;;;            4 -
;;;            5,6,7 - sw revision (0 .. 8) (r/o bits)
;;; 
;;;  register1 - adc high bits
;;;  register2 - adc low bits
;;;  register3 - 
;;;  register4 - 
;;;  register5 - 
;;;  register6 - 
;;;  register7 - 
;;;
;******************************************************************************
        
;******************************************************************************
;Defines
;******************************************************************************

#define TRISIO_BITS     B'11001011' ; GPIO 2,4,5 are outputs, GPIO 0,1,3 -inputs
#define WPU_BITS        B'00000000' ; weak pull-up off
#define CMCON_BITS	B'00000111' ; configure comparator inputs as digital I/O
#define T1CON_BITS      b'00110001' ; TMR1ON, 1:8 prescaler
#define ADCON0_BITS     b'11000001' ; right justified, external reference
#define ANSEL_BITS      b'00010011' ; Fosc/8, GPIO0,1 are analog inputs

#define MOTOR           GPIO2
#define ACTIVITY_LED    GPIO5

#define SW_REVISION     b'01000000' ; sw revision 2

;******************************************************************************
;General Purpose Registers (GPR's) 
;******************************************************************************

MAIN_VARS       UDATA   0x20

tmr1_skip_counter       RES     1
motor_on_counter        RES     1
        
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
       
        ;; initialize TMR1 for 4.0 msec count
        ;; in the end of which it generates interrupt
        ;; using prescaler 1:8
        ;; timer counts every 8us
        ;; need 4000us, counting backwards to 0
        ;; 4000us corresponds to 4000/8=500 counts
        ;; preload timer counter with 65536-500 = 65036 = 0xFE0C
        ;; 
tmr1_4_ms:
        movlw   0xFE
        movwf   TMR1H
        movlw   0x0C
        movwf   TMR1L
        goto    tmr1_start
        
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
        ;BANK1
        ;bsf     PIE1, TMR1IE
        ;BANK0
        ;bcf     PIR1, TMR1IF
        ;bsf     INTCON, PEIE
        ;bsf     INTCON, GIE

        BANKSEL PIR1
        bcf     PIR1, TMR1IF

        return
        
        ;; ################################################################
        ;; Init
        ;; ################################################################
Init
        ;; only needed for 12F509 ? Not for 675 and 683
;	call    0x3FF      ; retrieve factory calibration value
;	movwf   OSCCAL          ; update register with factory cal value

        ;; Note that OPTION_REG is initialized in ds1init
        
        BANKSEL TRISIO
	movlw	TRISIO_BITS
	movwf	TRISIO
        movlw   WPU_BITS
        movwf   WPU

        movlw   ANSEL_BITS
        BANKSEL ANSEL
	movwf	ANSEL		; configure A/D I/O as digital

        movlw   ADCON0_BITS
        BANKSEL ADCON0
        movwf   ADCON0
        
	BANKSEL CMCON
        movlw   CMCON_BITS
	movwf	CMCON
        
        BANKSEL TMR0
	clrf	TMR0
        clrf    TMR1L
        clrf    TMR1H

        bcf     GPIO, MOTOR

        clrf    motor_on_counter

        ;; Inititalize ds1wire-1pin code
        ;; assign activity and error indicator ports
        movlw   GPIO4
        call    ds1init
        clrw                    ; OPTION_REG bits GPPU and INTEDG == 0
        call    set_option_reg_bits
        movlw   ACTIVITY_LED
        call    set_activity_led_port

        movlw   d'20'
        movwf   tmr1_skip_counter ; to make LED flash every 2 seconds
        call    tmr1_one_tenth_sec

        call    ds1main         ; loops forever 
        
        ;; ################################################################
        ;; hooks ds1wire-1pin calls
        ;; ################################################################

read_register_hook:
        ;; register0 (status reg.) bits 1-7 are read-only
        ;; reset all bits except for bit 0
        movlw   SW_REVISION
        movwf   register0
        bcf     register0, 0
        btfsc   GPIO, MOTOR
        bsf     register0, 0
        return

write_to_register_hook:
        btfss   register0, 0
        return
        movlw   d'10'
        movwf   motor_on_counter
        return

idle_hook:
        BANKSEL PIR1
        btfss   PIR1, TMR1IF
        return                    ; timer has not overflown yet
        bcf     PIR1, TMR1IF

        decfsz  tmr1_skip_counter,f
        goto    motor
        movlw   d'20'
        movwf   tmr1_skip_counter ; to make LED flash every 2 sec
        call    actled_on

        ;; also make adc measurement every 2 sec
        call    adc
        
motor:
        movf    motor_on_counter,f
        btfsc   STATUS,Z
        goto    motor_off          ; motor_on_counter == 0

motor_on:
        bsf     GPIO, MOTOR
        decf    motor_on_counter,f
        goto    motor_done

motor_off:
        bcf     GPIO, MOTOR
        
motor_done:

restart_tmr1:
        call    tmr1_one_tenth_sec
        return

        ;; ################################################################
        ;; Perform one ADC measurement, channel 0
        ;; ################################################################
adc:    BANKSEL ADCON0
        movlw   ADCON0_BITS
        movwf   ADCON0          ; turn adc on
        call    delay40us       ; acquisition pause
        bsf     ADCON0,GO       ; start conversion
_wait_adc:      
        btfsc   ADCON0,GO_DONE
        goto    _wait_adc
        ;; ADC data ready
        BANKSEL ADRESH
        movfw   ADRESH
        BANKSEL GPIO
        movwf   register1
        BANKSEL ADRESL
        movfw   ADRESL
        BANKSEL GPIO
        movwf   register2
        ;; turn adc off
        BANKSEL ADCON0
        clrf    ADCON0
        BANKSEL GPIO
        return

delay40us:      
        call    delay4us
        call    delay4us
        call    delay4us
        call    delay4us
        call    delay4us
        call    delay4us
        call    delay4us
        call    delay4us
        call    delay4us
        call    delay4us
delay4us:       
        return

        
        end
        
