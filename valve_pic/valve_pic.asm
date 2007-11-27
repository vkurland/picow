
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
;;;  GPIO0 - button (turns channel 1 on/off manually)
;;;  GPIO1 - channel 1
;;;  GPIO2 - led indicator of WDT reset or other errors
;;;  GPIO3 - 
;;;  GPIO4 - 1-wire
;;;  GPIO5 - "activity" LED
;;;
;;;
;;;  Controlling valves:
;;;
;;;  register0 - status register. Bits:
;;;            0 - 0: reg1 time in 0.1 sec; 1: reg1 time in seconds (r/w bit)
;;;            1 - valve status: '1' - open, '0' - closed (r/o bit)
;;;            2 -
;;;            3 -
;;;            4 -
;;;            5,6,7 - sw revision (0 .. 8) (r/o bits)
;;; 
;;;  register1 - valve #1: the value represents time this valve is open
;;;  register2 - 
;;;  register3 - 
;;;  register4 - 
;;;  register5 - 
;;;  register6 - 
;;;  register7 - 
;;;
;;; Bit 0 of register0 defines units of time for the interval defined by
;;; register1. Although this bit of register0 is not overwritten by the program
;;; during normal operation, it does not survive reset initiated by WDT.
;;; Always set this bit by writing '1' or '0' to register0 before setting time
;;; using register1.
;;; 
;;; Using WDT to avoid deadlocks. Test by shorting 1-wire bus several
;;; times until false presence pulse is 'detected', then watch WDT reset
;;; the device.
;;;
;;; 12F683:
;;; Watchdog timer can be configured for ~2sec timeout (separate prescaler)
;;;
;;; 12F675:
;;; watchdog timer runs with prescaler 1:1 because this PIC has one prescaler
;;; shared between tmr0 and wdt. This means 18ms WDT interval.
;;;
;;; Code assumes 18ms WDT timeout regardless of the PIC type.
;;;
;******************************************************************************
        
;******************************************************************************
;Defines
;******************************************************************************

#define TRISIO_BITS     B'11001001' ; GPIO 1,2,4,5 are outputs, GPIO 0,3 -inputs
#define WPU_BITS        B'00000001' ; weak pull-up for GPIO0
#define CMCON_BITS	B'00000111' ; configure comparator inputs as digital I/O
#define T1CON_BITS      b'00110001' ; TMR1ON, 1:8 prescaler

#define BTN             GPIO0
#define CH1             GPIO1
#define ERROR_LED       GPIO2
#define ACTIVITY_LED    GPIO5


#define TMR1_SKIP_CONSTANT D'25'

#define SW_REVISION     b'01000000' ; sw revision 2

BANK0:  MACRO
	bcf     STATUS,RP0	; change to PORT memory bank
        ENDM

BANK1:  MACRO
	bsf     STATUS,RP0	; change to memory bank 1
        ENDM

;******************************************************************************
;General Purpose Registers (GPR's) 
;******************************************************************************

MAIN_VARS       UDATA   0x20
;; temp variables to save state on interrupt entry
WTEMP           RES      1
STATUSTEMP      RES      1
PCLATHTEMP      RES      1
FSRTEMP         RES      1
        
        ;; tmr1 runs with period of 4 ms so we can reset WDT often enough
        ;; use this skip counter to perform other functions at 0.1 interval
tmr1_skip_counter       RES     1

        ;; use this for 1 sec resolution intervals
tmr1_1_sec_counter      RES     1
        
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
	clrf	ANSEL		; configure A/D I/O as digital

	BANKSEL CMCON
        movlw   CMCON_BITS
	movwf	CMCON
        
        BANKSEL TMR0
	clrf	TMR0
        clrf    TMR1L
        clrf    TMR1H

;        bcf     GPIO, BTN
        bcf     GPIO, CH1          ; valve 1

        ;; Inititalize ds1wire-1pin code
        ;; assign activity and error indicator ports
        movlw   GPIO4
        call    ds1init
        clrw                    ; OPTION_REG bits GPPU and INTEDG == 0
        call    set_option_reg_bits
        movlw   ACTIVITY_LED
        call    set_activity_led_port
        movlw   ERROR_LED
        call    set_error_led_port
        
        movlw   TMR1_SKIP_CONSTANT
        movwf   tmr1_skip_counter
        movlw   D'10'
        movwf   tmr1_1_sec_counter

        call    tmr1_4_ms

        call    ds1main         ; loops forever 
        
        ;; ################################################################
        ;; hooks ds1wire-1pin calls
        ;; ################################################################

read_register_hook:
        ;; register0 (status reg.) bits 1-7 are read-only
        ;; reset all bits except for bit 0
        movlw   1
        andwf   register0,f
        movlw   SW_REVISION
        iorwf   register0,f
        ;; bits 1-7 of register0 were cleared above when we wrote
        ;; sw revision in it
        btfsc   GPIO, CH1
        bsf     register0, CH1
        return

write_to_register_hook:
        return

idle_hook:
        BANKSEL PIR1
        btfss   PIR1, TMR1IF
        goto    intext          ; timer has not overflown yet
        bcf     PIR1, TMR1IF

        decfsz  tmr1_skip_counter,f
        goto    restart_tmr1

        call    actled_on
        
        movlw   TMR1_SKIP_CONSTANT
        movwf   tmr1_skip_counter

        ;; check button
        btfss   GPIO,BTN
        ;; user pressed the button, activate channel 1
        goto    r1_on

        ;; do we follow 0.1 or 1.0 sec intervals?
        btfss   register0, 0
        goto    r1
        
        decfsz  tmr1_1_sec_counter,f
        goto    restart_tmr1
        movlw   D'10'
        movwf   tmr1_1_sec_counter

r1:     movf    register1,f
        btfsc   STATUS,Z
        goto    r1_off          ; register1 == 0

r1_on:  bsf     GPIO, CH1
        decf    register1,f
        goto    r1_done

r1_off: bcf     GPIO, CH1
        
r1_done:

restart_tmr1:
        call    tmr1_4_ms
        call	clearwdt

        call    actled_off
        call    errled_off

intext:
        return
        
        end
        
