
        ;processor p12f675
        
        include p12f675.inc
        errorlevel  -302               ; suppress message 302 from list file

        ;__config (_WDT_OFF & _INTRC_OSC_NOCLKOUT)
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
;;;  GPIO2 - 
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
#define OPTION_BITS	b'00000000' ; assign TMR0 prescaler 1:2 for TMR0,
                                    ; GPIO pull-ups enabled
#define T1CON_BITS      b'00110001' ; TMR1ON, 1:8 prescaler

#define BTN             GPIO0
#define CH1             GPIO1
#define CH2             GPIO2
#define ACTIVITY        GPIO5

#define TMR1_SKIP_CONSTANT D'25'

#define SW_REVISION     b'00100000' ; sw revision 1

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

_delay          RES     1
_pause          RES     1
bcntr           RES     1
tmpbit          RES     1
tmpind          RES     1
tmpb            RES     1
        
REGISTERS       RES     8       ; 8 1-byte registers

bcntr1          RES     1
sec_cntr        RES     1

        ;; tmr1 runs with period of 4 ms so we can reset WDT often enough
        ;; use this skip counter to perform other functions at 0.1 interval
tmr1_skip_counter       RES     1

        ;; use this for 1 sec resolution intervals
tmr1_1_sec_counter      RES     1
        
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

        decfsz  tmr1_skip_counter,f
        goto    restart_tmr1

        bsf     GPIO, ACTIVITY  ; "activity" LED
        
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
        decfsz  register1,f
        goto    r1_on
r1_off: bcf     GPIO, CH1
        goto    r1_done
r1_on:  bsf     GPIO, CH1
        
r1_done:
        ;; register0 (status reg.) is read-only
        ;; reset its contents
        movlw   1
        andwf   register0,f
        movlw   SW_REVISION
        iorwf   register0,f
        ;; bits 1-7 of register0 were cleared above when we wrote
        ;; sw revision in it
        btfsc   GPIO, CH1
        bsf     register0, CH1

restart_tmr1:
        call    tmr1_4_ms

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
        BANK1
        bsf     PIE1, TMR1IE
        BANK0
        bcf     PIR1, TMR1IF
        bsf     INTCON, PEIE
        bsf     INTCON, GIE
        return
        
v1:     bsf     GPIO,CH1
        call    v_pause         ; delay in sec is in W
        bcf     GPIO,CH1
        return

v2:     bsf     GPIO,CH2
        call    v_pause         ; delay in sec is in W
        bcf     GPIO,CH2
        return
        
        ;; ################################################################
        ;; Init
        ;; ################################################################
Init
        ;; only needed for 12F509 ? Not for 675 and 683
;	call    0x3FF      ; retrieve factory calibration value
;	movwf   OSCCAL          ; update register with factory cal value
        
        BANKSEL TRISIO
	movlw	TRISIO_BITS
	movwf	TRISIO
        movlw   WPU_BITS
        movwf   WPU
        movlw   OPTION_BITS
	movwf	OPTION_REG
	clrf	ANSEL		; configure A/D I/O as digital

	BANKSEL CMCON

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
        movwf   bcntr
        clrf    INDF
        incf    FSR,f
        decfsz  bcntr,f
        goto    $-3

        movlw   GPIO4
        call    ds1init

        movlw   TMR1_SKIP_CONSTANT
        movwf   tmr1_skip_counter
        movlw   D'10'
        movwf   tmr1_1_sec_counter

        call    tmr1_4_ms
        
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
        call    ds1rec_open_ended ; disables interrupts
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
        ;; valve to open. The number defines how long (in 0.1 sec)
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

        
        end
        
