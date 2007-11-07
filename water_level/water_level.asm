
        include p12f675.inc
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
;  GPIO0 - ADC input (wl switches)
;  GPIO1 - 
;  GPIO2 - 
;  GPIO3 - 
;  GPIO4 - 1-wire
;  GPIO5 - "activity" LED
;
;  Controlling PWM output:
;
;  register0 - status and control register. Bits:
;
;    0 - 
;    1 - 
;    2 - 
;    3 - 
;    4 - 
;    5 - 
;    6 - 
;    7 - 
;        
;  register1 - water level sensor voltage
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

#define TRISIO_BITS     B'11011111' ; All inputs, GPIO 5 output
#define WPU_BITS        B'00000000' ; weak pull-ups off
#define OPTION_BITS	b'10000000' ; assign TMR0 prescaler 1:2 for TMR0,
                                    ; GPIO pull-ups disabled
#define T1CON_BITS      b'00110001' ; TMR1ON, 1:8 prescaler

;******************************************************************************
; pin assignment
;******************************************************************************

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

        ;; perform measurement
        call    adc

        call    tmr1_init
        
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
tmr1_4_ms:      
        movlw   0xFE
        movwf   TMR1H
        movlw   0x0C
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
        ;; Perform one ADC measurement
        ;; ################################################################
adc:
        BANKSEL TRISIO
        bsf     TRISIO,GPIO0
        BANKSEL ANSEL
        movlw   b'00010001'     ; Fosc/8, GPIO0  analog
        movwf   ANSEL

        BANKSEL ADCON0
        movlw   b'00000001'     ; left justify, using Vdd, AN0, ADC on
        movwf   ADCON0
        
        call    adc_sample_time
        bsf     ADCON0,GO       ; start conversion
        btfsc   ADCON0,GO_DONE
        goto    $-1
        ;; ADC data ready
        BANKSEL ADRESH
        movfw   ADRESH
        BANKSEL register1
        movwf   register1
        BANKSEL ADRESL
        movfw   ADRESL
        BANKSEL register2
        movwf   register2
        
_4us:   return
        

adc_sample_time:
        call    _4us
        call    _4us
        return

       
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

        ;; turn comprator off and configure GPIO 1,2 as digital.
        ;; If GPIO bit is configured as analog, it always
        ;; reads '0' even when TRISIO configures it as output
        BANKSEL CMCON
        movlw   b'00000000'     ; comparator off, all digital
        movwf   CMCON

	BANKSEL TMR0

	clrf	TMR0
        clrf    TMR1L
        clrf    TMR1H

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

        call    tmr1_init

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

   
        end
        
