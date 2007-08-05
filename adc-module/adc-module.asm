
        include p12f683.inc
        errorlevel  -302               ; suppress message 302 from list file

        ;; Note: code protection is on
        ;; 
        __CONFIG  _WDT_OFF & _PWRTE_OFF & _INTRC_OSC_NOCLKOUT & _MCLRE_OFF & _CPD_ON & _CP_ON

        CODE
        DA      "Copyright 2007, Vadm Kurland"
        ;; secret phrase :-)
        DA      "Homer: Homer no function beer well without. "

        include "../ds1wire-1pin.asm"
        
 
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
;  GPIO0 - ADC input 0
;  GPIO1 - ADC input 1
;  GPIO2 - ADC input 2
;  GPIO3 - 
;  GPIO4 - 1-wire 
;  GPIO5 - "activity" LED
;
;  Controlling the adc
;
;  register0 - status and control register. Bits:
;
;    0 -   R   0: adc conversion complete; 1: begin adc conversion
;    1 - 
;    2-3 - R/W ADC channel
;    4 - 
;    5 - 
;    6 - 
;    7 -  A/D conversion result format: 1 - right justigied, 0 - left justified
;        
;  register1 - ADC output1 
;  register2 - ADC output2
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

#define TRISIO_BITS     B'11011111' ; GPIO 1,4,5: out, GPIO 0,1,2,3: in
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

#define ACTIVITY        GPIO5

;******************************************************************************
; register0 bits        
;
#define BEGIN_TRANSFER  0
#define ADC_CHAN        b'1100'
        
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
        
REGISTERS       RES     8       ; 8 1-byte registers

tmp1            RES     1
tmp2            RES     1
        
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
        retfie
                
;******************************************************************************
;Initialization
;******************************************************************************
MAIN    CODE


        ;; ################################################################
        ;; Perform one ADC measurement, channel is in bits 2,3 of register0
        ;; ################################################################
adc:
        btfss   register0,0
        return                  ; bit 0 in reg0 is "go"
        
        movfw   register0
        movwf   tmp1
        ;; bits 2,3 of reg0 is channel #,
        ;; bit 7 is ADFM (result format select bit)
        movlw   b'10001100'
        andwf   tmp1,f
        ;; bits 2-3 in tmp1 represent channel #
        bsf     tmp1,ADON       ; ADC on bit
        movfw   tmp1
        BANKSEL ADCON0
        movwf   ADCON0
        
        call    adc_sample_time
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
        bcf     register0,0        
        return
        

        ;; sampling time approx 40us
adc_sample_time:                
        call    _4us            ; 4us
        call    _4us            ; 4us
        call    _4us            ; 4us
        call    _4us            ; 4us
        call    _4us            ; 4us
        call    _4us            ; 4us
        call    _4us            ; 4us
        call    _4us            ; 4us
        call    _4us            ; 4us
        call    _4us            ; 4us
_4us:   return
        
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
        movlw   b'00010111'     ; Fosc/8, GPIO0,1,2 are analog inputs
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
        
        call    ds1init
        clrf    register0       ; pwm off, fastest period
        ;call    tmr1_init

        goto    main_loop

wait_reset_end:
        call    ds1wait_short
        goto    wait_cmd

main_loop:      
        bcf     GPIO,ACTIVITY     ; "activity" led
        call    ds1wait
        goto    wait_cmd

wait_cmd:
        call    ds1rec_enable_int
        ;call    ds1rec
        btfsc   dsstat,1
        goto    wait_reset_end

cmd:    bsf     GPIO,ACTIVITY     ; "activity" led

        movlw   SEARCH_ROM
        subwf   indat,w
        btfss   STATUS,Z
        goto    mr

        ;; Master issued search ROM command
        call    ds1_search_rom
        ;; we do not support any subcommands after SEARCH_ROM at this time
        ;call    ds1_wait_reset
        goto    main_loop
        
mr:     movlw   MATCH_ROM
        subwf   indat,w
        btfss   STATUS,Z
        goto    main_loop
        ;;  Match ROM command
        call    ds1_match_rom
        btfsc   dsstat,1
        goto    main_loop       ; match_rom did not match our address
        
        ;; Perform operations specific to MATCH_ROM
        call    ds1rec
        
        movlw   0xF5
        subwf   indat,w
        btfss   STATUS,Z
        goto    reg_write

        ;; Command 0xF5: read content of the register N
        ;; register number follows (1 byte)
        call    ds1rec

        call    adc
        
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

        call    adc
        
        goto    main_loop

reg_wr_err:
        movlw   0xA0
        movwf   outdat
        call    ds1sen
        clrf    outdat
        call    ds1sen
        goto    main_loop

   
        end
        
