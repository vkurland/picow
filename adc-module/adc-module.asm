
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
;  GPIO0 - ADC input 0
;  GPIO1 - 
;  GPIO2 - 
;  GPIO3 - 
;  GPIO4 - 1-wire 
;  GPIO5 - "activity" LED
;
;  Controlling the adc
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

#define TRISIO_BITS     B'11011111' ; GPIO 5: out, GPIO 0,1,2,3: in
#define WPU_BITS        B'00000000' ; weak pull-ups off
#define OPTION_BITS	b'10000000' ; assign TMR0 prescaler 1:2 for TMR0,
                                    ; GPIO pull-ups disabled
#define T1CON_BITS      b'00110001' ; TMR1ON, 1:8 prescaler

#define ACTIVITY_LED    GPIO5

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

        movlw   GPIO4
        call    ds1init
        movlw   OPTION_BITS
        call    set_option_reg_bits
        movlw   ACTIVITY_LED
        call    set_activity_led_port

        call    ds1main         ; loops forever 

        ;; ################################################################
        ;; hooks ds1wire-1pin calls
        ;; ################################################################

read_register_hook:
        return

write_to_register_hook:
        return

idle_hook:
        ;; perform measurement
        call    adc
        return
        
        end
        
