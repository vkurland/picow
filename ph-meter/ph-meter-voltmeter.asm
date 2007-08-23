
        include p12f683.inc
        errorlevel  -302               ; suppress message 302 from list file

        ;; Note: code protection is on
        ;; 
        __CONFIG  _WDT_OFF & _PWRTE_OFF & _INTRC_OSC_NOCLKOUT & _MCLRE_OFF & _CPD_ON & _CP_ON

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
;
;  pins:
;  GPIO0 - input: measure voltage 
;  GPIO1 - 
;  GPIO2 - 
;  GPIO3 - 
;  GPIO4 - 
;  GPIO5 - output (sending code to the controller)
;
;******************************************************************************
        
;******************************************************************************
;Defines
;******************************************************************************

#define TRISIO_BITS     B'11111111' ; GPIO  0,1,2,3,4,5: in
#define WPU_BITS        B'00100000' ; gpio5 pull-up on

;; assign TMR0 prescaler 1:2 for TMR0
#define OPTION_BITS	b'00000000'
#define WDTCON_BITS     b'00010110' ; WDT disabled, period 1:65536 (~2.1sec)
#define T1CON_BITS      b'00000001' ; TMR1ON, 1:1 prescaler
#define CCP1CON_BITS    b'00001100' ; DC1B1,DC1B0=0, PWM mode active high

#define VOLTMETER_DQ    GPIO5   ; communication with voltmeter
        
;******************************************************************************
;General Purpose Registers (GPR's) 
;******************************************************************************

MAIN_VARS       UDATA   0x20
tmp2            RES     1
adch            RES     1
adcl            RES     1
        
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
        movlw   b'00010011'     ; Fosc/8, GPIO0 and GPIO1 are analog
        movwf   ANSEL

        BANKSEL ADCON0
        movlw   b'01000001'     ; left justify, using Vref, AN0, ADC on
        movwf   ADCON0
        
        call    adc_sample_time
        bsf     ADCON0,GO       ; start conversion
        btfsc   ADCON0,GO_DONE
        goto    $-1
        ;; ADC data ready
        BANKSEL ADRESH
        movfw   ADRESH
        BANKSEL adch
        movwf   adch
        BANKSEL ADRESL
        movfw   ADRESL
        BANKSEL adcl
        movwf   adcl
        
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
        movlw   b'00010001'     ; Fosc/8, GPIO0 is analog input
        movwf   ANSEL

        BANKSEL WDTCON
        movlw   WDTCON_BITS
        movwf   WDTCON
        
        ;; turn comprator off and configure GPIO 1,2 as digital.
        ;; If GPIO bit is configured as analog, it always
        ;; reads '0' even when TRISIO configures it as output
        BANKSEL CMCON0
        movlw   b'00000000'     ; comparator off, all digital
        movwf   CMCON0

main_loop:
        BANKSEL GPIO
        movlw   VOLTMETER_DQ
        ;; ds1init enables interrupt-on-change - sets IOC and clears GPIF bit
        call    ds1init

        bsf     INTCON,GPIE     ; enable GPIO change interrupt
        sleep
        nop
        bcf     INTCON,GPIE     ; disable Interrupt on GPIO port change

        call    ds1wait
        call    ds1rec
        
        clrf    adch
        clrf    adcl
        
        call    adc
        ;; result is in adch, adcl

        movfw   adch
        movwf   outdat
        call    ds1sen

        ;; do it again, reversed
        movfw   adch
        movwf   outdat
        comf    outdat,f
        call    ds1sen

        movfw   adcl
        movwf   outdat
        call    ds1sen

        ;; do it again, reversed
        movfw   adcl
        movwf   outdat
        comf    outdat,f
        call    ds1sen

        goto    main_loop
        
        end
        
