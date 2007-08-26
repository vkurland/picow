
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
;  GPIO0 - input: measure voltage on the "power" capacitor
;  GPIO1 - output: "power" capacitor
;  GPIO2 - power to ADUM5241
;  GPIO3 - 
;  GPIO4 - 1-wire in/out
;  GPIO5 - input from voltmeter
;
;  Controlling the servo:
;
;  register0 - status and control register. Bits:
;
;    0 - 
;    1 - M_GO   : 1: perform measurement, 0: measurement complete
;    2 - M_ON   : 1: power transfer is on, measurement reading is enabled
;    3 - 
;    4 - 
;    5 - 
;    6 - 
;    7 - 
;        
;  register1 - 
;  register2 - result: ADCH
;  register3 - result: ADCL
;  register4 - if '1', then there was an error reading data from voltmeter
;  register5 - 
;  register6 - 
;  register7 - 
;              
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

#define TRISIO_BITS     B'11111011' ; all bits: in, GPIO2 - out
#define WPU_BITS        B'00000000' ; weak pull-ups off
        
;; assign TMR0 prescaler 1:2 for TMR0, GPIO pull-ups enabled
;; TMR0 is used in ds1wire code, prescaler 1:2 is a requirement
#define OPTION_BITS	b'0000000'

#define T1CON_BITS      b'00110001' ; TMR1ON, 1:8 prescaler
#define CCP1CON_BITS    b'00001100' ; DC1B1,DC1B0=0, PWM mode active high

#define ACTIVITY        GPIO2
        ;; 1-wire bus
#define DQ              GPIO1
        ;; communication with voltmeter via second 1-wire bus
        ;; (requires bi-directional galvanic isolator)
#define VOLTMETER_DQ    GPIO5
        ;; communication with voltmeter via simplified 2-wire protocol
        ;; (each wire is unidirectional)
#define VOLTMETER_IO    GPIO5
#define VOLTMETER_CLK   GPIO4

;******************************************************************************
; register0 bits        
;
#define M_GO        1
#define M_ON        2

;******************************************************************************
;General Purpose Registers (GPR's) 
;******************************************************************************

MAIN_VARS       UDATA   0x20
;; temp variables to save state on interrupt entry
WTEMP           RES      1
STATUSTEMP      RES      1
PCLATHTEMP      RES      1
FSRTEMP         RES      1

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
        CODE

        ;; ################################################################
        ;; Init
        ;; ################################################################
Init:   
        BANKSEL TRISIO          ; page 1
	movlw	TRISIO_BITS
	movwf	TRISIO
        movlw   WPU_BITS
        movwf   WPU
        movlw   OPTION_BITS
	movwf	OPTION_REG
        movlw   b'00010001'     ; Fosc/8, GPIO0 is analog input
        movwf   ANSEL
        
        BANKSEL ADCON0          ; page 0
        movlw   b'00000001'     ; left justify, AN0, ADC on
        movwf   ADCON0

        ;; turn comprator off and configure GPIO 0,1,2 as digital.
        ;; If GPIO bit is configured as analog, it always
        ;; reads '0' even when TRISIO configures it as output
        movlw   b'00000111'     ; comparator off, all digital
        movwf   CMCON0

	clrf	TMR0
        clrf    TMR1L
        clrf    TMR1H
        
        bcf     GPIO, ACTIVITY
        
        ;; clear all registers
        movlw   REGISTERS
        movwf   FSR
        movlw   D'8'
        movwf   tmp1
_clr_reg_loop:  
        clrf    INDF
        incf    FSR,f
        decfsz  tmp1,f
        goto    _clr_reg_loop

        ;; initialize 1-wire code to work with GPIO4
        movlw   DQ
        call    ds1init
        
        goto    main_loop

wait_reset_end:
        call    ds1wait_short
        goto    wait_cmd

main_loop:
        bsf     INTCON, GIE     ; enable all interrupts
        call    ds1wait

wait_cmd:
        call    ds1rec_open_ended
        ;call    ds1rec
        btfsc   dsstat,1
        goto    wait_reset_end

cmd:    movlw   SEARCH_ROM
        subwf   indat,w
        btfss   STATUS,Z
        goto    mr

        ;; Master issued search ROM command
        call    ds1_search_rom
        ;; it appears we get stuck inside ds1_search_rom
        ;; may be not return from it when bits do not match
        ;; but instead wait for reset

        ;; we do not support any subcommands after SEARCH_ROM at this time
        goto    main_loop

        
        btfsc   dsstat,1
        goto    main_loop       ; search did not match our address
        ;; Master may issue chip-specific command after SEARCH_ROM

        call    ds1rec_open_ended
        
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
        
        ;; Perform operations specific to MATCH_ROM
        call    ds1rec
        
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

        ;; Command 0x5A: write a byte into the register N
        ;; register number follows (1 byte)
        ;; receive 3 bytes from the master (indat1, indat2, indat3)
        ;; indat3 is reversed indat2
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
        movfw   INDF
        movwf   outdat
        call    ds1sen

        btfss   register0,M_GO
        goto    main_loop
        
        bsf     GPIO,ACTIVITY
        call    read_result
        bcf     GPIO,ACTIVITY
        bcf     register0,M_GO
        
        goto    main_loop
        
reg_wr_err:
        movlw   0xA0
        movwf   outdat
        call    ds1sen
        clrf    outdat
        call    ds1sen
        goto    main_loop


delay_16ms:
        movlw   D'100'
_long_delay:
        movwf   tmp2
        call    delay_160us  ; ~160us
        decfsz  tmp2,f
        goto    $-2
        return

delay_32ms:
        movlw   D'200'
        goto    _long_delay

delay_160us:    
        movlw   0xaf        ; wait ~160 us
        movwf   TMR0
        bcf     INTCON,T0IF
        btfss   INTCON,T0IF     
        goto    $-1
        return

delay_40us:    
        movlw   0xeb        ; wait ~40 us
        movwf   TMR0
        bcf     INTCON,T0IF
        btfss   INTCON,T0IF     
        goto    $-1
        return

        ;; ################################################################
        ;; communication with voltmeter
        ;; via simplified 2-wire protocol
read_result:
        call    comm_init

        movlw   1
        movwf   register4
        movlw   0xFF
        movwf   register2
        movwf   register3
        
        call    comm_reset
        
        call    comm_rec
        movfw   indat
        movwf   register2
        
        ;; again, reversed
        call    delay_40us
        call    comm_rec
        comf    indat,f
        movfw   indat
        xorwf   register2,w
        btfss   STATUS,Z
        return                        ; error
                
        call    delay_40us
        call    comm_rec
        movfw   indat
        movwf   register3
        
        ;; again, reversed
        call    delay_40us
        call    comm_rec
        comf    indat,f
        movfw   indat
        xorwf   register3,w
        btfss   STATUS,Z
        return                        ; error
        
        clrf    register4
        
        return


        ;; ################################################################
        ;; Routines for simplified 2-wire communication protocol
        ;; based on 1-wire timings but works with two unidirectional
        ;; lines
comm_init:
        BANKSEL TRISIO
        bsf     TRISIO,VOLTMETER_IO      ; i/o line - input
        bcf     TRISIO,VOLTMETER_CLK     ; clk - output
        BANKSEL WPU
        bsf     WPU,VOLTMETER_IO         ; i/o line weak pull-up
        BANKSEL GPIO
        bsf     GPIO,VOLTMETER_CLK       ; clk line high
        return
        
        ;; send reset pulse (50us low, then 50us high)
comm_reset:     
        bcf     GPIO,VOLTMETER_CLK
        movlw   0x10            ; wait ~50 us
        movwf   tmp1
        decfsz  tmp1,f
        goto    $-1
        bsf     GPIO,VOLTMETER_CLK
        movlw   0x10            ; wait ~50 us
        movwf   tmp1
        decfsz  tmp1,f
        goto    $-1
        return
        
        ;; read time slot
        ;; return bit in C
comm_rd:
        bcf     GPIO,VOLTMETER_CLK
        call    comm_4us
        call    comm_4us
        call    comm_4us
        bcf     STATUS,C
        btfsc   GPIO,VOLTMETER_IO
        bsf     STATUS,C
        bsf     GPIO,VOLTMETER_CLK
        call    comm_4us
        call    comm_4us
        call    comm_4us
comm_4us:
        return

        ;; receive a byte, return it in indat
comm_rec:
        movlw   0x08
        movwf   tmp2
        clrf    indat
comm_rec1:        
        call    comm_rd
        rrf     indat,f
        decfsz  tmp2,f
        goto    comm_rec1
        return
        
        end
        
