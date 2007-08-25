
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
;  GPIO2 - PWM output
;  GPIO3 - 
;  GPIO4 - 1-wire in/out
;  GPIO5 - input from voltmeter
;
;  Controlling the servo:
;
;  register0 - status and control register. Bits:
;
;    0 - M_READY: 1: ready for measurement (charge complete)
;    1 - M_GO   : 1: perform measurement, 0: measurement complete
;    2 - M_ON   : 1: power transfer is on, measurement reading is enabled
;    3 - 
;    4 - 
;    5 - 
;    6 - 
;    7 - 
;        
;  register1 - max charge threshold (default 200)
;  register2 - result: ADCH
;  register3 - result: ADCL
;  register4 - if '1', then there was an error reading data from voltmeter
;  register5 - charge capacitor voltage
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

#define TRISIO_BITS     B'11111100' ; GPIO 0,1: out, GPIO 3,4,5: in, 2: PWM
#define WPU_BITS        B'00001000' ; weak pull-ups off, GPIO3 pull-up on
        
;; assign TMR0 prescaler 1:2 for TMR0, GPIO pull-ups enabled
;; TMR0 is used in ds1wire code, prescaler 1:2 is a requirement
#define OPTION_BITS	b'0000000'

#define T1CON_BITS      b'00110001' ; TMR1ON, 1:8 prescaler
#define CCP1CON_BITS    b'00001100' ; DC1B1,DC1B0=0, PWM mode active high

#define PWM             GPIO2
        ;; 1-wire bus
#define DQ              GPIO4
        ;; communication with voltmeter via second 1-wire bus
        ;; (requires bi-directional galvanic isolator)
#define VOLTMETER_DQ    GPIO5
        ;; communication with voltmeter via simplified 2-wire protocol
        ;; (each wire is unidirectional)
#define VOLTMETER_IO    GPIO3
#define VOLTMETER_CLK   GPIO5

;******************************************************************************
; register0 bits        
;
#define M_CHARGING  0
#define M_GO        1
#define M_ON        2

; we charge during 1 period out of total CHARGE_CYCLE periods
#define CHARGE_CYCLE D'16'
        
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
charge_cntr     RES     1
        
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

        btfss   PIR1, TMR1IF
        goto    intext          ; not tmr1 interrupt
        bcf     PIR1, TMR1IF

        btfsc   register0,M_CHARGING
        call    pwm_disable
        bcf     register0,M_CHARGING
        
        decfsz  charge_cntr,f
        goto    restart_tmr
        movlw   CHARGE_CYCLE
        movwf   charge_cntr

        btfss   register0,M_ON
        goto    restart_tmr
        
        bsf     register0,M_CHARGING
        call    pwm_enable
        
restart_tmr:
        call    tmr_init
        
intext:
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
        CODE

tmr_init:
        call    tmr1_4096_usec
        ;call    tmr1_one_tenth_sec
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

tmr1_max:       
        movlw   0
        movwf   TMR1H
        movwf   TMR1L
        goto    tmr1_start
        
        ;; initialize TMR1 for 4.096 msec count
        ;; in the end of which it generates interrupt
        ;; using prescaler 1:8
        ;; timer counts every 8us
        ;; need 4096us, counting backwards to 0
        ;; 4096us corresponds to 4096/8=512 counts
        ;; preload timer counter with 65536-512 = 65024 = 0xFE00
        ;; 
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
        
        bcf     GPIO, PWM
        
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
        
        call    tmr_init
        call    pwm_init

        movlw   CHARGE_CYCLE
        movwf   charge_cntr
        bsf     register0,M_CHARGING
        call    pwm_enable
        
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

        btfss   register0,M_ON
        goto    $+3
        btfsc   register0,M_GO
        call    read_result     ; only if M_ON && M_GO
        bcf     register0,M_GO
        
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
        ;; if oscillator frequency is 4MHz and using prescaler 1, we
	;; need to load PR2 with 1 to get 500kHz PWM signal
        ;;
        
        BANKSEL PR2
        movlw   1
        movwf   PR2

        ;; 3. configure CCP module for PWM mode by loading
        ;; the CCP1CON register with the approp. val.
        BANKSEL CCP1CON
        movlw   CCP1CON_BITS
        movwf   CCP1CON

        ;; 4. set the PWM duty cycle by loading the CCPR1L
        ;; register and DC1B bits of the CCP1CON register
        ;; For duty cycle 50% CCPR1L=1, CCP1CON<5:4>=0
        BANKSEL CCPR1L
        movlw   1
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
        clrf    T2CON           ; prescaler 1 : b'00'
        bsf     T2CON,TMR2ON    ; tmr2 on

        ;; 6. Enable PWM output after a new PWM cycle has 
        ;;    started: 
        ;;  • Wait until Timer2 overflows (TMR2IF bit of 
        ;;    the PIR1 register is set). 
        ;;  • Enable the CCP1 pin output driver by 
        ;;    clearing the associated TRIS bit.
        call    wait_tmr2
        ;; do not enable pwm output yet
        return
        
wait_tmr2:
        BANKSEL PIR1
        btfss   PIR1,TMR2IF
        goto    $-1
        return

        ;; enable or disable pwm
        ;; if C is set, then enable
        ;; if C is cleared, then disable
pwm_change_status:
        btfss   STATUS,C
        goto    pwm_disable

pwm_enable:
        BANKSEL TRISIO
        movlw   TRISIO_BITS
        movwf   tmp1
        bcf     tmp1, TRISIO2
        movfw   tmp1
        movwf   TRISIO
        BANKSEL GPIO
        return
        
pwm_disable:
        BANKSEL TRISIO
        movlw   TRISIO_BITS
        movwf   tmp1
        bsf     tmp1, TRISIO2
        movfw   tmp1
        movwf   TRISIO
        BANKSEL GPIO
        return

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
IFDEF   VOLTMETER_OVER_1WIRE_COMM
        
        ;; communication with voltmeter
        ;; via normal 1-wire protocol over the second 1-wire bus
        ;; requires bi-directional galvanic isolator
read_result:
        call    delay_32ms
        ;call    delay_32ms
        ;call    delay_32ms
        ;call    delay_32ms


        call    ds1close
        movlw   VOLTMETER_DQ
        call    ds1init

        ;; error code '1' means there was no presence pulse
        movlw   1
        movwf   register4
        movlw   0xFF
        movwf   register2
        movwf   register3
        
        call    dm1res          ; reset, wait for presence
        btfsc   STATUS,C
        goto    restore_1w      ; no presence pulse
        
        ;; error code '2' means two copies of a byte did not match
        movlw   2
        movwf   register4

        movlw   SKIP_ROM
        call    dm1sen
        
        call    delay_160us
        call    dm1rec
        movfw   indat
        movwf   register2
        
        ;; again, reversed
        call    delay_160us
        call    dm1rec
        comf    indat,f
        movfw   indat
        xorwf   register2,w
        btfss   STATUS,Z
        goto    restore_1w      ; error
                
        call    delay_160us
        call    dm1rec
        movfw   indat
        movwf   register3
        
        ;; again, reversed
        call    delay_160us
        call    dm1rec
        comf    indat,f
        movfw   indat
        xorwf   register3,w
        btfss   STATUS,Z
        goto    restore_1w      ; error
        
        clrf    register4
        
restore_1w:     
        call    ds1close
        movlw   DQ
        call    ds1init

        return

ELSE
        ;; communication with voltmeter
        ;; via simplified 2-wire protocol
read_result:
        call    delay_32ms

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
        
ENDIF
        
        end
        
