
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
;  GPIO0 - servo current input
;  GPIO1 - 
;  GPIO2 - PWM output
;  GPIO3 - 
;  GPIO4 - 1-wire
;  GPIO5 - "activity" LED
;
;  Controlling the servo:
;
;  register0 - status and control register. Bits:
;
;    0 - R   0: target code reached; 1: begin transfer/transfer in progress
;    1 - R/W 1: 'jump' mode                           '2'
;    2 - R/W 1: linear transfer                       '4'
;    3 - R/W 1: 'fast' linear transfer                '8'
;    4 - R/W 1: slow start/breaking                   '16'
;    5 - 
;    6 - 
;    7 - R/W PWM change period; 0: 4096us; 1: 40960us
;        
;  register1 - PWM pulse 8 MSB
;  register2 - PWM pulse 2 LSB
;
;  register5 - servo current (8 bit resolution):
;              I(servo) = reg5*0.036 A (for resistor 0.47 Ohm)
;  register6 - current value of 8 MSB of duty cycle code
;              should be equal to register1
;  register7 - current value of 2 LSB of duty cycle code
;              should be equal to register2
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

#define TRISIO_BITS     B'11011101' ; GPIO 1,5: out, GPIO 0,3,4: in, 2: PWM
#define WPU_BITS        B'00000000' ; weak pull-ups off
#define OPTION_BITS	b'10000000' ; assign TMR0 prescaler 1:2 for TMR0,
                                    ; GPIO pull-ups disabled
#define T1CON_BITS      b'00110001' ; TMR1ON, 1:8 prescaler
#define CCP1CON_BITS    b'00001100' ; DC1B1,DC1B0=0, PWM mode active high

;******************************************************************************
; pin assignment
;******************************************************************************

#define PWM             GPIO2
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

        decfsz  skip_for_adc,f
        goto    skip_adc
        movlw   d'128'
        movwf   skip_for_adc
        
        bsf     ADCON0,GO       ; start conversion
        
        call    run_pwm

        BANKSEL ADCON0
_wait_adc:       
        btfsc   ADCON0,GO_DONE
        goto    _wait_adc
        ;; ADC data ready
        BANKSEL ADRESH
        movfw   ADRESH
        movwf   register5
        goto    led_off
        
skip_adc:       
        call    run_pwm

led_off:        
        BANKSEL GPIO
        bcf     GPIO, ACTIVITY    ; "activity" LED

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

        ;; initialize timer1. Use register0
        ;; bit 3 - 0: period 4096us; 1: period 40960us
        
        ;; *** Keep this in this code block to ensure
        ;; it ends up in the lower part of address space. This
        ;; simplifies calculated jump code.
tmr1_init:
        btfss   register0,TIMER_SPEED_BIT
        goto    tmr1_4096_usec
        goto    tmr1_40960_usec

        
        
;******************************************************************************
;Initialization
;******************************************************************************
MAIN    CODE

        ;;  register1 -- PWM duty cycle code, 8 MSB
        ;;  register2 -- PWM duty cycle code, 2 LSB
        ;;
        ;;  register6 -- current value of 8 MSB of duty cycle code
        ;;               should be equal to register1
        ;;  register7 -- current value of 2 LSB of duty cycle code
        ;;               should be equal to register2

run_pwm:        
        movfw   register0
        andlw   ALL_MODES       ; bits that control transfer modes
        btfsc   STATUS,Z
        goto    _dis
        call    pwm_enable      ; register0:1 or 2 != 0
        goto    _cont_pwm_run
_dis:   
        call    pwm_disable     ; register0:1 and 2 == 0
        goto    _ext_pwm_run

_cont_pwm_run:
        call    adjust_duty_cycle_code
        btfsc   STATUS,C
        call    pwm_change_duty_cycle
_ext_pwm_run:   
        call    tmr1_init
        return

        ;; delta = (4*register1 + register2) - (4*register6 + register7)
        ;; delta may be > 256 or < 0
        ;; 
        ;; Return:
        ;; 
        ;; abs(combined 10-bit value of delta)
        ;; if abs(combined 10-bit value of delta) > 64, then return 64
        ;; (64 is the maximum length of slow start/stop sequence)
        ;; 
compute_delta:
        clrf    delta
        clrf    r1r6neg

        ;; first check if abs(register1 - register6) > 16
        ;; note that registers 1 and 6 represent higher 8 bits of
        ;; a 10-bit integer. To compare this 10-bit number with 64
        ;; only using higher 8 bits, we compare with 64/4=16
        ;; If abs(r1 - r6) > 16, then we always just return '64'
        ;; and do not compute anything.
        ;; If abs(r1 - r6) < 16, then complete 10-bit numbers differ
        ;; no more than by 16*4 = 64. This means delta < 64.
        ;; Since delta is lesser than 64, we can use signed arightmetics
        ;; to simplify code.
        ;; 
        movfw   register1
        movwf   tmp1
        movfw   register6
        subwf   tmp1,f
        ;; tmp1 = register1 - register6
        btfsc   STATUS,Z
        goto    _lt_16          ; register1 == register6
        btfsc   STATUS,C
        goto    _cmp_16
        comf    tmp1,f
        incf    tmp1,f
        incf    r1r6neg,f       ; to indicate that (r1-r6) is negative
_cmp_16:
        ;; compare tmp1 with 16 (actually 15)
        movlw   b'11110000'
        andwf   tmp1,w
        btfss   STATUS,Z
        ;; there are some '1' in bits 7,6,5,4
        goto    _ret_64
        
        ;; if we get here, then abs(register1 - register6) < 16
        ;; tmp1 = abs(register1 - register6)
_lt_16:
        ;; restore sign of tmp1
        btfss   r1r6neg,1
        goto    _mul_tmp1_by_4
        comf    tmp1,f
        incf    tmp1,f

_mul_tmp1_by_4:     
        ;; tmp1 = tmp1*4
        bcf     STATUS,C
        rlf     tmp1,f
        rlf     tmp1,f

        ;; compute r2-r7
        movfw   register2
        movwf   tmp2
        movfw   register7
        subwf   tmp2,f
        ;; tmp2 = register2 - register7
        movfw   tmp2
        addwf   tmp1,f
        ;; tmp1 = tmp1 + tmp2
        btfss   tmp1,7
        goto    _delta_ready
        comf    tmp1,f
        incf    tmp1,f
_delta_ready:    
        movfw   tmp1
        movwf   delta
        return
        
_ret_64:
        movlw   d'64'
        movwf   delta
        return
        
        ;; increment or decrement code in reg6/reg7
        ;; Eventually register6 should become equal to register1
        ;; and register7 to register2
        ;; Return with bit C set if we need to push updated
        ;; duty cycle code to PWM module
        ;; Keeping this funtion in the main code block
        ;; to minimize amount of memory used in IRQ service routine
        
adjust_duty_cycle_code:
        decfsz  skip_counter,f
        goto    _ext_c_clear

        btfss   register0,BEGIN_TRANSFER
        goto    _ext_c_clear
        
        movfw   register6
        subwf   register1,w     ; w = register1 - register6
        ;; if register1 == register6 then check register7
        btfsc   STATUS,Z
        goto    check_reg7
        ;; if register1 > register6 then goto incr_duty_cycle_code
        btfss   STATUS,C
        goto    decr_duty_cycle_code
        goto    incr_duty_cycle_code

check_reg7:
        movfw   register7
        subwf   register2,w     ; w = register2 - register7
        ;; if register2 == register7 then we are done
        btfsc   STATUS,Z
        goto    end_of_transfer
        ;; if register2 > register7 then goto incr_duty_cycle_code
        btfss   STATUS,C
        goto    decr_duty_cycle_code
        
incr_duty_cycle_code:
        btfsc   register0,JUMP_MODE
        goto    jump_mode
        
        ;; register1 > register6
        incf    register7,f
        btfss   register7,2
        goto    return_with_change
        ;; register7 > 3
        clrf    register7
        incf    register6,f
        goto    return_with_change

decr_duty_cycle_code:
        btfsc   register0,JUMP_MODE
        goto    jump_mode
        
        ;; register1 < register6
        decf    register7,f   ; register7 = register7 - 1
        btfss   register7,7
        goto    return_with_change
        ;; register7 < 0
        movlw   3
        movwf   register7
        decf    register6,f
        
return_with_change:     
        call    get_skip_counter
        movwf   skip_counter
        bsf     STATUS,C
        return

end_of_transfer:
        ;; r1,2 == r6,7 - servo is in requested position
        ;; clear slow start sequence counter to prepare for
        ;; the next move
        clrf    delta
        clrf    offset
        clrf    start_seq_idx
        clrf    cruizing
        bcf     register0,BEGIN_TRANSFER
        movlw   1
        movwf   skip_counter
_ext_c_clear:   
        bcf     STATUS,C
        return

        ;; jump mode, just copy reg1 -> reg6, reg2 -> reg7
jump_mode:      
        movfw   register1
        movwf   register6
        movfw   register2
        movwf   register7
        goto    return_with_change
        
        
get_skip_counter:
        btfsc   register0,SLOW_START_MODE
        goto    slow_start
        btfsc   register0,LINEAR_MODE
        retlw   3               ; linear mode skips 3
        btfsc   register0,FAST_LINEAR_MODE
        retlw   2               ; fast linear mode skips 2
        retlw   1               ; other modes do not skip
slow_start:     
        ;; slow start/stop
        btfsc   cruizing,0
        goto    _in_cruize_mode
        
        ;; in slow start yet
        movfw   start_seq_idx
        incf    start_seq_idx,f
        call    get_seq_code
        ;; important !!! PCLATH is used in get_seq_code because it
        ;; is located in the page 3
        clrf    PCLATH
        movwf   tmp1
        btfss   tmp1,7
        goto    _ret_skip_code
        ;; skip code is b'10000000' => switch to cruize mode
        bsf     cruizing,0
        movlw   0
        goto    _ret_skip_code
        
_in_cruize_mode:
        ;; in cruize mode use delta to get skip counter
        call    compute_delta
        movfw   delta
        call    get_seq_code
        ;; important !!! PCLATH is used in get_seq_code because it
        ;; is located in the page 3
        clrf    PCLATH
        movwf   tmp1
        btfss   tmp1,7
        goto    _ret_skip_code
        ;; skip code is b'10000000' => replace with 0, no breaking yet
        movlw   0

_ret_skip_code: 
        ;; we use skip counter with decfsz, we do not
        ;; skip if skip_cntr==1.
        addlw   1
        return                  ; code is in W
 
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
        ;; Perform one ADC measurement
        ;; ################################################################
adc:
        BANKSEL TRISIO
        bsf     TRISIO,GPIO0
        BANKSEL ANSEL
        movlw   b'00010001'     ; Fosc/8, GPIO0 is analog input
        iorwf   ANSEL,f

        BANKSEL ADCON0
        movlw   b'00000001'     ; left justify, AN0, ADC on
        movwf   ADCON0
        
        call    adc_sample_time
        bsf     ADCON0,GO       ; start conversion
        btfsc   ADCON0,GO_DONE
        goto    $-1
        ;; ADC data ready
        BANKSEL ADRESH
        movfw   ADRESH
        movwf   register3
        BANKSEL ADRESL
        movfw   ADRESL
        movwf   register4
        
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

	BANKSEL TMR0

;;         ;;  interrupts
;;         ;;  GPIO state change interrupt
;;         ;;  first, read from GPIO to clear mismatches
;;         movfw   GPIO
;;         bsf     INTCON,GPIE     ;Interrupt on GPIO port change
;;         bcf     INTCON,GPIF     ;Clear port change Interrupt Flag
;;         bsf     INTCON,GIE      ;Turn on Global Interrupts
        
        clrf    skip_for_adc
	clrf	TMR0
        clrf    TMR1L
        clrf    TMR1H

        bcf     GPIO, PWM
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
        
        clrf    register0       ; pwm off, fastest period
        call    tmr1_init
        call    pwm_init

        clrf    offset
        clrf    delta
        clrf    start_seq_idx
        clrf    cruizing
        movlw   1
        movwf   skip_counter
        
        goto    main_loop

wait_reset_end:
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

        ;; do not enable pwm at the beginning, wait for the user
        ;; to set bit 1 or 2 in register0 instead
        ;call    pwm_enable
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

        ;; sequence for slow start
        ;; index is in W on entry
get_seq_code:
        BANKSEL EEADR
        movwf   EEADR
        bsf     EECON1,RD
        movfw   EEDAT
        BANKSEL GPIO
        return

        ;; placing data at org 0x2100 makes compiler
        ;; generate code section for EEPROM which programmer
        ;; dutifully writes to EEPROM
        ;;
        ;; Each figure in this table makes us skip corresponding
        ;; number of interrupt cycles. Code b'10000000' means end of table
        ;; and skips no cycles (is equal to zero).
        ;; When interrupt timer is programmed for 4096us cycle (~4ms)
        ;; 8 blocks 16 values each configured as follows:
        ;; 8,8,8... 4,8,4,8...
        ;; 4,4,4... 2,4,2,4...
        ;; 2,2,2... 1,2,1...
        ;; 1,1,1,1... 0,1,0,1...
        ;; in total takes 416 cycles or 1664ms
        ;;
        ;; 8 blocks of 10 values take 260 cycles or 1064ms
        ;; 
        ORG     0x2100
        DW      8,8,8,8
        DW      4,8,4,8,4,8
        DW      4,4,4,4,4,4,4,4
        DW      2,4,2,4,2,4,2,4,2,4
        DW      2,2,2,2,2,2,2,2,2,2
        DW      1,2,1,2,1,2,1,2,1,2
        DW      1,1,1,1,1,1,1,1,1,1
        DW      0,1,0,1,0,1,0,1,0,1

        DW      b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000',b'10000000'
   
        end
        
