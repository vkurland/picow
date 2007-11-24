;******************************************************************************
; 1-wire commands
;******************************************************************************
#define SEARCH_ROM      0xF0
#define COND_SEARCH     0xEC
#define READ_ROM        0x33
#define MATCH_ROM       0x55
#define READ_MEMORY     0xAA
#define SKIP_ROM        0xCC

        
dareset equ     1               ; reset flag bit in dsstat

IFDEF   DS1WIRE_CODE
        GLOBAL  ds1close,ds1init, ds1main
        GLOBAL  REGISTERS
        GLOBAL  set_error_led_port, set_activity_led_port
        GLOBAL  actled_on, actled_off, errled_on, errled_off

        EXTERN  read_register_hook, write_to_register_hook, idle_hook
ELSE
        EXTERN  ds1close,ds1init, ds1main
        EXTERN  REGISTERS
        EXTERN  set_error_led_port, set_activity_led_port
        EXTERN  actled_on, actled_off, errled_on, errled_off

        GLOBAL  read_register_hook, write_to_register_hook, idle_hook
ENDIF

#define register0 REGISTERS
#define register1 REGISTERS+1
#define register2 REGISTERS+2
#define register3 REGISTERS+3
#define register4 REGISTERS+4
#define register5 REGISTERS+5
#define register6 REGISTERS+6
#define register7 REGISTERS+7


