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
        GLOBAL  indat,indat1,indat2,indat3,outdat,dsstat
        GLOBAL  ds1close,ds1init
        GLOBAL  ds1wait_short,ds1wait,ds1rec_open_ended
        GLOBAL  ds1_search_rom,ds1_match_rom
        GLOBAL  ds1rec,ds1sen,ds1_rx3,ds1sen
        GLOBAL  dm1res,dm1sen,dm1rec
ELSE
        EXTERN  indat,indat1,indat2,indat3,outdat,dsstat
        EXTERN  ds1close,ds1init
        EXTERN  ds1wait_short,ds1wait,ds1rec_open_ended
        EXTERN  ds1_search_rom,ds1_match_rom
        EXTERN  ds1rec,ds1sen,ds1_rx3
        EXTERN  dm1res,dm1sen,dm1rec
ENDIF


