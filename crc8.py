#!/usr/bin/python

# Generate CRC8 for Dallas 1-Wire addresses

import os
import sys

DSCRC_TABLE = [
    0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65, 
    157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220, 
    35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98, 
    190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255, 
    70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7, 
    219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154, 
    101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36, 
    248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185, 
    140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205, 
    17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80, 
    175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238, 
    50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115, 
    202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139, 
    87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22, 
    233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168, 
    116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53
]

addr = sys.argv[1]

# address should be provided in the form '10.BDF303010800'

aa = addr.split('.')

addr = aa[1]
family_code = int(aa[0],16)

alist = []

crc8 = 0
crc8 = DSCRC_TABLE[crc8 ^ family_code]
alist.append(family_code)

for c in range(0,6):
    idx = c*2
    ab = addr[idx:idx+2]
    iab = int(ab,16)
    crc8 = DSCRC_TABLE[crc8 ^ iab]
    alist.append(iab)

alist.append(crc8)

print
print "\t;; 1-wire address %s " % '.'.join(['%02x' % x for x in alist])

print """
\tGLOBAL\tget_addr_byte

DS1W_addr  CODE  0x40

;-----------------------------------------------------------------------------
;  Note that bytes of the address are in reverse order
; this is so we can use index as a counter and iterate using decfsz
get_addr_byte:\taddwf   PCL,f
\t\tretlw   0"""

cntr = 7
while len(alist)>0:
    a = alist.pop(-1)
    print 'ds1_addr%d\tretlw\t0x%x' % (cntr,a)
    cntr -= 1

print "\treturn"
