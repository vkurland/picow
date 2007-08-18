#!/usr/bin/python


import sys
import os
import time

REGISTER = '/uncached/FE.000000000005/register%d'
ADC_CALIBRATION = 0.004914

def ReadOWFile(owfsf, timeout=1.0, pause=0.3):
  res = ''
  for line in os.popen('owread -s localhost:3000 %s' % owfsf):
    res = line
  return res

def WriteOWFile(owfsf, str, timeout=1.0, pause=0.3):
  os.system('owwrite -s localhost:3000 %s %s' % (owfsf,str))


def ready():
  owfs_file = REGISTER % 0
  r0 = int(ReadOWFile(owfs_file))
  return r0 & 1

def start():
  owfs_file = REGISTER % 0
  r0_val = 6
  WriteOWFile(owfs_file, '%d' % r0_val)
  time.sleep(0.5)
  return

def read_adc():
  owfs_file = REGISTER % 0
  reg0_val = int(ReadOWFile(owfs_file))
  owfs_file = REGISTER % 2
  reg2_val = int(ReadOWFile(owfs_file))
  owfs_file = REGISTER % 3
  reg3_val = int(ReadOWFile(owfs_file))
  # data is left-justified
  reg3_val_c = (reg3_val >> 6) & 3
  return (reg2_val,reg3_val,(4*reg2_val + reg3_val_c) * ADC_CALIBRATION)

def int2bin(n, count=24):
  """returns the binary of integer n, using count number of digits"""
  return "".join([str((n >> y) & 1) for y in range(count-1, -1, -1)])
  
def main(args):
  v0 = v0max = 0
  v0min = 10000
  start_time = time.time()

  while True:
    #while not ready():
    #  time.sleep(1)
    start()
    (reg2,reg3,v0) = read_adc()

    owfs_file = REGISTER % 4
    reg4 = int(ReadOWFile(owfs_file))
    owfs_file = REGISTER % 5
    reg5 = int(ReadOWFile(owfs_file))
    owfs_file = REGISTER % 6
    reg6 = int(ReadOWFile(owfs_file))
    owfs_file = REGISTER % 7
    reg7 = int(ReadOWFile(owfs_file))

    if reg4==0:
      v0max = max(v0,v0max)
      v0min = min(v0,v0min)

    print '%05d: r2=%s r3=%s r4=%s r5=%d r6=%d r7=%d %6.3f (%6.3f ..%6.3f)' % \
        (time.time()-start_time,
         int2bin(reg2,8),int2bin(reg3,8),int2bin(reg4,8),
	 reg5,reg6,reg7,
	 v0,v0min,v0max)
    #time.sleep(1)

if __name__ == '__main__':
    sys.exit( main(sys.argv[1:]) )
