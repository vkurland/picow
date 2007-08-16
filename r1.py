#!/usr/bin/python


import sys
import os
import time

REGISTER = '/uncached/FE.000000000005/register%d'

def ReadOWFile(owfsf, timeout=1.0, pause=0.3):
  res = ''
  for line in os.popen('owread -s localhost:3000 %s' % owfsf):
    res = line
  return res

def WriteOWFile(owfsf, str, timeout=1.0, pause=0.3):
  os.system('owwrite -s localhost:3000 %s %s' % (owfsf,str))

def int2bin(n, count=24):
  """returns the binary of integer n, using count number of digits"""
  return "".join([str((n >> y) & 1) for y in range(count-1, -1, -1)])
  
def main(args):
  for r in range(0,8):
    owfs_file = REGISTER % r
    regv = int(ReadOWFile(owfs_file))
    print 'r%d: %4d %02x %s' % (r,regv,regv,int2bin(regv,8))
  print

if __name__ == '__main__':
    sys.exit( main(sys.argv[1:]) )
