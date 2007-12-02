#!/bin/sh

ADDR=$1

while :; do 
  NOW=`date`
  R0=`owread -s localhost:3000 /uncached/$ADDR/register0`
  R1=`owread -s localhost:3000 /uncached/$ADDR/register1`
  R2=`owread -s localhost:3000 /uncached/$ADDR/register2`
  printf "%s  r0=%3d   r1=%3d   r2=%3d\n" "$NOW" "$R0" "$R1" "$R2"
  sleep 0.2
done
