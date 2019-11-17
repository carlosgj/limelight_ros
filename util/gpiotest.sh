#!/bin/bash
for i in {0..45}
do
  echo "$i" > /sys/class/gpio/export
done

for i in {0..45}
do
  cat /sys/class/gpio/gpio$i/value | tr '\n' ' '
done
echo ""
