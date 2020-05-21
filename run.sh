#!/bin/bash

dir=$(basename "$PWD")

mkdir runs
for i in $(seq 6 10)
do
  mkdir runs/simul$i
  time ./waf --run "mswim --randomSeed=$i --algorithm=$dir" > runs/simul$i/${dir}_${i}.txt
  mv *xml *txt *csv runs/simul$i
done
