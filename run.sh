#!/bin/bash

dir=$(basename "$PWD")

mkdir runs
for k in $(seq 0 2)
do
	mkdir runs/type${k}
	for i in $(seq 6 10)
	do
	  mkdir runs/type${k}/simul$i
	  time ./waf --run "mswim --randomSeed=$i --algorithm=$dir --applicationType=$k" > runs/type${k}/simul$i/${dir}_${i}.txt
	  mv *xml *txt *csv runs/type${k}/simul$i
	done
done
