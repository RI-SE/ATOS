#!/bin/bash
kill $(pidof object.o)
mkdir -p ./object/src/build && cd ./object/src/build
cc ../object_byte.c -lm -o ./object.o
for (( i=0; i<100; i++ ))
do
    portA=$((i*2+53240))
    portB=$((i*2+53241))
    echo "Start object $i with port $portA  $portB"
    ./object.o 57.7775688 12.7812653 123.43 "$portA" "$portB" &
done

