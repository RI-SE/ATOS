#!/bin/bash
for n in {1..100}; do
    cp ./server/traj/0.traj ./server/build/traj/127.0.0.$n
done
