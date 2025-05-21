#!/bin/bash

# Build MiniDexed
cd src
make clean || true 
make -j
ls *.img
cd ..
