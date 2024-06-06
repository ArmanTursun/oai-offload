#!/bin/bash

cd flexric-offload/build/
sudo make uninstall
cd ..
rm -rf build
mkdir build
cd build
cmake ..
make -j10
sudo make install
