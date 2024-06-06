#!/bin/bash

#cd oai-offload/cmake_targets/
cd openairinterface5g/cmake_targets/
sudo ./build_oai -I --gNB -w SIMU --build-e2 --ninja #--build-lib telnetsrv --build-lib uescope --build-lib nrscope
#cd ran_build/build/
#make measurement_display
