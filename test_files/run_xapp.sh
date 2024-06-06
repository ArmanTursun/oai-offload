#!/bin/sh -x

#cd flexric-offload
#./build/examples/xApp/c/monitor/xapp_kpm_moni

#cd flexric-offload
#./build/examples/xApp/c/monitor/xapp_gtp_mac_rlc_pdcp_moni

##
#cd flexric-offload
#./build/examples/xApp/c/kpm_rc/xapp_kpm_rc
#

#cd flexric-offload
#./build/examples/xApp/c/helloworld/xapp_hw

#cd flexric-offload
#./build/examples/xApp/c/ctrl/xapp_mac_ctrl

cd flexric-offload/build/examples/xApp/python3/
python3 xapp_mac_rlc_pdcp_gtp_moni.py
#python3 xapp_slice_moni_ctrl.py

