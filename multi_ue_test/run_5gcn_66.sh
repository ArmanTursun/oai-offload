#!/bin/sh -x
#cd ../openairinterface5g/ci-scripts/yaml_files/5g_rfsimulator
cd ../openairinterface5g/ci-scripts/yaml_files/5g_rfsimulator_u0_25prb
sudo docker compose up -d mysql oai-amf oai-smf oai-upf oai-ext-dn
