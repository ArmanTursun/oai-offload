#!/bin/sh -x
#cd ../openairinterface5g/ci-scripts/yaml_files/5g_rfsimulator_u0_25prb
cd ../openairinterface5g/ci-scripts/yaml_files/5g_rfsimulator
#docker compose down mysql oai-nrf oai-amf oai-smf oai-spgwu oai-ext-dn
sudo docker compose down mysql oai-amf oai-smf oai-upf oai-ext-dn
