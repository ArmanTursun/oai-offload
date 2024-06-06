#!/bin/sh -x

cd ../openairinterface5g/ci-scripts/yaml_files/5g_rfsimulator
#docker compose down mysql oai-nrf oai-amf oai-smf oai-spgwu oai-ext-dn oai-nr-ue oai-nr-ue2 oai-nr-ue3 oai-nr-ue4 oai-nr-ue5 oai-nr-ue6 oai-nr-ue7 oai-nr-ue8 oai-nr-ue9 oai-nr-ue10 oai-nr-ue11 oai-gnb

docker compose down mysql oai-amf oai-smf oai-nr-ue oai-gnb oai-upf oai-ext-dn
