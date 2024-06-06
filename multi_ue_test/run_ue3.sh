#!/bin/sh -x
#cd /root/openairinterface5g.develop/ci-scripts/yaml_files/5g_rfsimulator
cd ../openairinterface5g/ci-scripts/yaml_files/5g_rfsimulator_u0_25prb/
docker compose up -d oai-nr-ue3
docker exec -it rfsim5g-oai-nr-ue3 /bin/bash
