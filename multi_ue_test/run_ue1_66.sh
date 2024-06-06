#!/bin/sh -x
#cd ../openairinterface5g/ci-scripts/yaml_files/5g_rfsimulator
cd ../openairinterface5g/ci-scripts/yaml_files/5g_rfsimulator_u0_25prb/
docker compose up -d oai-nr-ue
docker exec -it rfsim5g-oai-nr-ue /bin/bash
