#!/bin/sh -x
cd /root/openairinterface5g.develop/ci-scripts/yaml_files/5g_rfsimulator
docker compose up -d oai-nr-ue11
docker exec -it rfsim5g-oai-nr-ue11 /bin/bash
