#!/bin/bash

for d in ./data/*/ ; do
    echo "Processing $d ..."
    ./bin/run_lidar2camera "$d"
    python update.py
done