#!/bin/bash

# 获取所有数据文件夹
dirs=(./data/*/)

# 随机选择一个文件夹
if [ ${#dirs[@]} -gt 0 ]; then
    # 生成随机索引
    random_index=$((RANDOM % ${#dirs[@]}))
    selected_dir=${dirs[$random_index]}
    
    echo "Randomly selected: $selected_dir"
    echo "Processing $selected_dir ..."
    ./bin/run_lidar2camera "$selected_dir"
    python update.py
else
    echo "No data directories found!"
fi