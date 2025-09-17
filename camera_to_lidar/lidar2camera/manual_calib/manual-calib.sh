#!/bin/bash

if [ $# -ne 1 ]; then
  echo "用法: $0 帧号"
  exit 1
fi

frame=$(printf "%04d" $1)  # 保证是4位数，比如输入190会变成0190

# 创建 records 文件夹（如果不存在）
mkdir -p records

# 移动当前目录下的 .jpg 和 .txt 文件（排除 CMakeLists.txt）
shopt -s extglob  # 开启扩展通配符
mv -f ./*.jpg !(CMakeLists).txt records/ 2>/dev/null
shopt -u extglob  # 关闭扩展通配符

echo "已将当前目录下的 .jpg 和 .txt 文件(排除 CMakeLists.txt)移到 records 文件夹"

cmd="./bin/run_lidar2camera data/${frame}/${frame}.png \
    data/${frame}/${frame}.pcd \
    data/${frame}/center_camera-intrinsic.json \
    data/${frame}/top_center_lidar-to-center_camera-extrinsic.json"

echo "执行命令: $cmd"
eval $cmd

