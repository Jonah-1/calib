#!/bin/bash

echo "========================================"
echo "Lidar-Image数据合并工具"
echo "========================================"
echo

# 检查参数
if [ $# -lt 1 ]; then
    echo "用法: $0 <帧数> [输出文件]"
    echo
    echo "示例:"
    echo "  $0 0000"
    echo "  $0 0000 -o merged_0000.json"
    echo
    echo "说明:"
    echo "  自动加载 data/<帧数>/lidar.txt 和 data/<帧数>/image.json"
    echo "  输出合并后的数据到 data/<帧数>/merged_data.json"
    echo
    exit 1
fi

FRAME_ID=$1
OUTPUT_FILE=${2:-"data/${FRAME_ID}/merged_data.json"}

# 构建文件路径
FRAME_DIR="data/${FRAME_ID}"
LIDAR_PATH="${FRAME_DIR}/lidar.txt"
IMAGE_PATH="${FRAME_DIR}/image.json"

# 检查文件夹是否存在
if [ ! -d "$FRAME_DIR" ]; then
    echo "错误: 文件夹不存在: $FRAME_DIR"
    exit 1
fi

# 检查文件是否存在
if [ ! -f "$LIDAR_PATH" ]; then
    echo "错误: Lidar文件不存在: $LIDAR_PATH"
    exit 1
fi

if [ ! -f "$IMAGE_PATH" ]; then
    echo "错误: Image文件不存在: $IMAGE_PATH"
    exit 1
fi

echo "帧数: $FRAME_ID"
echo "Lidar文件: $LIDAR_PATH"
echo "Image文件: $IMAGE_PATH"
echo "输出文件: $OUTPUT_FILE"
echo

# 运行Python程序
python3 merge_data.py "$LIDAR_PATH" "$IMAGE_PATH" -o "$OUTPUT_FILE"
