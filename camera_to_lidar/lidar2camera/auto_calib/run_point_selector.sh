#!/bin/bash

echo "========================================"
echo "图片点选择工具"
echo "========================================"
echo

# 检查参数
if [ $# -lt 1 ]; then
    echo "用法: $0 <帧数>"
    echo
    echo "示例:"
    echo "  $0 0000      # 选择 0000/0000.png"
    echo "  $0 0001      # 选择 0001/0001.png"
    echo
    echo "输出文件将保存在对应文件夹下:"
    echo "  - image.json"
    echo "  - image.txt"
    echo
    exit 1
fi

FRAME_ID=$1
FRAME_ID=$(printf "%04d" $FRAME_ID)
# 构建文件路径
FRAME_DIR="data/${FRAME_ID}"
IMAGE_PATH="${FRAME_DIR}/${FRAME_ID}.png"
OUTPUT_PATH="${FRAME_DIR}/image.json"

# 检查文件夹是否存在
if [ ! -d "$FRAME_DIR" ]; then
    echo "错误: 文件夹不存在: $FRAME_DIR"
    exit 1
fi

# 检查文件是否存在
if [ ! -f "$IMAGE_PATH" ]; then
    echo "错误: 图片文件不存在: $IMAGE_PATH"
    exit 1
fi

echo "帧数: $FRAME_ID"
echo "图片文件: $IMAGE_PATH"
echo "输出文件: $OUTPUT_PATH"
echo

# 运行Python程序 (使用matplotlib版本)
python3 point_selector_matplotlib.py "$IMAGE_PATH" -o "$OUTPUT_PATH"

