#!/bin/bash

echo "========================================"
echo "PnP标定工具"
echo "========================================"
echo

# 检查参数
if [ $# -lt 1 ]; then
    echo "用法: $0 <帧数> [选项] [输出文件]"
    echo
    echo "示例:"
    echo "  $0 0000                                    # 默认不使用Tr初始值"
    echo "  $0 0000 -o calibration_0000.json         # 指定输出文件"
    echo "  $0 0000 --use-tr-initial                  # 使用Tr初始值"
    echo "  $0 0000 --use-tr-initial -o output.json   # 使用Tr初始值并指定输出"
    echo
    echo "选项:"
    echo "  --use-tr-initial      使用calib.txt中的Tr外参作为PnP求解的初始值"
    echo
    echo "说明:"
    echo "  读取 data/<帧数>/merged_data.json 和 data/<帧数>/calib.txt"
    echo "  使用PnP算法计算雷达到相机的外参矩阵"
    echo "  输出标定结果到 data/<帧数>/calibration_results.json"
    echo
    exit 1
fi

FRAME_ID=$1
FRAME_ID=$(printf "%04d" $FRAME_ID)

# 获取除第一个参数外的所有参数
shift
SCRIPT_ARGS="$@"

# 如果没有指定--use-tr-initial，默认不使用Tr初始值
if [[ "$SCRIPT_ARGS" != *"--use-tr-initial"* ]]; then
    # 默认不使用Tr初始值，保持原有行为
    SCRIPT_ARGS="$SCRIPT_ARGS"
fi

# 构建文件路径
FRAME_DIR="data/${FRAME_ID}"
MERGED_DATA_PATH="${FRAME_DIR}/merged_data.json"
CALIB_PATH="${FRAME_DIR}/calib.txt"


# 检查文件夹是否存在
if [ ! -d "$FRAME_DIR" ]; then
    echo "错误: 文件夹不存在: $FRAME_DIR"
    exit 1
fi

# 检查文件是否存在
if [ ! -f "$MERGED_DATA_PATH" ]; then
    echo "错误: 合并数据文件不存在: $MERGED_DATA_PATH"
    echo "请先运行 ./run_merge.sh $FRAME_ID 生成合并数据"
    exit 1
fi

if [ ! -f "$CALIB_PATH" ]; then
    echo "错误: 相机内参文件不存在: $CALIB_PATH"
    exit 1
fi

echo "帧数: $FRAME_ID"
echo "合并数据: $MERGED_DATA_PATH"
echo "相机内参: $CALIB_PATH"
if [ -n "$SCRIPT_ARGS" ]; then
    echo "额外参数: $SCRIPT_ARGS"
fi
echo

# 运行Python程序
python3 calibrate_pnp.py "$FRAME_ID" $SCRIPT_ARGS
