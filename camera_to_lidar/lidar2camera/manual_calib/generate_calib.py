#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从JSON文件生成calib.txt文件
"""

import json
import os
import sys

def generate_calib_txt(intrinsic_json_path, extrinsic_json_path, output_calib_path):
    """
    从内参和外参JSON文件生成calib.txt文件
    """
    try:
        # 读取内参JSON文件
        with open(intrinsic_json_path, 'r') as f:
            intrinsic_data = json.load(f)
        
        # 读取外参JSON文件
        with open(extrinsic_json_path, 'r') as f:
            extrinsic_data = json.load(f)
        
        # 提取内参矩阵
        camera_matrix = intrinsic_data["camera_matrix"]
        distortion_coeffs = intrinsic_data["distortion_coefficients"]
        
        # 提取外参矩阵
        extrinsic_matrix = extrinsic_data["top_center_lidar-to-center_camera-extrinsic"]["param"]["sensor_calib"]["data"]
        
        # 生成calib.txt内容
        lines = []
        
        # P2行: 内参矩阵展开为12个数字 (3x4矩阵，最后一列补0)
        p2_values = []
        for i in range(3):
            for j in range(3):
                p2_values.append(str(camera_matrix[i][j]))
            p2_values.append("0")  # 添加第4列的0
        lines.append("P2: " + " ".join(p2_values))
        
        # D行: 畸变系数
        d_values = [str(d) for d in distortion_coeffs]
        lines.append("D: " + " ".join(d_values))
        
        # Tr行: 外参矩阵前3行展开为12个数字
        tr_values = []
        for i in range(3):  # 只取前3行
            for j in range(4):
                tr_values.append(str(extrinsic_matrix[i][j]))
        lines.append("Tr: " + " ".join(tr_values))
        
        # 写入calib.txt文件
        with open(output_calib_path, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        
        print(f"成功生成: {output_calib_path}")
        return True
        
    except Exception as e:
        print(f"生成calib.txt失败: {e}")
        return False

def main():
    if len(sys.argv) != 2:
        print("用法: python generate_calib.py <数据目录>")
        print("例如: python generate_calib.py data/0013")
        return
    
    data_dir = sys.argv[1]
    
    # 文件路径
    intrinsic_json = os.path.join(data_dir, "center_camera-intrinsic.json")
    extrinsic_json = os.path.join(data_dir, "top_center_lidar-to-center_camera-extrinsic.json")
    calib_txt = os.path.join(data_dir, "calib.txt")
    
    # 检查文件是否存在
    if not os.path.exists(intrinsic_json):
        print(f"错误: 文件不存在 {intrinsic_json}")
        return
    
    if not os.path.exists(extrinsic_json):
        print(f"错误: 文件不存在 {extrinsic_json}")
        return
    
    # 生成calib.txt
    generate_calib_txt(intrinsic_json, extrinsic_json, calib_txt)

if __name__ == "__main__":
    main()
