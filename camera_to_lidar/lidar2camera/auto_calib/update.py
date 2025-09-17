#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
程序功能：将calibration_0.txt中的外参数据替换data目录下所有子文件夹中
top_center_lidar-to-center_camera-extrinsic.json文件的"data"字段
"""

import os
import json
import glob
import re

def parse_calibration_file(calibration_file):
    """
    解析extrinsic.txt文件中的外参矩阵
    返回: 4x4的外参矩阵列表
    """
    try:
        with open(calibration_file, 'r') as f:
            lines = f.readlines()
        
        # 查找"Extrinsic = "行，然后读取接下来的4行矩阵数据
        matrix = []
        reading_matrix = False
        
        for i, line in enumerate(lines):
            line_stripped = line.strip()
            
            # 找到"Extrinsic = "行（可能后面是空的）
            if line_stripped == "Extrinsic =" or line_stripped.startswith("Extrinsic = "):
                reading_matrix = True
                continue
            
            if reading_matrix:
                # 如果遇到非矩阵行（如Roll =），停止读取
                if not line_stripped.startswith('['):
                    break
                
                # 清理和解析矩阵行
                row_str = line_stripped.replace('[', '').replace(']', '').rstrip(',')
                row_values = [float(x) for x in row_str.split(',')]
                matrix.append(row_values)
                
                # 读取完4行后停止
                if len(matrix) == 4:
                    break
        
        if len(matrix) == 4:
            return matrix
        else:
            print("错误：未找到完整的4x4外参矩阵")
            return None
        
    except Exception as e:
        print(f"解析extrinsic文件失败: {e}")
        return None

def find_extrinsic_json_files(data_dir):
    """
    查找data目录下所有子文件夹中的top_center_lidar-to-center_camera-extrinsic.json文件
    """
    pattern = os.path.join(data_dir, "**", "top_center_lidar-to-center_camera-extrinsic.json")
    json_files = glob.glob(pattern, recursive=True)
    return json_files

def find_calib_txt_files(data_dir):
    """
    查找data目录下所有子文件夹中的calib.txt文件
    """
    pattern = os.path.join(data_dir, "**", "calib.txt")
    txt_files = glob.glob(pattern, recursive=True)
    return txt_files

def update_manual_calib_json_file(json_file_path, new_extrinsic_matrix):
    """
    更新JSON文件中的data字段
    """
    try:
        # 直接构建完整的JSON内容
        json_content = """{
    "top_center_lidar-to-center_camera-extrinsic": {
        "sensor_name": "top_center_lidar",
        "target_sensor_name": "center_camera",
        "device_type": "relational",
        "param_type": "extrinsic",
        "param": {
            "time_lag": 0,
            "sensor_calib": {
                "rows": 4,
                "cols": 4,
                "type": 6,
                "continuous": true,
                "data": [
                """ + f"""[{','.join([str(val) for val in new_extrinsic_matrix[0]])}],
                [{','.join([str(val) for val in new_extrinsic_matrix[1]])}],
                [{','.join([str(val) for val in new_extrinsic_matrix[2]])}],
                [{','.join([str(val) for val in new_extrinsic_matrix[3]])}]""" + """
                ]
            }
        }
    }
}"""
        
        # 写入文件
        with open(json_file_path, 'w') as f:
            f.write(json_content)
        
        print(f"成功更新: {json_file_path}")
        return True
        
    except Exception as e:
        print(f"更新文件失败 {json_file_path}: {e}")
        return False
        
def update_auto_calib_txt_file(txt_file_path, new_extrinsic_matrix):
    """
    更新calib.txt文件中Tr:行的数据
    """
    try:
        # 读取txt文件
        with open(txt_file_path, 'r') as f:
            lines = f.readlines()
        
        # 将外参矩阵前三行转换为一行数字（3x4 = 12个数字）
        tr_values = []
        for i in range(3):  # 只取前三行
            for j in range(4):  # 每行四个数字
                tr_values.append(str(new_extrinsic_matrix[i][j]))
        
        # 生成新的Tr行
        new_tr_line = "Tr: " + " ".join(tr_values)
        
        # 查找并替换Tr:行
        updated_lines = []
        for line in lines:
            if line.strip().startswith("Tr:"):
                updated_lines.append(new_tr_line)
            else:
                updated_lines.append(line)
        
        # 写回文件
        with open(txt_file_path, 'w') as f:
            f.writelines(updated_lines)
        
        print(f"成功更新: {txt_file_path}")
        return True
        
    except Exception as e:
        print(f"更新文件失败 {txt_file_path}: {e}")
        return False

def main():
    """
    主函数
    """
    # 设置文件路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    calibration_file = os.path.join(current_dir, "extrinsic.txt")
    data_dir = os.path.join(current_dir, "..", "manual_calib", "data")
    data_dir2 = os.path.join(current_dir, "data")
    
    print(f"标定文件路径: {calibration_file}")
    print(f"数据目录路径: {data_dir}")
    
    # 检查文件是否存在
    if not os.path.exists(calibration_file):
        print(f"错误：标定文件不存在: {calibration_file}")
        return
    
    if not os.path.exists(data_dir):
        print(f"错误：数据目录不存在: {data_dir}")
        return
    
    # 1. 解析calibration_0.txt中的外参矩阵
    print("\n1. 解析外参矩阵...")
    extrinsic_matrix = parse_calibration_file(calibration_file)
    if extrinsic_matrix is None:
        return
    
    print("解析到的外参矩阵:")
    for row in extrinsic_matrix:
        print(row)
    
    # 2. 查找所有JSON文件
    print("\n2. 查找JSON文件...")
    json_files = find_extrinsic_json_files(data_dir)
    print(f"找到 {len(json_files)} 个JSON文件:")
    for json_file in json_files:
        print(f"  - {json_file}")
    
    # 3. 更新所有JSON文件
    print("\n3. 更新JSON文件...")
    success_count = 0
    for json_file in json_files:
        if update_manual_calib_json_file(json_file, extrinsic_matrix):
            success_count += 1

    txt_files = find_calib_txt_files(data_dir2)
    for txt_file in txt_files:
        if update_auto_calib_txt_file(txt_file, extrinsic_matrix):
            success_count += 1
    
    print(f"\n完成！成功更新了 {success_count}/{len(json_files) + len(txt_files)} 个文件")

if __name__ == "__main__":
    main()
