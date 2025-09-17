#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import shutil
import argparse
import json
from pathlib import Path
from collections import defaultdict

def read_camera_parameters(param_file):
    """从参数文件中读取相机内参和畸变系数"""
    params = {}
    with open(param_file, 'r', encoding='utf-8') as f:
        lines = f.readlines()
        
    for line in lines:
        # 使用冒号分割键值
        if ':' in line:
            key, value = line.split(':', 1)
            key = key.strip()
            value = value.strip()
            
            # 忽略值为'/'或'null'的参数
            if value != '/' and value != 'null' and value != '':
                try:
                    # 尝试将值转换为浮点数
                    params[key] = float(value)
                except ValueError:
                    # 如果不能转换为浮点数，则保存为字符串
                    params[key] = value
    
    return params

def organize_files(camera_folders,sub_folders):

    for camera_folder in camera_folders:
        auto_calib_dir = os.path.join(camera_folder, "auto-calib")
        mannual_calib_dir = os.path.join(camera_folder, "mannual-calib")

        #清空原来的数据    
        if os.path.exists(auto_calib_dir):
            shutil.rmtree(auto_calib_dir)
            print(f"已清空: {auto_calib_dir}")
            os.makedirs(auto_calib_dir)
        else:
            os.makedirs(auto_calib_dir)
            
        if os.path.exists(mannual_calib_dir):
            shutil.rmtree(mannual_calib_dir)
            print(f"已清空: {mannual_calib_dir}")
            os.makedirs(mannual_calib_dir)
        else:
            os.makedirs(mannual_calib_dir)

        # 构建当前camera_folder的完整sub_folder路径
        camera_sub_folders = [os.path.join(camera_folder, sf) for sf in sub_folders]
        
        basenames=[]
        targets={}

        #统计需要创建的文件夹
        for file_path in Path(os.path.join(camera_folder, "undistorted")).glob('*'):
            print(file_path)
            basename=file_path.stem
            basenames.append(basename)
        print(f"共有{len(basenames)}个文件夹被打包")

        #保存需要复制的路径
        for basename in basenames:
            targets[basename] = []  # 初始化列表
            for sub_folder in camera_sub_folders:
                if sub_folder.endswith("masks"):
                    print(Path(sub_folder))
                    for file_path in Path(sub_folder).glob('*'):
                        if(basename==os.path.basename(file_path)):
                            targets[basename].append(file_path)
                else:
                    for file_path in Path(sub_folder).glob('*'):
                        filename=os.path.basename(file_path)
                        if(basename==os.path.splitext(filename)[0]):
                            targets[basename].append(file_path)
        #开始复制    
        for basename in basenames:
            save_auto_folder=os.path.join(auto_calib_dir,basename)
            save_mannual_folder=os.path.join(mannual_calib_dir,basename)
            os.makedirs(save_auto_folder, exist_ok=True)
            os.makedirs(save_mannual_folder, exist_ok=True)
            for file_path in targets[basename]:
                if 'masks' in str(file_path):
                    shutil.copytree(file_path, os.path.join(save_auto_folder, 'masks'))
                else:
                    shutil.copy2(file_path, os.path.join(save_auto_folder, os.path.basename(file_path)))
                    shutil.copy2(file_path, os.path.join(save_mannual_folder, os.path.basename(file_path)))

def copy_json_to_mannualcalib(camera_folders):
    """
    根据每个相机的参数文件，生成包含具体内参的JSON文件到mannual-calib的所有最深层子文件夹中
    """
    intrinsic_file = "center_camera-intrinsic.json"
    extrinsic_file = "top_center_lidar-to-center_camera-extrinsic.json"
    
    # 检查外参文件是否存在（内参将动态生成）
    if not os.path.exists(extrinsic_file):
        print(f"错误: 缺少源外参文件 ({extrinsic_file})！")
        return

    copied_files_count = 0
    processed_subfolders = 0
    
    for camera_folder in camera_folders:
        mannualcalib_dir = os.path.join(camera_folder, "mannual-calib")
        if not os.path.isdir(mannualcalib_dir):
            continue

        # 确定对应的参数文件路径
        param_file_path = f"Parameters/{camera_folder}.txt"
        if not os.path.exists(param_file_path):
            print(f"警告: 未找到参数文件 '{param_file_path}'，跳过 {camera_folder}")
            continue
        
        # 读取相机参数
        try:
            params = read_camera_parameters(param_file_path)
            
            # 提取内参，如果是特殊的pinhole-front，使用硬编码的参数
            # if camera_folder == "pinhole-front":
            #     fx = 1910.3417311410
            #     fy = 1910.3058674355
            #     cx = 1917.7001038394
            #     cy = 1081.4421265044
            # else:
                # fx = params.get("FX", 0.0)
                # fy = params.get("FY", 0.0)
                # cx = params.get("CX", 0.0)
                # cy = params.get("CY", 0.0)
            fx = params.get("FX", 0.0)
            fy = params.get("FY", 0.0)
            cx=672//2
            cy=544//2

            # 构建内参JSON内容
            intrinsic_data = {
                "center_camera-intrinsic": {
                    "sensor_name": "center_camera",
                    "target_sensor_name": "center_camera",
                    "device_type": "camera",
                    "param_type": "intrinsic",
                    "param": {
                        "img_dist_w": 672,
                        "img_dist_h": 544,
                        "cam_K": {
                            "rows": 3,
                            "cols": 3,
                            "type": 6,
                            "continuous": True,
                            "data": [
                                [fx, 0, cx],
                                [0, fy, cy],
                                [0, 0, 1]
                            ]
                        },
                        "cam_dist": {
                            "rows": 1,
                            "cols": 5,
                            "type": 6,
                            "continuous": True,
                            "data": [
                                [0, 0, 0, 0, 0]
                            ]
                        }
                    }
                }
            }
            
        except Exception as e:
            print(f"警告: 读取参数文件 '{param_file_path}' 时出错: {e}")
            # 如果读取失败，使用默认内容
            intrinsic_data = {
                "center_camera-intrinsic": {
                    "sensor_name": "center_camera",
                    "target_sensor_name": "center_camera",
                    "device_type": "camera",
                    "param_type": "intrinsic",
                    "param": {
                        "img_dist_w": 672,
                        "img_dist_h": 544,
                        "cam_K": {
                            "rows": 3,
                            "cols": 3,
                            "type": 6,
                            "continuous": True,
                            "data": [
                                [0.0, 0, 0.0],
                                [0, 0.0, 0.0],
                                [0, 0, 1]
                            ]
                        },
                        "cam_dist": {
                            "rows": 1,
                            "cols": 5,
                            "type": 6,
                            "continuous": True,
                            "data": [
                                [0, 0, 0, 0, 0]
                            ]
                        }
                    },
                    "error": f"无法读取参数文件 {param_file_path}"
                }
            }

        # 遍历时间戳子文件夹
        for subfolder_name in os.listdir(mannualcalib_dir):
            subfolder_path = os.path.join(mannualcalib_dir, subfolder_name)
            if not os.path.isdir(subfolder_path):
                continue
            
            processed_subfolders += 1
            
            # 生成内参JSON文件
            intrinsic_file_path = os.path.join(subfolder_path, intrinsic_file)
            with open(intrinsic_file_path, 'w', encoding='utf-8') as f:
                json.dump(intrinsic_data, f, indent=4, ensure_ascii=False)
            
            # 复制外参JSON文件
            shutil.copy2(extrinsic_file, os.path.join(subfolder_path, extrinsic_file))
            
            copied_files_count += 2
            print(f"已生成JSON文件到: {subfolder_path}")
    
    if processed_subfolders == 0:
        print(f"警告: 在任何相机文件夹下都没有找到 'mannual-calib' 子文件夹！")
    else:
        print(f"已向 {processed_subfolders} 个子文件夹中生成了 {copied_files_count} 个JSON文件")


def copy_calib_to_autocalib(camera_folders):
    """
    根据每个相机的参数文件，生成包含具体内参的calib.txt文件到auto-calib的所有最深层子文件夹中
    """
    copied_files_count = 0
    processed_subfolders = 0
    
    for camera_folder in camera_folders:
        autocalib_dir = os.path.join(camera_folder, "auto-calib")
        if not os.path.isdir(autocalib_dir):
            continue

        # 确定对应的参数文件路径
        param_file_path = f"Parameters/{camera_folder}.txt"
        if not os.path.exists(param_file_path):
            print(f"警告: 未找到参数文件 '{param_file_path}'，跳过 {camera_folder}")
            continue
        
        # 读取相机参数
        try:
            params = read_camera_parameters(param_file_path)
            
            # 提取内参，如果是特殊的pinhole-front，使用硬编码的参数
            # if camera_folder == "pinhole-front":
            #     fx = 1910.3417311410
            #     fy = 1910.3058674355
            #     cx = 1917.7001038394
            #     cy = 1081.4421265044
            # else:
            #     fx = params.get("FX", "N/A")
            #     fy = params.get("FY", "N/A")
            #     cx = params.get("CX", "N/A")
            #     cy = params.get("CY", "N/A")

            #因为不知道软件去畸变后的内参，假设主点在图片中心
            fx = params.get("FX", "N/A")
            fy = params.get("FY", "N/A")
            cx=672//2
            cy=544//2
            # 构建calib.txt内容，按照固定格式
            calib_content = f"P2: {fx} 0 {cx} 0 {fy} {cy} 0 0 1\n"
            calib_content += "D: 0 0 0 0 0\n"
            calib_content += "Tr: -0.994522 -0.00182427 0.104513 4.56915e-05 6.88468e-18 -0.999848 -0.0174524 0.155236 0.104528 -0.0173568 0.99437 0.0173913"
            
        except Exception as e:
            print(f"警告: 读取参数文件 '{param_file_path}' 时出错: {e}")
            # 如果读取失败，使用默认内容
            calib_content = "P2: 0 0 0 0 0 0 0 0 0 0 1\n"
            calib_content += "D: 0 0 0 0 0\n"
            calib_content += "Tr: -0.994522 -0.00182427 0.104513 4.56915e-05 6.88468e-18 -0.999848 -0.0174524 0.155236 0.104528 -0.0173568 0.99437 0.0173913 "

        # 遍历时间戳子文件夹
        for subfolder_name in os.listdir(autocalib_dir):
            subfolder_path = os.path.join(autocalib_dir, subfolder_name)
            if not os.path.isdir(subfolder_path):
                continue

            processed_subfolders += 1
            # 创建calib.txt文件
            calib_file_path = os.path.join(subfolder_path, "calib.txt")
            with open(calib_file_path, 'w', encoding='utf-8') as f:
                f.write(calib_content)
            copied_files_count += 1
            print(f"已生成calib.txt文件到: {subfolder_path}")
    
    if processed_subfolders == 0:
        print(f"警告: 在任何相机文件夹下都没有找到 'auto-calib' 子文件夹！")
    else:
        print(f"已向 {processed_subfolders} 个子文件夹中生成了 {copied_files_count} 个calib.txt文件")


def main():
    parser = argparse.ArgumentParser(description='将源文件夹中的同名文件整理到两个指定文件夹中')
    parser.add_argument('--copy-json-only', action='store_true',
                        help='只复制JSON文件到mannual-calib文件夹的子文件夹中，不执行整理操作')
    
    args = parser.parse_args()
    
    camera_folders = ["fisheye-front", "fisheye-left", "fisheye-right", "pinhole-back", "pinhole-front"]
    sub_folders = ["pointclouds", "undistorted", "masks"]
    
    organize_files(camera_folders,sub_folders)
    copy_calib_to_autocalib(camera_folders)
    copy_json_to_mannualcalib(camera_folders)
    
    src_dirs = []
    for cam_folder in camera_folders:
        for sub_folder in sub_folders:
            path = os.path.join(cam_folder, sub_folder)
            if os.path.isdir(path):
                src_dirs.append(path)

    if not src_dirs:
        print("错误：未在当前目录下找到任何有效的源文件夹（例如 'fisheye-front/pointclouds'）。")
        print("请确保脚本与相机数据文件夹在同一目录下。")
        return

    if args.copy_json_only:
        print(f"开始复制JSON文件到 'mannual-calib' 的所有子文件夹...")
        copy_json_to_mannualcalib(camera_folders)
    else:
        print(f"自动发现的源文件夹: {', '.join(src_dirs)}")
        print(f"目标结构: camera_folder/auto-calib/..., camera_folder/mannual-calib/... ")
        print("开始整理文件...\n")
        

        


if __name__ == "__main__":
    main() 