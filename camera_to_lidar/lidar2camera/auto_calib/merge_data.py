#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
整合lidar.txt和image.json数据
按标签号排列，包含2D和3D坐标
"""

import json
import argparse
from pathlib import Path


def load_lidar_data(lidar_path):
    """加载lidar.txt数据"""
    print(f"加载Lidar数据: {lidar_path}")
    
    lidar_data = {}
    
    try:
        with open(lidar_path, 'r') as f:
            lines = f.readlines()
        
        for line_num, line in enumerate(lines, 1):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            parts = line.split()
            if len(parts) >= 4:
                try:
                    x = float(parts[0])
                    y = float(parts[1])
                    z = float(parts[2])
                    label = str(int(float(parts[3])))  # 第四列是标签号，转换为整数再转字符串
                    
                    lidar_data[label] = {
                        'x': x,
                        'y': y,
                        'z': z
                    }
                    print(f"  Lidar点 {label}: ({x:.6f}, {y:.6f}, {z:.6f})")
                except ValueError as e:
                    print(f"  警告: 第{line_num}行数据格式错误: {line}")
                    continue
            else:
                print(f"  警告: 第{line_num}行数据不完整: {line}")
                continue
        
        print(f"成功加载 {len(lidar_data)} 个Lidar点")
        return lidar_data
        
    except FileNotFoundError:
        print(f"错误: 找不到文件 {lidar_path}")
        return None
    except Exception as e:
        print(f"错误: 加载Lidar数据失败: {e}")
        return None


def load_image_data(image_path):
    """加载image.json数据"""
    print(f"加载Image数据: {image_path}")
    
    image_data = {}
    
    # 只支持JSON格式
    if not image_path.endswith('.json'):
        print(f"错误: 只支持JSON格式的image文件: {image_path}")
        return None
    
    try:
        with open(image_path, 'r') as f:
            data = json.load(f)
        
        if 'image_points' in data:
            for point in data['image_points']:
                if 'id' in point and 'x' in point and 'y' in point:
                    point_id = str(point['id'])  # 转换为字符串
                    x = point['x']
                    y = point['y']
                    
                    image_data[point_id] = {
                        'x': x,
                        'y': y
                    }
                    print(f"  Image点 {point_id}: ({x}, {y})")
        
        print(f"成功加载 {len(image_data)} 个Image点")
        return image_data
        
    except FileNotFoundError:
        print(f"错误: 找不到文件 {image_path}")
        return None
    except json.JSONDecodeError as e:
        print(f"错误: image.json格式错误: {e}")
        return None
    except Exception as e:
        print(f"错误: 加载Image数据失败: {e}")
        return None


def merge_data(lidar_data, image_data):
    """合并数据"""
    print("\n开始合并数据...")
    
    # 找到共同的标签
    common_labels = set(lidar_data.keys()) & set(image_data.keys())
    
    if not common_labels:
        print("错误: 没有找到共同的标签")
        return None
    
    print(f"找到 {len(common_labels)} 个共同标签: {sorted(common_labels)}")
    
    # 合并数据
    merged_data = {}
    
    for label in sorted(common_labels):
        lidar_point = lidar_data[label]
        image_point = image_data[label]
        
        merged_data[label] = {
            'label': label,
            'coordinates_3d': {
                'x': lidar_point['x'],
                'y': lidar_point['y'],
                'z': lidar_point['z']
            },
            'coordinates_2d': {
                'x': image_point['x'],
                'y': image_point['y']
            }
        }
        
        print(f"  合并标签 {label}:")
        print(f"    3D坐标: ({lidar_point['x']:.6f}, {lidar_point['y']:.6f}, {lidar_point['z']:.6f})")
        print(f"    2D坐标: ({image_point['x']}, {image_point['y']})")
    
    print(f"成功合并 {len(merged_data)} 个点")
    return merged_data


def save_merged_data(merged_data, output_path, lidar_path, image_path):
    """保存合并后的数据"""
    print(f"\n保存合并数据到: {output_path}")
    
    # 准备输出数据
    output_data = {
        "source_files": {
            "lidar_file": str(lidar_path),
            "image_file": str(image_path)
        },
        "num_points": len(merged_data),
        "merged_points": []
    }
    
    # 按标签排序添加合并点
    for label in sorted(merged_data.keys()):
        output_data["merged_points"].append(merged_data[label])
    
    # 保存JSON文件
    try:
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(output_data, f, indent=2, ensure_ascii=False)
        print(f"✓ 成功保存JSON文件: {output_path}")
    except Exception as e:
        print(f"错误: 保存JSON文件失败: {e}")
        return False
    
    
    return True


def main():
    parser = argparse.ArgumentParser(description='合并Lidar和Image数据')
    parser.add_argument('lidar', type=str, help='Lidar文件路径 (.txt)')
    parser.add_argument('image', type=str, help='Image文件路径 (.json)')
    parser.add_argument('-o', '--output', type=str, default='merged_data.json',
                       help='输出文件路径 (默认: merged_data.json)')
    
    args = parser.parse_args()
    
    print("="*60)
    print("Lidar-Image数据合并工具")
    print("="*60)
    
    # 加载数据
    lidar_data = load_lidar_data(args.lidar)
    if lidar_data is None:
        return 1
    
    image_data = load_image_data(args.image)
    if image_data is None:
        return 1
    
    # 合并数据
    merged_data = merge_data(lidar_data, image_data)
    if merged_data is None:
        return 1
    
    # 保存结果
    if not save_merged_data(merged_data, args.output, args.lidar, args.image):
        return 1
    
    print("\n" + "="*60)
    print("数据合并完成!")
    print("="*60)
    print(f"合并了 {len(merged_data)} 个点")
    print(f"输出文件: {args.output}")
    
    return 0


if __name__ == '__main__':
    exit(main())
