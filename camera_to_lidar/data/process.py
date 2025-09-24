#!/usr/bin/env python3
import shutil
from pathlib import Path
import argparse
import os
import re

# 定义相机类型
cameras = ["fisheye-front", "fisheye-left", "fisheye-right", "pinhole-front", "pinhole-back"]

# 时间戳同步相关函数

def extract_timestamp_from_filename(filename):
    """
    从文件名中提取时间戳
    假设文件名格式为: timestamp.png 或 timestamp.pcd
    """
    # 移除文件扩展名
    name_without_ext = os.path.splitext(filename)[0]
    
    # 尝试提取数字时间戳
    timestamp_match = re.match(r'^(\d+(?:\.\d+)?)$', name_without_ext)
    if timestamp_match:
        return float(timestamp_match.group(1))
    
    # 如果是其他格式，尝试提取数字部分
    numbers = re.findall(r'\d+(?:\.\d+)?', name_without_ext)
    if numbers:
        return float(numbers[0])
    
    return None

def load_files_with_timestamps(directory, extension):
    """
    加载目录中指定扩展名的文件，并提取时间戳
    返回: [(时间戳, 文件路径), ...]
    """
    files_with_timestamps = []
    
    if not os.path.exists(directory):
        print(f"目录 {directory} 不存在")
        return files_with_timestamps
    
    for filename in os.listdir(directory):
        if filename.lower().endswith(extension.lower()):
            timestamp = extract_timestamp_from_filename(filename)
            if timestamp is not None:
                file_path = os.path.join(directory, filename)
                files_with_timestamps.append((timestamp, file_path))
    
    # 按时间戳排序
    files_with_timestamps.sort(key=lambda x: x[0])
    return files_with_timestamps

def sync_files_by_timestamp(png_files, pcd_files, time_tolerance=0.03):
    """
    根据时间戳同步PNG和PCD文件
    参数:
        png_files: [(时间戳, 文件路径), ...]
        pcd_files: [(时间戳, 文件路径), ...]
        time_tolerance: 时间容差（秒）
    返回: [(png_path, pcd_path), ...] 同步的文件对
    """
    synchronized_pairs = []
    png_idx, pcd_idx = 0, 0
    
    while png_idx < len(png_files) and pcd_idx < len(pcd_files):
        png_time, png_path = png_files[png_idx]
        pcd_time, pcd_path = pcd_files[pcd_idx]
        
        # 检查时间戳是否在容差范围内
        if abs(png_time - pcd_time) <= time_tolerance:
            print(f"匹配文件对: PNG时间戳 {png_time}, PCD时间戳 {pcd_time}")
            synchronized_pairs.append((png_path, pcd_path))
            png_idx += 1
            pcd_idx += 1
        elif png_time < pcd_time:
            png_idx += 1
        else:
            pcd_idx += 1
    
    return synchronized_pairs

def transfer_synchronized_files(synchronized_pairs, target_dir1, target_dir2):
    """
    复制同步的文件对到目标目录，并按顺序重命名
    """
    # 确保目标目录存在
    os.makedirs(target_dir1, exist_ok=True)
    os.makedirs(target_dir2, exist_ok=True)
    
    # 清空目标目录
    for target_dir in [target_dir1, target_dir2]:
        for item in Path(target_dir).iterdir():
            if (item.name == "initial_error.txt" and 
                    item.parent.name == "auto"):
                    print(f"保留文件: {item}")
                    continue
            if item.is_dir():
                print(f"删除目录: {item}")
                shutil.rmtree(item)
            else:
                item.unlink()
        print(f"成功清空目标目录: {target_dir}")
    
    # 复制并重命名文件
    for idx, (png_path, pcd_path) in enumerate(synchronized_pairs):
        # 生成新的文件名
        new_png_name = f"{idx:04d}.png"
        new_pcd_name = f"{idx:04d}.pcd"
        
        target_png_path = os.path.join(target_dir1, new_png_name)
        target_pcd_path = os.path.join(target_dir2, new_pcd_name)
        
        # 复制文件
        shutil.copy2(png_path, target_png_path)
        shutil.copy2(pcd_path, target_pcd_path)
        
        print(f"复制文件对 {idx}: {os.path.basename(png_path)} -> {new_png_name}, {os.path.basename(pcd_path)} -> {new_pcd_name}")
    
    print(f"成功复制 {len(synchronized_pairs)} 个同步文件对")

def transfer(source_dir, target_dir):
    """复制文件夹内容"""
    source_path = Path(source_dir)
    target_path = Path(target_dir)
    
    if not source_path.exists():
        print(f"源目录 {source_dir} 不存在，跳过复制。")
        return
    
    # 确保目标目录存在
    target_path.mkdir(parents=True, exist_ok=True)
    
    # 删除目标目录中的内容，但保留目录本身
    if target_path.exists():
        for item in target_path.iterdir():
            if (item.name == "initial_error.txt" and 
                    item.parent.name == "auto"):
                    print(f"保留文件: {item}")
                    continue
            if item.is_dir():
                print(f"删除目录: {item}")
                shutil.rmtree(item)
        print(f"成功清空目标目录内容: {target_path}")

    
    for item in source_path.iterdir():
        target_item = target_path / item.name
        
        # 复制
        if item.is_dir():
            shutil.copytree(item, target_item)
        else:
            shutil.copy2(item, target_item)
    
    print(f"已将 {source_dir} 的内容复制到 {target_dir}")

def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='相机图像去畸变处理工具')
    parser.add_argument('--sorts', choices=['pinhole-back', 'pinhole-front', 'fisheye-front', 'fisheye-left', 'fisheye-right'], default='fisheye-front',
                        help='相机类型选择')
    parser.add_argument('--dir', default='./0923pcdpnp',
                        help='数据源目录路径')
    parser.add_argument('--sync', action='store_true',
                        help='是否启用时间戳同步模式')
    parser.add_argument('--time-tolerance', type=float, default=0.03,
                        help='时间同步容差（秒），默认0.03秒')
    return parser.parse_args()

if __name__ == "__main__":
    # 解析命令行参数
    args = parse_arguments()
    cameras_name={
        'fisheye-front': {
            'source_dir1': f"{args.dir}/output_png4",
            'source_dir2': f"{args.dir}/front",
            'target_dir1': "fisheye-front/images",
            'target_dir2': "fisheye-front/pointclouds"
        },
        'fisheye-left': {
            'source_dir1': f"{args.dir}/output_png5",
            'source_dir2': f"{args.dir}/back",
            'target_dir1': "fisheye-left/images",
            'target_dir2': "fisheye-left/pointclouds"
        },
        'fisheye-right': {
            'source_dir1': f"{args.dir}/output_png7",
            'source_dir2': f"{args.dir}/front",
            'target_dir1': "fisheye-right/images",
            'target_dir2': "fisheye-right/pointclouds"
        },
        'pinhole-back': {
            'source_dir1': f"{args.dir}/output_png6",
            'source_dir2': f"{args.dir}/back",
            'target_dir1': "pinhole-back/images",
            'target_dir2': "pinhole-back/pointclouds"
        },
        'pinhole-front': {
            'source_dir1': f"{args.dir}/output_png0",
            'source_dir2': f"{args.dir}/front",
            'target_dir1': "pinhole-front/images",
            'target_dir2': "pinhole-front/pointclouds"
        }

    }
    camera = args.sorts
    source_dir1 = cameras_name[camera]['source_dir1']  # PNG图像目录
    source_dir2 = cameras_name[camera]['source_dir2']  # PCD点云目录
    target_dir1 = cameras_name[camera]['target_dir1']
    target_dir2 = cameras_name[camera]['target_dir2']
    
    if args.sync:
        print(f"启用时间戳同步模式，时间容差: {args.time_tolerance}秒")
        print(f"处理相机类型: {camera}")
        print(f"PNG源目录: {source_dir1}")
        print(f"PCD源目录: {source_dir2}")
        
        # 加载PNG和PCD文件及其时间戳
        png_files = load_files_with_timestamps(source_dir1, '.png')
        pcd_files = load_files_with_timestamps(source_dir2, '.pcd')
        
        print(f"找到 {len(png_files)} 个PNG文件和 {len(pcd_files)} 个PCD文件")
        
        if len(png_files) == 0 or len(pcd_files) == 0:
            print("警告: 没有找到足够的文件进行同步")
            exit(1)
        
        # 同步文件
        synchronized_pairs = sync_files_by_timestamp(png_files, pcd_files, args.time_tolerance)
        print(f"成功同步 {len(synchronized_pairs)} 个文件对")
        
        if len(synchronized_pairs) == 0:
            print("警告: 没有找到匹配的文件对")
            exit(1)
        
        # 传输同步的文件
        transfer_synchronized_files(synchronized_pairs, target_dir1, target_dir2)
        
    else:
        print("使用普通复制模式")
        transfer(source_dir1, target_dir1)
        transfer(source_dir2, target_dir2)
    
    print(f"处理完成: {camera}")