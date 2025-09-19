#!/usr/bin/env python3
import shutil
from pathlib import Path
import argparse

# 定义相机类型
cameras = ["fisheye-front", "fisheye-left", "fisheye-right", "pinhole-front", "pinhole-back"]

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
    parser.add_argument('--sorts', choices=['pinhole-back', 'pinhole-front', 'fisheye-front', 'fisheye-left',   'fisheye-right'], default='fisheye-front',
                        help='图片选择模式: random(随机选择), select(选择特定帧)')
    return parser.parse_args()

if __name__ == "__main__":
    # 解析命令行参数
    args = parse_arguments()
    cameras_name={
        'pinhole-back': {
            'source_dir1': "pinhole-back/auto-calib",
            'source_dir2': "pinhole-back/manual-calib",
            'target_dir1': "../lidar2camera/auto_calib/data",
            'target_dir2': "../lidar2camera/manual_calib/data"
        },
        'pinhole-front': {
            'source_dir1': "pinhole-front/auto-calib",
            'source_dir2': "pinhole-front/manual-calib",
            'target_dir1': "../lidar2camera/auto_calib/data",
            'target_dir2': "../lidar2camera/manual_calib/data"
        },
        'fisheye-front': {
            'source_dir1': "fisheye-front/auto-calib",
            'source_dir2': "fisheye-front/manual-calib",
            'target_dir1': "../lidar2camera/auto_calib/data",
            'target_dir2': "../lidar2camera/manual_calib/data"
        },
        'fisheye-left': {
            'source_dir1': "fisheye-left/auto-calib",
            'source_dir2': "fisheye-left/manual-calib",
            'target_dir1': "../lidar2camera/auto_calib/data",
            'target_dir2': "../lidar2camera/manual_calib/data"
        },
        'fisheye-right': {
            'source_dir1': "fisheye-right/auto-calib",
            'source_dir2': "fisheye-right/manual-calib",
            'target_dir1': "../lidar2camera/auto_calib/data",
            'target_dir2': "../lidar2camera/manual_calib/data"
        }
    }
    camera=args.sorts
    source_dir1 = cameras_name[camera]['source_dir1']
    source_dir2 = cameras_name[camera]['source_dir2']
    target_dir1 = cameras_name[camera]['target_dir1']
    target_dir2 = cameras_name[camera]['target_dir2']
    
    transfer(source_dir1, target_dir1)
    transfer(source_dir2, target_dir2)