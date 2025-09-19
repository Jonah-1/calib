#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将五个相机文件夹的undistorted文件夹内容复制到对应的masks文件夹中
"""

import os
import shutil
from pathlib import Path



def copy_undistorted_to_masks():
    """
    将undistorted文件夹的内容复制到对应的masks文件夹中
    """
    # 定义五个相机的名称
    camera_names = [
        "pinhole-back",
        "pinhole-front", 
        "fisheye-front",
        "fisheye-left",
        "fisheye-right"
    ]
    
    # 基础路径
    base_path = Path(".")
    
    success_count = 0
    total_files_copied = 0
    
    print("\n开始复制undistorted文件夹内容到masks文件夹...")
    print("=" * 60)
    
    for camera_name in camera_names:
        print(f"\n处理相机: {camera_name}")
        
        # 源路径：相机文件夹下的undistorted文件夹
        source_dir1 = base_path / camera_name / "undistorted"
        source_dir2 = base_path / camera_name / "masks"
        
        # 目标路径：masks文件夹下对应相机的undistorted文件夹
        target_dir1 = base_path / "masks" / camera_name / "undistorted"
        target_dir2 = base_path / "masks" / camera_name / "masks"
        
        print(f"源目录: {source_dir1}")
        print(f"目标目录: {target_dir1}")
        print(f"源目录: {source_dir2}")
        print(f"目标目录: {target_dir2}")
        # 检查源目录是否存在
        if not source_dir1.exists():
            print(f"❌ 警告: 源目录不存在 - {source_dir1}")
            continue
            
        # 检查源目录是否为空
        source_files1 = list(source_dir1.glob("*"))
        source_files2 = list(source_dir2.glob("*"))
        if not source_files1:
            print(f"⚠️  警告: 源目录为空 - {source_dir1}")
            continue
            
        try:
            # 创建目标目录（如果不存在）
            target_dir1.mkdir(parents=True, exist_ok=True)
            target_dir2.mkdir(parents=True, exist_ok=True)
            print(f"✅ 目标目录已准备: {target_dir1}")
            
            # 复制所有文件
            files_copied = 0
            for source_file in source_files1:
                if source_file.is_file():
                    target_file = target_dir1 / source_file.name
                    
                    # 复制文件
                    shutil.copy2(source_file, target_file)
                    files_copied += 1
                    print(f"   📄 已复制: {source_file.name}")
            for source_item in source_files2:
                if source_item.is_dir():
                    target_subdir = target_dir2 / source_item.name
                    
                    # 复制整个文件夹
                    shutil.copytree(source_item, target_subdir)
                    files_copied += 1
                    print(f"   📁 已复制文件夹: {source_item.name}")
            
            print(f"✅ {camera_name} 复制完成，共复制 {files_copied} 个文件")
            success_count += 1
            total_files_copied += files_copied
            
        except Exception as e:
            print(f"❌ 复制 {camera_name} 时出错: {str(e)}")
    
    print("\n" + "=" * 60)
    print(f"复制任务完成!")
    print(f"成功处理的相机数量: {success_count}/{len(camera_names)}")
    print(f"总共复制的文件数量: {total_files_copied}")
    
    if success_count == len(camera_names):
        print("🎉 所有相机的undistorted文件都已成功复制到masks文件夹!")
    else:
        print(f"⚠️  有 {len(camera_names) - success_count} 个相机的文件复制失败或跳过")

def main():
    """主函数"""
    try:
        # 切换到数据目录
        script_dir = Path(__file__).parent
        os.chdir(script_dir)
        
        print(f"当前工作目录: {os.getcwd()}")
        
        # 首先清空masks文件夹中的undistorted文件夹
        masks_dir = Path(".") / "masks"
        if masks_dir.exists():
            # 递归删除masks文件夹中的所有内容
            shutil.rmtree(masks_dir)
            print("✅ 已完全删除masks文件夹及其所有内容")
        
        # 然后执行复制操作
        copy_undistorted_to_masks()
        
    except Exception as e:
        print(f"❌ 程序执行出错: {str(e)}")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())
