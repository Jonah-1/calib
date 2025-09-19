#!/usr/bin/env python3
import shutil
from pathlib import Path

# 定义相机类型
cameras = ["fisheye-front", "fisheye-left", "fisheye-right", "pinhole-front", "pinhole-back"]

data_path = Path(".")
source_dir = data_path / "masks"

for camera in cameras:
    source_camera = source_dir / camera
    target_camera = data_path / camera
    
    if source_camera.exists():
        print(f"复制 {camera}...")
        
        # 复制所有内容
        for item in source_camera.iterdir():
            target_item = target_camera / item.name
            
            # 删除已存在的目标
            if target_item.exists():
                if target_item.is_dir():
                    shutil.rmtree(target_item)
                else:
                    target_item.unlink()
            
            # 复制
            if item.is_dir():
                shutil.copytree(item, target_item)
            else:
                target_camera.mkdir(exist_ok=True)
                shutil.copy2(item, target_item)
        
        print(f"  完成 {camera}")
    else:
        print(f"  跳过 {camera} (源目录不存在)")

print("所有操作完成")