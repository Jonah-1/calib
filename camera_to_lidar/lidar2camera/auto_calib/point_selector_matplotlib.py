#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
交互式点选择工具 - Matplotlib版本
同时操作图片和点云，选取对应点并保存坐标
"""

import cv2
import numpy as np
import open3d as o3d
import json
import argparse
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import matplotlib


class PointSelector:
    def __init__(self, image_path, output_path="image.json"):
        """
        初始化图片点选择器
        
        Args:
            image_path: 图片路径
            output_path: 输出JSON文件路径
        """
        self.image_path = image_path
        self.output_path = output_path
        
        # 加载图片
        self.image = cv2.imread(image_path)
        if self.image is None:
            raise ValueError(f"无法加载图片: {image_path}")
        
        # OpenCV使用BGR，matplotlib使用RGB
        self.image_rgb = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        
        # 存储选择的点
        self.image_points = []  # 2D图像点 (x, y)
        
        # Matplotlib相关
        self.fig = None
        self.ax = None
        self.point_markers = []
        
        # 颜色列表（用于区分不同的点对）
        self.colors = [
            '#00FF00',  # 绿色
            '#FF0000',  # 红色
            '#0000FF',  # 蓝色
            '#FFFF00',  # 黄色
            '#FF00FF',  # 品红
            '#00FFFF',  # 青色
            '#FFA500',  # 橙色
            '#800080',  # 紫色
        ]
        
        print("\n" + "="*60)
        print("Image Point Selector - Matplotlib Version")
        print("="*60)
        print("\nInstructions:")
        print("1. Shift + Left-click on the image to select points")
        print("2. Press 's' to save all image points")
        print("3. Press 'u' to undo the last point")
        print("4. Press 'c' to clear all points")
        print("5. Close the window or press 'q' to exit")
        print("="*60 + "\n")
    
    def on_image_click(self, event):
        """图片鼠标点击事件处理"""
        if event.inaxes != self.ax:
            return
        
        # 检查是否为Shift+左键
        if event.button == 1 and event.key == 'shift':  # Shift+左键
            x, y = int(event.xdata), int(event.ydata)
            
            # 检查点击是否在图像范围内
            if 0 <= x < self.image_rgb.shape[1] and 0 <= y < self.image_rgb.shape[0]:
                self.image_points.append((x, y))
                print(f"\nImage point #{len(self.image_points)} selected (Shift+Click): ({x}, {y})")
                
                # 更新显示
                self.update_display()
        elif event.button == 1:  # 普通左键点击
            print("\n提示: 请使用 Shift+左键 来选择点")
    
    def on_key_press(self, event):
        """键盘按键事件处理"""
        if event.key == 's':
            self.save_image_points()
        elif event.key == 'u':
            self.undo_last()
        elif event.key == 'c':
            print("\nAre you sure to clear all points? Press 'c' again to confirm, or any other key to cancel")
            self.clear_confirm = True
        elif event.key == 'q':
            print("\nUser exited the program")
            plt.close(self.fig)
        elif hasattr(self, 'clear_confirm') and self.clear_confirm:
            if event.key == 'c':
                self.clear_all()
            self.clear_confirm = False
    
    
    def update_display(self):
        """更新图片显示，显示所有已选择的点"""
        # 清除之前的标记
        for marker in self.point_markers:
            marker.remove()
        self.point_markers.clear()
        
        # 重新绘制图片
        self.ax.clear()
        self.ax.imshow(self.image_rgb)
        self.ax.set_title(f'Image Point Selector - {len(self.image_points)} points | s:Save u:Undo c:Clear q:Quit')
        self.ax.axis('off')
        
        # 绘制所有点
        for i, img_pt in enumerate(self.image_points):
            color = self.colors[i % len(self.colors)]
            
            # 绘制圆点
            circle = Circle(img_pt, 8, color=color, fill=True, alpha=0.8)
            self.ax.add_patch(circle)
            self.point_markers.append(circle)
            
            # 绘制外圈
            circle_outer = Circle(img_pt, 10, color='white', fill=False, linewidth=2)
            self.ax.add_patch(circle_outer)
            self.point_markers.append(circle_outer)
            
            # 绘制编号
            text = self.ax.text(img_pt[0]+15, img_pt[1]-15, f"{i+1}", 
                               color=color, fontsize=12, weight='bold',
                               bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7))
            self.point_markers.append(text)
        
        self.fig.canvas.draw()
    
    def save_image_points(self):
        """保存图片点到JSON文件"""
        if len(self.image_points) == 0:
            print("\nWarning: No image points to save!")
            return
        
        data = {
            "image_path": str(self.image_path),
            "num_points": len(self.image_points),
            "image_points": []
        }
        
        for i, img_pt in enumerate(self.image_points):
            point_data = {
                "id": i + 1,
                "x": int(img_pt[0]),
                "y": int(img_pt[1])
            }
            data["image_points"].append(point_data)
        
        # 保存到文件
        with open(self.output_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        
        print(f"\n✓ Saved {len(self.image_points)} image points to: {self.output_path}")
        
    
    def undo_last(self):
        """撤销最后一个点"""
        if len(self.image_points) > 0:
            removed_img = self.image_points.pop()
            print(f"\n✓ Undid last image point: {removed_img}")
            self.update_display()
        else:
            print("\nWarning: No points to undo!")
    
    def clear_all(self):
        """清除所有点"""
        if len(self.image_points) > 0:
            self.image_points.clear()
            print("\n✓ Cleared all image points")
            self.update_display()
        else:
            print("\nInfo: No image points to clear")
    
    def run(self):
        """运行主循环"""
        # 使用TkAgg后端
        matplotlib.use('TkAgg')
        
        # 创建matplotlib窗口
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.fig.canvas.manager.set_window_title('Point Selector Tool')
        
        # 显示图片
        self.update_display()
        
        # 连接事件
        self.fig.canvas.mpl_connect('button_press_event', self.on_image_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        print("\n✓ Window created successfully!")
        print("Waiting for user interaction...")
        
        # 显示窗口
        plt.show()
        
        # 退出前询问是否保存
        if len(self.image_points) > 0:
            response = input(f"\nSave {len(self.image_points)} image points before exiting? (y/n): ")
            if response.lower() == 'y':
                self.save_image_points()


def main():
    parser = argparse.ArgumentParser(description='Image Point Selector - Matplotlib Version')
    parser.add_argument('image', type=str, help='Image file path')
    parser.add_argument('-o', '--output', type=str, default='image.json',
                       help='Output file path (default: image.json)')
    
    args = parser.parse_args()
    
    try:
        selector = PointSelector(args.image, args.output)
        selector.run()
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == '__main__':
    exit(main())

