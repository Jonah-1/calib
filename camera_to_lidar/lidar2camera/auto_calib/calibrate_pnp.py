#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
使用PnP算法计算雷达到相机的外参矩阵
读取merged_data.json文件，使用3D-2D对应点进行标定
"""

import json
import numpy as np
import cv2
import argparse
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class PnPCalibrator:
    def __init__(self, frame_id, data_dir="data", use_tr_initial=False):
        """
        初始化PnP标定器
        
        Args:
            frame_id: 帧数ID
            data_dir: 数据目录
            use_tr_initial: 是否使用calib.txt中的Tr外参作为初始值
        """
        self.frame_id = frame_id
        self.data_dir = data_dir
        self.use_tr_initial = use_tr_initial
        self.merged_data_path = f"{data_dir}/{frame_id}/merged_data.json"
        
        # 存储数据
        self.object_points = []  # 3D点 (N, 3)
        self.image_points = []  # 2D点 (N, 2)
        self.labels = []  # 标签
        
        # 标定结果
        self.rvec = None  # 旋转向量
        self.tvec = None  # 平移向量
        self.rotation_matrix = None  # 旋转矩阵
        self.translation_vector = None  # 平移向量
        self.extrinsic_matrix = None  # 外参矩阵
        
        # Tr初始值
        self.tr_rvec_initial = None  # Tr的旋转向量初始值
        self.tr_tvec_initial = None  # Tr的平移向量初始值
        
        # 图像数据
        self.image = None  # 原始图像
        self.image_path = f"{data_dir}/{frame_id}/{frame_id}.png"
        
        print(f"PnP标定器初始化")
        print(f"帧数: {frame_id}")
        print(f"数据文件: {self.merged_data_path}")
    
    def load_merged_data(self):
        """加载merged_data.json数据"""
        print(f"\n加载合并数据: {self.merged_data_path}")
        
        try:
            with open(self.merged_data_path, 'r') as f:
                data = json.load(f)
            
            if 'merged_points' not in data:
                print("错误: merged_data.json中缺少merged_points字段")
                return False
            
            merged_points = data['merged_points']
            print(f"找到 {len(merged_points)} 个对应点")
            
            # 提取3D和2D坐标
            for point in merged_points:
                if 'label' in point and 'coordinates_3d' in point and 'coordinates_2d' in point:
                    label = point['label']
                    coords_3d = point['coordinates_3d']
                    coords_2d = point['coordinates_2d']
                    
                    # 3D坐标 (x, y, z)
                    point_3d = np.array([
                        coords_3d['x'],
                        coords_3d['y'],
                        coords_3d['z']
                    ], dtype=np.float32)
                    
                    # 2D坐标 (x, y)
                    point_2d = np.array([
                        coords_2d['x'],
                        coords_2d['y']
                    ], dtype=np.float32)
                    
                    self.object_points.append(point_3d)
                    self.image_points.append(point_2d)
                    self.labels.append(label)
                    
                    print(f"  点 {label}: 3D({coords_3d['x']:.3f}, {coords_3d['y']:.3f}, {coords_3d['z']:.3f}) -> 2D({coords_2d['x']}, {coords_2d['y']})")
            
            self.object_points = np.array(self.object_points)
            self.image_points = np.array(self.image_points)
            
            print(f"成功加载 {len(self.object_points)} 个对应点")
            return True
            
        except FileNotFoundError:
            print(f"错误: 找不到文件 {self.merged_data_path}")
            return False
        except json.JSONDecodeError as e:
            print(f"错误: JSON格式错误: {e}")
            return False
        except Exception as e:
            print(f"错误: 加载数据失败: {e}")
            return False
    
    def load_image(self):
        """加载原始图像"""
        print(f"\n加载图像: {self.image_path}")
        
        try:
            self.image = cv2.imread(self.image_path)
            if self.image is None:
                print(f"错误: 无法加载图像 {self.image_path}")
                return False
            
            print(f"图像尺寸: {self.image.shape[1]}x{self.image.shape[0]}")
            return True
            
        except Exception as e:
            print(f"错误: 加载图像失败: {e}")
            return False
            print(f"错误: JSON格式错误: {e}")
            return False
        except Exception as e:
            print(f"错误: 加载数据失败: {e}")
            return False
    
    def load_camera_intrinsics(self):
        """加载相机内参"""
        calib_path = f"{self.data_dir}/{self.frame_id}/calib.txt"
        print(f"\n加载相机内参: {calib_path}")
        
        try:
            with open(calib_path, 'r') as f:
                lines = f.readlines()
            
            print(f"calib.txt内容:")
            for i, line in enumerate(lines):
                print(f"  第{i+1}行: {line.strip()}")
            
            # 解析P2矩阵格式: "P2: fx 0 cx 0 fy cy 0 0 1"
            for line in lines:
                line = line.strip()
                if line.startswith('P2:'):
                    # 提取P2矩阵的数值
                    values = line.split(':')[1].strip().split()
                    if len(values) >= 9:
                        fx = float(values[0])
                        fy = float(values[4])
                        cx = float(values[2])
                        cy = float(values[5])
                        
                        self.camera_matrix = np.array([
                            [fx, 0, cx],
                            [0, fy, cy],
                            [0, 0, 1]
                        ], dtype=np.float32)
                        
                        # 畸变系数 (假设为0)
                        self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)
                        
                        print(f"解析P2矩阵:")
                        print(f"  fx: {fx}")
                        print(f"  fy: {fy}")
                        print(f"  cx: {cx}")
                        print(f"  cy: {cy}")
                        print(f"内参矩阵:\n{self.camera_matrix}")
                        
                        return True
                    else:
                        print(f"错误: P2矩阵格式不正确，需要至少9个数值")
                        return False
            
            # 如果没有找到P2格式，尝试简单数值格式
            if len(lines) >= 3:
                try:
                    fx = float(lines[0].strip())
                    fy = float(lines[1].strip())
                    cx = float(lines[2].strip())
                    cy = float(lines[3].strip()) if len(lines) > 3 else cx
                    
                    self.camera_matrix = np.array([
                        [fx, 0, cx],
                        [0, fy, cy],
                        [0, 0, 1]
                    ], dtype=np.float32)
                    
                    # 畸变系数 (假设为0)
                    self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)
                    
                    print(f"解析简单格式:")
                    print(f"  fx: {fx}")
                    print(f"  fy: {fy}")
                    print(f"  cx: {cx}")
                    print(f"  cy: {cy}")
                    print(f"内参矩阵:\n{self.camera_matrix}")
                    
                    return True
                except ValueError:
                    pass
            
            print("错误: 无法解析相机内参文件格式")
            return False
            
        except FileNotFoundError:
            print(f"错误: 找不到相机内参文件 {calib_path}")
            return False
        except Exception as e:
            print(f"错误: 加载相机内参失败: {e}")
            return False
    
    def parse_tr_matrix(self):
        """解析calib.txt中的Tr矩阵并转换为rvec和tvec初始值"""
        if not self.use_tr_initial:
            return True
            
        calib_path = f"{self.data_dir}/{self.frame_id}/calib.txt"
        print(f"\n解析Tr矩阵作为初始值: {calib_path}")
        
        try:
            with open(calib_path, 'r') as f:
                lines = f.readlines()
            
            # 查找Tr行
            tr_line = None
            for line in lines:
                if line.strip().startswith('Tr:'):
                    tr_line = line.strip()
                    break
            
            if tr_line is None:
                print("警告: 未找到Tr行，将不使用初始值")
                return True
            
            # 解析Tr矩阵: "Tr: r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz"
            values = tr_line.split(':')[1].strip().split()
            if len(values) < 12:
                print(f"错误: Tr行格式不正确，需要12个数值，实际得到{len(values)}个")
                return False
            
            # 构建4x4变换矩阵
            tr_matrix = np.array([
                [float(values[0]), float(values[1]), float(values[2]), float(values[3])],
                [float(values[4]), float(values[5]), float(values[6]), float(values[7])],
                [float(values[8]), float(values[9]), float(values[10]), float(values[11])],
                [0.0, 0.0, 0.0, 1.0]
            ], dtype=np.float32)
            
            # 提取旋转矩阵和平移向量
            rotation_matrix = tr_matrix[:3, :3]
            translation_vector = tr_matrix[:3, 3]
            
            # 转换为旋转向量
            self.tr_rvec_initial, _ = cv2.Rodrigues(rotation_matrix)
            self.tr_tvec_initial = translation_vector.reshape(3, 1)
            
            print(f"Tr矩阵解析成功:")
            print(f"  旋转矩阵:\n{rotation_matrix}")
            print(f"  平移向量: {translation_vector}")
            print(f"  旋转向量: {self.tr_rvec_initial.flatten()}")
            print(f"  平移向量: {self.tr_tvec_initial.flatten()}")
            
            return True
            
        except FileNotFoundError:
            print(f"错误: 找不到文件 {calib_path}")
            return False
        except Exception as e:
            print(f"错误: 解析Tr矩阵失败: {e}")
            return False
    
    def calculate_initial_reprojection_error(self):
        """计算Tr初始值的重投影误差"""
        if not self.use_tr_initial or self.tr_rvec_initial is None:
            return None, None
            
        print(f"\n计算Tr初始值的重投影误差...")
        
        try:
            # 使用Tr初始值进行重投影
            projected_points, _ = cv2.projectPoints(
                self.object_points,
                self.tr_rvec_initial,
                self.tr_tvec_initial,
                self.camera_matrix,
                self.dist_coeffs
            )
            
            projected_points = projected_points.reshape(-1, 2)
            
            # 计算误差
            errors = np.linalg.norm(self.image_points - projected_points, axis=1)
            mean_error = np.mean(errors)
            max_error = np.max(errors)
            
            print(f"Tr初始值重投影误差:")
            print(f"  平均误差: {mean_error:.4f} 像素")
            print(f"  最大误差: {max_error:.4f} 像素")
            
            # 显示每个点的误差
            for i, (label, error) in enumerate(zip(self.labels, errors)):
                print(f"  点 {label}: {error:.4f} 像素")
            
            return mean_error, max_error
            
        except Exception as e:
            print(f"错误: 计算Tr初始值重投影误差失败: {e}")
            return None, None
    
    def solve_pnp(self):
        """使用PnP算法求解外参"""
        print(f"\n开始PnP求解...")
        
        if len(self.object_points) < 4:
            print("错误: 至少需要4个对应点进行PnP求解")
            return False
        
        print(f"使用 {len(self.object_points)} 个对应点进行PnP求解")
        
        try:
            # 选择PnP算法和参数
            if self.use_tr_initial and self.tr_rvec_initial is not None:
                print("使用Tr矩阵作为初始值进行PnP求解")
                # 使用迭代算法，需要初始值
                success, rvec, tvec = cv2.solvePnP(
                    self.object_points,
                    self.image_points,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec=self.tr_rvec_initial,
                    tvec=self.tr_tvec_initial,
                    useExtrinsicGuess=True,
                    flags=cv2.SOLVEPNP_ITERATIVE
                )
            else:
                print("使用EPNP算法进行PnP求解")
                # 使用EPNP算法，不需要初始值
                success, rvec, tvec = cv2.solvePnP(
                    self.object_points,
                    self.image_points,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_EPNP
                )
            
            if not success:
                print("错误: PnP求解失败")
                return False
            
            self.rvec = rvec
            self.tvec = tvec
            
            # 转换为旋转矩阵
            self.rotation_matrix, _ = cv2.Rodrigues(rvec)
            self.translation_vector = tvec.flatten()
            
            # 构建4x4外参矩阵
            self.extrinsic_matrix = np.hstack([
                self.rotation_matrix,
                self.translation_vector.reshape(3, 1)
            ])
            # 添加齐次坐标的最后一行 [0, 0, 0, 1]
            self.extrinsic_matrix = np.vstack([
                self.extrinsic_matrix,
                np.array([0, 0, 0, 1])
            ])
            
            print("✓ PnP求解成功!")
            print(f"旋转向量 (rvec): {rvec.flatten()}")
            print(f"平移向量 (tvec): {tvec.flatten()}")
            print(f"旋转矩阵:\n{self.rotation_matrix}")
            print(f"平移向量: {self.translation_vector}")
            print(f"外参矩阵:\n{self.extrinsic_matrix}")
            
            return True
            
        except Exception as e:
            print(f"错误: PnP求解失败: {e}")
            return False
    
    def calculate_reprojection_error(self):
        """计算重投影误差"""
        print(f"\n计算重投影误差...")
        
        try:
            # 重投影2D点
            projected_points, _ = cv2.projectPoints(
                self.object_points,
                self.rvec,
                self.tvec,
                self.camera_matrix,
                self.dist_coeffs
            )
            
            projected_points = projected_points.reshape(-1, 2)
            
            # 计算误差
            errors = np.linalg.norm(self.image_points - projected_points, axis=1)
            mean_error = np.mean(errors)
            max_error = np.max(errors)
            
            print(f"重投影误差:")
            print(f"  平均误差: {mean_error:.4f} 像素")
            print(f"  最大误差: {max_error:.4f} 像素")
            
            # 显示每个点的误差
            for i, (label, error) in enumerate(zip(self.labels, errors)):
                print(f"  点 {label}: {error:.4f} 像素")
            
            return mean_error, max_error, projected_points
            
        except Exception as e:
            print(f"错误: 计算重投影误差失败: {e}")
            return None, None, None
    
    def visualize_projection_points(self, projected_points, output_dir="."):
        """在图像上绘制投影点并保存"""
        print(f"\n绘制投影点可视化图像...")
        
        try:
            # 加载原始图像
            if not Path(self.image_path).exists():
                print(f"错误: 找不到图像文件 {self.image_path}")
                return False
            
            image = cv2.imread(self.image_path)
            if image is None:
                print(f"错误: 无法读取图像文件 {self.image_path}")
                return False
            
            # 创建图像副本用于绘制
            vis_image = image.copy()
            
            # 定义颜色
            original_color = (0, 255, 0)  # 绿色 - 原始点
            projected_color = (0, 0, 255)  # 红色 - 投影点
            line_color = (255, 0, 0)  # 蓝色 - 连接线
            
            # 绘制原始点和投影点
            for i, (label, orig_pt, proj_pt) in enumerate(zip(self.labels, self.image_points, projected_points)):
                # 转换为整数坐标
                orig_x, orig_y = int(orig_pt[0]), int(orig_pt[1])
                proj_x, proj_y = int(proj_pt[0]), int(proj_pt[1])
                
                # 绘制原始点 (绿色圆圈)
                cv2.circle(vis_image, (orig_x, orig_y), 8, original_color, -1)
                cv2.circle(vis_image, (orig_x, orig_y), 10, (255, 255, 255), 2)
                
                # 绘制投影点 (红色圆圈)
                cv2.circle(vis_image, (proj_x, proj_y), 6, projected_color, -1)
                cv2.circle(vis_image, (proj_x, proj_y), 8, (255, 255, 255), 2)
                
                # 绘制连接线
                cv2.line(vis_image, (orig_x, orig_y), (proj_x, proj_y), line_color, 2)
                
                # 添加标签
                label_pos = (orig_x + 15, orig_y - 10)
                cv2.putText(vis_image, f"{label}", label_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(vis_image, f"{label}", label_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 1)
            
            # 添加图例
            legend_y = 50
            cv2.putText(vis_image, "Green: Original Points", (50, legend_y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, original_color, 2)
            cv2.putText(vis_image, "Red: Projected Points", (50, legend_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, projected_color, 2)
            cv2.putText(vis_image, "Blue: Error Vectors", (50, legend_y + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, line_color, 2)
            
            # 保存可视化图像
            output_path = f"{self.frame_id}_projection_visualization.png"
            cv2.imwrite(output_path, vis_image)
            print(f"✓ 投影点可视化图像已保存: {output_path}")
            
            # 同时保存一个更详细的版本，显示误差信息
            detailed_image = vis_image.copy()
            
            # 添加误差信息
            errors = np.linalg.norm(self.image_points - projected_points, axis=1)
            mean_error = np.mean(errors)
            
            info_text = [
                f"Frame: {self.frame_id}",
                f"Points: {len(self.image_points)}",
                f"Mean Error: {mean_error:.2f} pixels",
                f"Max Error: {np.max(errors):.2f} pixels"
            ]
            
            for i, text in enumerate(info_text):
                cv2.putText(detailed_image, text, (50, detailed_image.shape[0] - 150 + i * 25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(detailed_image, text, (50, detailed_image.shape[0] - 150 + i * 25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        
            
            return True
            
        except Exception as e:
            print(f"错误: 绘制投影点失败: {e}")
            return False
    
    def save_calibration_results(self, output_path):
        """保存标定结果"""
        print(f"\n保存标定结果到: {output_path}")
        
        try:
            results = {
                "frame_id": self.frame_id,
                "num_points": len(self.object_points),
                "camera_intrinsics": {
                    "fx": float(self.camera_matrix[0, 0]),
                    "fy": float(self.camera_matrix[1, 1]),
                    "cx": float(self.camera_matrix[0, 2]),
                    "cy": float(self.camera_matrix[1, 2]),
                    "camera_matrix": self.camera_matrix.tolist()
                },
                "lidar_to_camera_extrinsics": {
                    "rotation_vector": self.rvec.flatten().tolist(),
                    "translation_vector": self.tvec.flatten().tolist(),
                    "extrinsic_matrix": self.extrinsic_matrix.tolist()
                }
            }
            
            # 保存JSON文件
            with open(output_path, 'w', encoding='utf-8') as f:
                json.dump(results, f, indent=2, ensure_ascii=False)
            
            print(f"✓ 成功保存标定结果: {output_path}")
            
            return True
            
        except Exception as e:
            print(f"错误: 保存标定结果失败: {e}")
            return False
    
    def run(self, output_path=None):
        """运行完整的标定流程"""
        print("="*60)
        print("PnP标定工具")
        print("="*60)
        
        # 加载数据
        if not self.load_merged_data():
            return False
        
        if not self.load_camera_intrinsics():
            return False
        
        # 解析Tr矩阵作为初始值（如果启用）
        if not self.parse_tr_matrix():
            return False
        
        # 计算Tr初始值的重投影误差（如果使用）
        initial_mean_error, initial_max_error = self.calculate_initial_reprojection_error()
        
        # PnP求解
        if not self.solve_pnp():
            return False
        
        # 计算重投影误差
        mean_error, max_error, projected_points = self.calculate_reprojection_error()
        if mean_error is None:
            return False
        
        # 绘制投影点可视化
        if projected_points is not None:
            self.visualize_projection_points(projected_points, output_dir=".")
        
        # 保存结果
        if output_path is None:
            output_path = f"calibration_results.json"
        
        if not self.save_calibration_results(output_path):
            return False
        
        print("\n" + "="*60)
        print("PnP标定完成!")
        print("="*60)
        print(f"使用 {len(self.object_points)} 个对应点")
        
        # 显示误差对比（如果使用了Tr初始值）
        if initial_mean_error is not None:
            print(f"\n误差对比:")
            print(f"  Tr初始值 - 平均误差: {initial_mean_error:.4f} 像素, 最大误差: {initial_max_error:.4f} 像素")
            print(f"  PnP优化后 - 平均误差: {mean_error:.4f} 像素, 最大误差: {max_error:.4f} 像素")
            
            # 计算改善程度
            mean_improvement = initial_mean_error - mean_error
            max_improvement = initial_max_error - max_error
            print(f"  改善程度 - 平均误差: {mean_improvement:+.4f} 像素, 最大误差: {max_improvement:+.4f} 像素")
        else:
            print(f"平均重投影误差: {mean_error:.4f} 像素")
            print(f"最大重投影误差: {max_error:.4f} 像素")
        
        print(f"输出文件: {output_path}")
        
        return True


def main():
    parser = argparse.ArgumentParser(description='PnP标定工具')
    parser.add_argument('frame_id', type=str, help='帧数ID')
    parser.add_argument('-d', '--data_dir', type=str, default='data',
                       help='数据目录 (默认: data)')
    parser.add_argument('-o', '--output', type=str, default=None,
                       help='输出文件路径 (默认: data/<frame_id>/calibration_results.json)')
    parser.add_argument('--use-tr-initial', action='store_true',
                       help='使用calib.txt中的Tr外参作为PnP求解的初始值')
    
    args = parser.parse_args()
    
    try:
        calibrator = PnPCalibrator(args.frame_id, args.data_dir, args.use_tr_initial)
        success = calibrator.run(args.output)
        return 0 if success else 1
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    exit(main())
