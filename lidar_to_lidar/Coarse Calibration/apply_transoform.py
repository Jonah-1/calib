# FILE: apply_transform.py

import numpy as np
import os
import open3d as o3d
import shutil

class CloudTransformer:
    def __init__(self, pcd_path, transform_path):
        self.pcd_path = pcd_path
        self.transform_path = transform_path
        self.points = None
        self.intensities = None
        self.transform = None
        
    def read_pcd(self):
        """Read PCD file with binary support"""
        try:
            # First try reading as text
            try:
                with open(self.pcd_path, 'r', encoding='utf-8') as f:
                    lines = f.readlines()
            except UnicodeDecodeError:
                # If fails, try reading as binary
                with open(self.pcd_path, 'rb') as f:
                    lines = []
                    while True:
                        line = f.readline()
                        if not line:
                            break
                        try:
                            lines.append(line.decode('utf-8').strip())
                        except UnicodeDecodeError:
                            # Skip binary data lines
                            continue

            points = []
            intensities = []
            data_start = False
            data_binary = False
            
            # Parse header
            for line in lines:
                if 'DATA binary' in line:
                    data_binary = True
                    break
                if data_start and not data_binary:
                    values = line.strip().split()
                    if len(values) >= 4:
                        x, y, z, intensity = float(values[0]), float(values[1]), float(values[2]), float(values[3])
                        # 直接删除无效值
                        if (np.isfinite(x) and np.isfinite(y) and np.isfinite(z) and np.isfinite(intensity)):
                            points.append([x, y, z])
                            intensities.append(intensity)
                if 'DATA ascii' in line:
                    data_start = True

            # Handle binary data if needed
            if data_binary:
                with open(self.pcd_path, 'rb') as f:
                    # Skip header
                    for line in f:
                        if b'DATA binary' in line:
                            break
                    
                    # Read binary data
                    while True:
                        try:
                            # Read 16 bytes (4 floats: x,y,z,intensity)
                            data = f.read(16)
                            if not data or len(data) != 16:
                                break
                            x, y, z, i = np.frombuffer(data, dtype=np.float32)
                            # 直接删除无效值
                            if (np.isfinite(x) and np.isfinite(y) and np.isfinite(z) and np.isfinite(i)):
                                points.append([x, y, z])
                                intensities.append(i)
                        except:
                            break

            self.points = np.array(points)
            self.intensities = np.array(intensities)
            return True

        except Exception as e:
            print(f"Failed to read PCD file: {e}")
            return False
            
    def load_transform(self):
        """Load transformation matrix"""
        try:
            self.transform = np.load(self.transform_path)
            return True
        except Exception as e:
            print(f"Failed to load transform: {e}")
            return False
            
    def apply_transform(self):
        """Apply transformation to points with basic filtering"""
        try:
            # 检查变换矩阵的有效性
            if np.any(np.isnan(self.transform)) or np.any(np.isinf(self.transform)):
                print("警告：变换矩阵包含无效值！")
                print(f"变换矩阵:\n{self.transform}")
                return False
            
            # 应用变换前，使用 np.errstate 抑制警告
            points_h = np.hstack((self.points, np.ones((len(self.points), 1))))
            
            # 在矩阵乘法时抑制无效值警告
            with np.errstate(invalid='ignore', over='ignore'):
                transformed_points_h = points_h @ self.transform.T
                transformed_points = transformed_points_h[:, :3]

            # 1. 基础过滤：移除无效值（原始数据已在读取时过滤，这里主要处理变换后可能产生的无效值）
            valid_mask = ~np.any(np.isnan(transformed_points) | np.isinf(transformed_points), axis=1)

            # # 2. 距离过滤：使用更严格的阈值
            # distances = np.linalg.norm(transformed_points - self.points, axis=1)
            # mean_dist = np.mean(distances[valid_mask])  # 只用有效点计算均值
            # std_dist = np.std(distances[valid_mask])    # 只用有效点计算标准差
            # distance_mask = distances < (mean_dist + std_dist)  

            # # 3. 空间范围过滤
            # valid_points = transformed_points[valid_mask & distance_mask]
            # coord_ranges = np.ptp(valid_points, axis=0)  # 每个维度的范围
            # range_masks = []
            
            # for i in range(3):
            #     mean_coord = np.mean(transformed_points[:, i][valid_mask & distance_mask])
            #     std_coord = np.std(transformed_points[:, i][valid_mask & distance_mask])
            #     range_masks.append(
            #         (transformed_points[:, i] > mean_coord - 2*std_coord) & 
            #         (transformed_points[:, i] < mean_coord + 2*std_coord)
            #     )
            # spatial_mask = np.all(range_masks, axis=0)

            # # 组合所有过滤条件
            # final_mask = valid_mask & distance_mask & spatial_mask
            final_mask = valid_mask
            valid_indices = np.where(final_mask)[0]

            # 应用过滤
            self.points = transformed_points[valid_indices]
            self.intensities = self.intensities[valid_indices]

            print(f"移除的点数: {len(transformed_points) - len(self.points)}")
            print(f"剩余点数: {len(self.points)}")

            return True

        except Exception as e:
            print(f"变换应用失败: {e}")
            return False

            
    def save_pcd(self, output_path):
        """Save transformed points with intensity"""
        try:
            header = [
                "# .PCD v0.7 - Point Cloud Data file format",
                "VERSION 0.7",
                "FIELDS x y z intensity",
                "SIZE 4 4 4 4",
                "TYPE F F F F",
                "COUNT 1 1 1 1",
                f"WIDTH {len(self.points)}",
                "HEIGHT 1",
                "VIEWPOINT 0 0 0 1 0 0 0",
                f"POINTS {len(self.points)}",
                "DATA ascii"
            ]
            
            with open(output_path, 'w') as f:
                f.write('\n'.join(header) + '\n')
                for point, intensity in zip(self.points, self.intensities):
                    f.write(f"{point[0]} {point[1]} {point[2]} {intensity}\n")
            


            return True
            
        except Exception as e:
            print(f"Failed to save PCD: {e}")
            return False

if __name__ == "__main__":

    # Example usage
    #如果不存在data3文件夹，创建一个
    if not os.path.exists("data3"):
        os.makedirs("data3")
    
    transformer = CloudTransformer("data/source.pcd", "transform.npy")
    
    if transformer.read_pcd() and transformer.load_transform():
        if transformer.apply_transform():
            transformer.save_pcd("data3/transformed_source.pcd")
            print("Transformation applied and saved to 'transformed_source.pcd'")


    input_other = os.path.join('data', "target.pcd")
    output_other = os.path.join('data3', "original_target.pcd")
    
    # 直接复制文件
    shutil.copy2(input_other, output_other)
    print(f"已复制点云文件: {output_other}")