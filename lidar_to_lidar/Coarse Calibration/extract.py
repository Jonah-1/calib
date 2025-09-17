import open3d as o3d
import numpy as np
from sklearn.cluster import DBSCAN
import sys
import gc
import os
from scipy.spatial.distance import pdist

def calculate_angle_between_vectors(v1, v2):
        """
        计算两个向量之间的余弦角，并以角度表示。

        参数:
        v1 -- 第一个向量
        v2 -- 第二个向量

        返回:
        角度值（以度为单位）
        """
        # 计算点积
        dot_product = np.dot(v1, v2)
        
        # 计算范数
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)
        
        # 计算余弦值
        cosine_angle = dot_product / (norm_v1 * norm_v2)
        
        # 计算弧度
        angle_radians = np.arccos(np.clip(cosine_angle, -1.0, 1.0))
        
        # 将弧度转换为角度
        angle_degrees = np.degrees(angle_radians)
        
        return angle_degrees
    
def calculate_max_distance(points):
        """
        计算点云中最远点对之间的距离。

        参数:
        points -- 点云的坐标，NumPy 数组格式，形状为 (n, 3)

        返回:
        最远点对之间的距离
        """
        # 计算所有点对之间的距离
        distances = pdist(points)
        
        # 找到最大距离
        max_distance = np.max(distances)
        
        return max_distance



class Extractor():
    def __init__(self, visualize=True):
        """初始化检测器"""
        self.visualize = visualize
        self.reset()
        

    def reset(self):
        """重置检测器状态"""
        self.pcd = None
        self.plane1_eq = None
        self.plane2_eq = None 
        self.plane3_eq = None  # Add third plane equation
        self.plane_1 = None
        self.plane_2 = None
        self.plane_3 = None   # Add third plane
        self.direction = None
        self.point_on_line = None
        self.point_on_plane1 = None
        self.point_on_plane2 = None
        self.point_on_plane3 = None  # Add third plane point
        self.normal1 = None
        self.normal2 = None
        self.normal3 = None   # Add third normal
        self.head = None
        self.top = None
        self.rect_lineset = None
        self.remaining_cloud = None
        self.points_for_head = None
            


    def check_and_reverse_normal(self, plane_eq, point_on_plane_eq, point_on_otherplane, normal):
        """检查并在必要时反转法向量"""
        try:
            if np.dot(point_on_otherplane - point_on_plane_eq, normal) < 0:
                plane_eq = -plane_eq
            return plane_eq
        except Exception as e:
            print(f"法向量检查失败: {str(e)}")
            print("Variable shape:", point_on_otherplane.shape if hasattr(point_on_otherplane, 'shape') else "not a numpy array")
            raise  # 重新抛出异常以便程序停止

    
    def remove_rectangle_area(self, cloud, plane_model, distance):
        """去除法相方向一定距离的矩形区域"""
        a, b, c, d = plane_model
        points = np.asarray(cloud.points)
        distances = np.abs(a * points[:, 0] + b * points[:, 1] + c * points[:, 2] + d) / np.sqrt(a**2 + b**2 + c**2)
        mask = distances > distance
        return cloud.select_by_index(np.where(mask)[0])

    def process_point_cloud(self, pcd):
        """处理点云文件"""
        try:
            # 加载点云数据
            self.pcd = pcd
            if not self.pcd or len(np.asarray(self.pcd.points)) < 10:
                raise ValueError("点云数据无效或点数过少")
            print(f"已加载点云数据: {len(np.asarray(self.pcd.points))} 个点")

            self.remaining_cloud = self.pcd
            remaining_cloud = self.remaining_cloud

            plane_model_1, inliers_1 =  self.remaining_cloud.segment_plane(distance_threshold=0.05,
                                                        ransac_n=3,
                                                        num_iterations=1000)
            points_on_plane = self.remaining_cloud.select_by_index(inliers_1, invert=False)
            remaining_cloud = self.remaining_cloud
            print(f"Points on plane: {len(points_on_plane.points)}")
            if not points_on_plane.has_points():
                raise ValueError("选中的平面没有有效的点")      
            self.remaining_cloud = self.remove_rectangle_area(self.remaining_cloud, plane_model_1, 0.5)
            plane_normal = np.array(plane_model_1[:3])

            self.plane_1 = remaining_cloud.select_by_index(inliers_1)
            self.normal1 = np.array(plane_model_1[:3])
            self.plane1_eq = plane_model_1

            while True: #获得第二个平面
                plane_model_2, inliers_2 = self.remaining_cloud.segment_plane(distance_threshold=0.05,
                                                                        ransac_n=3,
                                                                        num_iterations=1000)
                points_on_plane = self.remaining_cloud.select_by_index(inliers_2, invert=False)
                remaining_cloud = self.remaining_cloud
                self.remaining_cloud = self.remove_rectangle_area(self.remaining_cloud, plane_model_2, 0.5)
                plane_normal = np.array(plane_model_2[:3])

                #检查是否是第一个平面的次多面
                if calculate_angle_between_vectors(plane_normal, self.normal1)<15 or calculate_angle_between_vectors(plane_normal, self.normal1)>160:  
                    continue
                break

            self.plane_2 = remaining_cloud.select_by_index(inliers_2)
            self.normal2 = np.array(plane_model_2[:3])
            self.plane2_eq = plane_model_2  

            #获得第三个平面
            if self.plane_3 is None:
            
                plane_model_3, inliers_3 = self.remaining_cloud.segment_plane(distance_threshold=0.05,
                                                                        ransac_n=3,
                                                                        num_iterations=1000)
                points_on_plane = self.remaining_cloud.select_by_index(inliers_3, invert=False)
                self.remaining_cloud = self.remove_rectangle_area(self.remaining_cloud, plane_model_3, 0.5)
                plane_normal = np.array(plane_model_3[:3])
                self.plane_3 = points_on_plane
                self.normal3 = np.array(plane_model_3[:3])
                self.plane3_eq = plane_model_3

            if self.plane_3 is not None:
                
                # 计算平面上的点
                self.point_on_plane1 = np.mean(np.asarray(self.plane_1.points), axis=0)
                self.point_on_plane2 = np.mean(np.asarray(self.plane_2.points), axis=0)
                self.point_on_plane3 = np.mean(np.asarray(self.plane_3.points), axis=0)

                # 检查和反转法向量
                self.plane1_eq = self.check_and_reverse_normal(self.plane1_eq, self.point_on_plane1, 
                                                            self.point_on_plane2, self.normal1)
                self.plane2_eq = self.check_and_reverse_normal(self.plane2_eq, self.point_on_plane2, 
                                                            self.point_on_plane1, self.normal2)
                self.plane3_eq = self.check_and_reverse_normal(self.plane3_eq, self.point_on_plane3,
                                                                self.point_on_plane1, self.normal3)

                self.normal1 = np.array(self.plane1_eq[:3])
                self.normal2 = np.array(self.plane2_eq[:3])
                self.normal3 = np.array(self.plane3_eq[:3])

                print(f"self.normal1: {self.normal1}")
                print(f"self.normal2: {self.normal2}")
                print(f"self.normal3: {self.normal3}")

                if self.plane1_eq is None or self.plane2_eq is None:
                    raise ValueError("法向量计算失败")

                # 计算交线方向和点
                self.direction = np.cross(self.normal1, self.normal2)
                self.direction = self.direction / np.linalg.norm(self.direction)
                
                # 计算交线上的点
                A = np.vstack([self.normal1, self.normal2])
                b = -np.array([self.plane1_eq[3], self.plane2_eq[3]])
                self.point_on_line = np.linalg.lstsq(A, b, rcond=None)[0]


                if self.visualize:
                    self.visualize_results()

                return True
            
            else:
                
                # 计算平面上的点
                self.point_on_plane1 = np.mean(np.asarray(self.plane_1.points), axis=0)
                self.point_on_plane2 = np.mean(np.asarray(self.plane_2.points), axis=0)

                # 检查和反转法向量
                self.plane1_eq = self.check_and_reverse_normal(self.plane1_eq, self.point_on_plane1, 
                                                            self.point_on_plane2, self.normal1)
                self.plane2_eq = self.check_and_reverse_normal(self.plane1_eq, self.point_on_plane2, 
                                                            self.point_on_plane1, self.normal2)

                self.normal1 = np.array(self.plane1_eq[:3])
                self.normal2 = np.array(self.plane2_eq[:3])

                if self.plane1_eq is None or self.plane2_eq is None:
                    raise ValueError("法向量计算失败")

                # 计算交线方向和点
                self.direction = np.cross(self.normal1, self.normal2)
                self.direction = self.direction / np.linalg.norm(self.direction)
                
                # 计算交线上的点
                A = np.vstack([self.normal1, self.normal2])
                b = -np.array([self.plane1_eq[3], self.plane2_eq[3]])
                self.point_on_line = np.linalg.lstsq(A, b, rcond=None)[0]

                # 查找头顶点
                # self.find_head_top_between_planes()

                if self.visualize:
                    self.visualize_results()

                return True

 

        except Exception as e:
            print(f"处理点云失败: {str(e)}")
            print("Variable shape:", np.asarray(self.pcd.points).shape if hasattr(self.pcd, 'points') else "not a numpy array")
            return False
        
    def visualize_results(self):
        """可视化结果"""
        try:
            # # 创建交线
            # line_length = 20
            # line_points = [self.point_on_line - line_length*0.05 * self.direction,
            #             self.point_on_line + line_length * self.direction]
            # line_set = o3d.geometry.LineSet(
            #     points=o3d.utility.Vector3dVector(line_points),
            #     lines=o3d.utility.Vector2iVector([[0, 1]])
            # )
            # line_set.paint_uniform_color([1, 0, 0])

            # 创建法向量显示
            normal_length = 5
            
            # 法线1设为红色
            normal1_set = o3d.geometry.LineSet(
                points=o3d.utility.Vector3dVector([self.point_on_plane1,
                                                self.point_on_plane1 + normal_length * self.normal1]),
                lines=o3d.utility.Vector2iVector([[0, 1]])
            )
            normal1_set.paint_uniform_color([1, 0, 0])  # 红色
            
            # 法线2设为绿色
            normal2_set = o3d.geometry.LineSet(
                points=o3d.utility.Vector3dVector([self.point_on_plane2,
                                                self.point_on_plane2 + normal_length * self.normal2]),
                lines=o3d.utility.Vector2iVector([[0, 1]])
            )
            normal2_set.paint_uniform_color([0, 1, 0])  # 绿色
            
            # 法线3设为蓝色
            normal3_set = o3d.geometry.LineSet(
                points=o3d.utility.Vector3dVector([self.point_on_plane3,
                                                self.point_on_plane3 + normal_length * self.normal3]),
                lines=o3d.utility.Vector2iVector([[0, 1]])
            )
            normal3_set.paint_uniform_color([0, 0, 1])  # 蓝色

            # 添加彩色平面点
            print(f"平面1点数: {len(np.asarray(self.plane_1.points)) if self.plane_1 and hasattr(self.plane_1, 'points') else 0}")
            print(f"平面2点数: {len(np.asarray(self.plane_2.points)) if self.plane_2 and hasattr(self.plane_2, 'points') else 0}")
            print(f"平面3点数: {len(np.asarray(self.plane_3.points)) if self.plane_3 and hasattr(self.plane_3, 'points') else 0}")
            
            # 清空原有几何体列表，重新添加
            geometries = []
            
            # if self.head is not None:
            #     print(f"可视化中使用的头顶点坐标: {self.head}")
            #     head_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
            #     head_sphere.translate(self.head)
            #     # 获取并打印球体中心点坐标
            #     sphere_center = np.asarray(head_sphere.get_center())
            #     print(f"球体中心点坐标: {sphere_center}")
            #     head_sphere.paint_uniform_color([0.5, 0, 0.5])
            #     geometries.append(head_sphere)

            if self.top is not None:
                print(f"可视化中使用的质心坐标: {self.top}")
                top_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
                top_sphere.translate(self.top)
                # 获取并打印球体中心点坐标
                sphere_center = np.asarray(top_sphere.get_center())
                print(f"质心球体中心点坐标: {sphere_center}")
                top_sphere.paint_uniform_color([0.5, 0, 0.5])
                geometries.append(top_sphere)
            # 计算除去三个平面后的剩余点云
            if self.pcd is not None:
                # 创建所有点的集合
                all_points = set(tuple(p) for p in np.asarray(self.pcd.points).tolist())
                plane1_points = set()
                plane2_points = set()
                plane3_points = set()
                
                # 收集所有平面上的点
                if self.plane_1 is not None and hasattr(self.plane_1, 'points'):
                    plane1_points = set(tuple(p) for p in np.asarray(self.plane_1.points).tolist())
                if self.plane_2 is not None and hasattr(self.plane_2, 'points'):
                    plane2_points = set(tuple(p) for p in np.asarray(self.plane_2.points).tolist())
                if self.plane_3 is not None and hasattr(self.plane_3, 'points'):
                    plane3_points = set(tuple(p) for p in np.asarray(self.plane_3.points).tolist())
                
                # 计算剩余点
                remaining_points = all_points - plane1_points - plane2_points - plane3_points
                
                if remaining_points:
                    # 转换回numpy数组
                    remaining_array = np.array(list(remaining_points))
                    
                    # 创建剩余点云
                    remaining_cloud = o3d.geometry.PointCloud()
                    remaining_cloud.points = o3d.utility.Vector3dVector(remaining_array)
                    remaining_cloud.paint_uniform_color([0.7, 0.7, 0.7])  # 灰色
                    geometries.append(remaining_cloud)
                    print(f"已添加剩余点云（灰色），点数: {len(remaining_array)}")
            
            # 创建新的点云对象用于平面1
            if self.plane_1 is not None and hasattr(self.plane_1, 'points'):
                print(f"平面1类型: {type(self.plane_1)}")
                plane_1_colored = o3d.geometry.PointCloud()
                plane_1_colored.points = self.plane_1.points
                plane_1_colored.paint_uniform_color([1, 0, 0])  # 红色
                geometries.append(plane_1_colored)
                print(f"已添加平面1的红色点云，点数: {len(np.asarray(plane_1_colored.points))}")
            
            # 创建新的点云对象用于平面2
            if self.plane_2 is not None and hasattr(self.plane_2, 'points'):
                print(f"平面2类型: {type(self.plane_2)}")
                plane_2_colored = o3d.geometry.PointCloud()
                plane_2_colored.points = self.plane_2.points
                plane_2_colored.paint_uniform_color([0, 1, 0])  # 绿色
                geometries.append(plane_2_colored)
                print(f"已添加平面2的绿色点云，点数: {len(np.asarray(plane_2_colored.points))}")
            
            # 创建新的点云对象用于平面3
            if self.plane_3 is not None and hasattr(self.plane_3, 'points'):
                print(f"平面3类型: {type(self.plane_3)}")
                plane_3_colored = o3d.geometry.PointCloud()
                plane_3_colored.points = self.plane_3.points
                plane_3_colored.paint_uniform_color([0, 0, 1])  # 蓝色
                geometries.append(plane_3_colored)
                print(f"已添加平面3的蓝色点云，点数: {len(np.asarray(plane_3_colored.points))}")
            
            # 添加其他几何体
            # geometries.append(line_set)
            geometries.append(normal1_set)
            geometries.append(normal2_set)
            geometries.append(normal3_set)

            # if self.rect_lineset is not None:
            #     geometries.append(self.rect_lineset)

            # 添加平面上的点的可视化
            if self.point_on_plane1 is not None:
                point1_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.08)
                point1_sphere.translate(self.point_on_plane1)
                point1_sphere.paint_uniform_color([1, 0.5, 0.5])  # 浅红色
                geometries.append(point1_sphere)
                
            if self.point_on_plane2 is not None:
                point2_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.08)
                point2_sphere.translate(self.point_on_plane2)
                point2_sphere.paint_uniform_color([0.5, 1, 0.5])  # 浅绿色
                geometries.append(point2_sphere)
                
            if self.point_on_plane3 is not None:
                point3_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.08)
                point3_sphere.translate(self.point_on_plane3)
                point3_sphere.paint_uniform_color([0.5, 0.5, 1])  # 浅蓝色
                geometries.append(point3_sphere)

            # 打印结果
            print(f"\n平面1方程: {self.plane1_eq[0]:.4f}x + {self.plane1_eq[1]:.4f}y + "
                f"{self.plane1_eq[2]:.4f}z + {self.plane1_eq[3]:.4f} = 0")
            print(f"平面2方程: {self.plane2_eq[0]:.4f}x + {self.plane2_eq[1]:.4f}y + "
                f"{self.plane2_eq[2]:.4f}z + {self.plane2_eq[3]:.4f} = 0")
            print(f"平面3方程: {self.plane3_eq[0]:.4f}x + {self.plane3_eq[1]:.4f}y + "
                    f"{self.plane3_eq[2]:.4f}z + {self.plane3_eq[3]:.4f} = 0")
            print(f"交线方向向量: [{self.direction[0]:.4f}, {self.direction[1]:.4f}, {self.direction[2]:.4f}]")
            print(f"交线上的点: [{self.point_on_line[0]:.4f}, {self.point_on_line[1]:.4f}, "
                f"{self.point_on_line[2]:.4f}]")

            # 使用自定义可视化
            vis = o3d.visualization.Visualizer()
            vis.create_window()
            
            # 添加所有几何体
            for geom in geometries:
                vis.add_geometry(geom)
            
            # 设置渲染选项
            opt = vis.get_render_option()
            opt.point_size = 5.0  # 进一步增大点的大小
            opt.background_color = np.array([1, 1, 1])  # 黑色背景
            
            # 更新视图并运行可视化
            vis.update_renderer()
            vis.run()
            vis.destroy_window()

        except Exception as e:
            print(f"可视化失败: {str(e)}")

    def get_results(self):
        return {
            'plane1_equation': self.plane1_eq,
            'plane2_equation': self.plane2_eq,
            'plane3_equation': self.plane3_eq,
            'intersection_direction': self.direction,
            'intersection_point': self.point_on_line,
            'head_top_coordinate': self.top,
            'points_on_plane1':self.plane_1.points,
            'points_on_plane2':self.plane_2.points,
            'points_on_plane3':self.plane_3.points
        }
    

if __name__ == "__main__":
    if len(sys.argv) > 1:
        file = sys.argv[1]
    else:
        file = 'data1/target.pcd'

    pcd = o3d.io.read_point_cloud(file)
        
    detector = Extractor(visualize=True)
    if detector.process_point_cloud(pcd):
        results = detector.get_results()
        if results['head_top_coordinate'] is not None:
            print(f"找到头顶点坐标: {results['head_top_coordinate']}")