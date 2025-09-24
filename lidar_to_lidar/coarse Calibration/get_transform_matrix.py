from locale import normalize as locale_normalize
from extract import Extractor
import os
import numpy as np
import sys
import open3d as o3d
import copy

def copy_point_cloud(pcd):
    """正确复制点云，避免deepcopy可能导致的问题"""
    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points))
    if pcd.has_colors():
        new_pcd.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors))
    if pcd.has_normals():
        new_pcd.normals = o3d.utility.Vector3dVector(np.asarray(pcd.normals))
    return new_pcd

def rotation_matrix(axis, theta):
    """
    计算绕任意轴的旋转矩阵
    :param axis: 旋转轴 (3D 向量)
    :param theta: 旋转角度（弧度）
    :return: 旋转矩阵 (3x3)
    """
    axis = normalize(axis)
    a = np.cos(theta / 2.0)
    b, c, d = -axis * np.sin(theta / 2.0)
    return np.array([
        [a*a + b*b - c*c - d*d, 2*(b*c - a*d), 2*(b*d + a*c)],
        [2*(b*c + a*d), a*a + c*c - b*b - d*d, 2*(c*d - a*b)],
        [2*(b*d - a*c), 2*(c*d + a*b), a*a + d*d - b*b - c*c]
    ])
    
def normalize(v):
    """单位化向量"""
    norm = np.linalg.norm(v)
    if norm < 1e-6:
        return v  # 避免除零错误
    return v / norm

def calculate_angle_between_vectors(v1, v2):
    """
    计算两个向量之间的角度（弧度）
    
    Args:
        v1, v2: np.ndarray - 输入向量
            
    Returns:
        float - 两向量之间的夹角（弧度）
    """
    v1_norm = np.linalg.norm(v1)
    v2_norm = np.linalg.norm(v2)
    
    # 检查向量是否为零向量
    if v1_norm < 1e-6 or v2_norm < 1e-6:
        return 0.0
    
    # 归一化向量
    v1_normalized = v1 / v1_norm
    v2_normalized = v2 / v2_norm
    
    # 计算点积并使用arccos计算角度
    dot_product = np.clip(np.dot(v1_normalized, v2_normalized), -1.0, 1.0)
    angle = np.arccos(dot_product)
    
    return angle

class PointCloudTransformer():
    def __init__(self,folder_path="data1"):
        self.folder_path = folder_path
        self.folder = []
        self.transform= np.eye(4)
        self.source_features = None
        self.target_features = None
        self.pcd_target = None
        self.pcd_source = None


    def process_pcd_files(self):
        try:
            
            # 遍历文件夹下的所有pcd文件
            for root, _, files in os.walk(self.folder_path):
                for file in files:
                    if file.endswith('.pcd'):
                        full_path = os.path.join(root, file)
                        print(f"Processing {file}...")
                        self.folder.append(full_path)

                        if file == 'target.pcd':
                            self.pcd_target = o3d.io.read_point_cloud(full_path)
                        elif file == 'source.pcd':
                            self.pcd_source = o3d.io.read_point_cloud(full_path)
            
            return True
            

        except Exception as e:
            print(f"Failed to process pcd files: {str(e)}")
            return False
        
    def align_planes(self,n1, m1):
        """
        计算旋转矩阵，使点云1的法向量(n1)对齐到点云2的法向量(m1)
        """
        # 单位化法向量
        n1 = n1 / np.linalg.norm(n1)
        m1 = m1 / np.linalg.norm(m1)
        
        # 计算点积
        dot_product = np.dot(n1, m1)
        
        # 如果向量几乎相反
        if abs(dot_product + 1) < 1e-6:
            # 构造180度旋转矩阵
            # 可以选择任意与n1垂直的轴进行旋转
            # 这里我们可以用[1,0,0]或[0,1,0]与n1叉乘得到一个垂直轴
            if abs(n1[2]) < 0.9:  # 如果n1不太垂直于xy平面
                rot_axis = np.cross(n1, [0,0,1])
            else:
                rot_axis = np.cross(n1, [0,1,0])
            rot_axis = rot_axis / np.linalg.norm(rot_axis)
            return rotation_matrix(rot_axis, np.pi)
        
        # 计算旋转
        axis1 = np.cross(n1, m1)
        if np.linalg.norm(axis1) > 1e-6:
            theta1 = np.arccos(np.clip(dot_product, -1.0, 1.0))
            axis1 = axis1 / np.linalg.norm(axis1)
            return rotation_matrix(axis1, theta1)
        else:
            return np.eye(3)
    
    def get_optimal_translation_vector(self,points, plane_eq, direction):
        """
        Calculate optimal translation vector for plane to minimize distances to points
        
        Args:
            points: np.ndarray (N,3) - Point cloud data
            plane_eq: tuple (a,b,c,d) - Plane equation ax+by+cz+d=0 
            direction: np.ndarray (3,) - Direction vector
            
        Returns:
            np.ndarray (3,) - Optimal translation vector
        """
        a, b, c, d = plane_eq
        normal = np.array([a, b, c])
        
        # Normalize direction vector
        direction = direction / np.linalg.norm(direction)
        
        # Project all points onto direction vector
        proj = np.dot(points, direction)
        
        # Calculate optimal translation distance
        dot_dir_normal = np.dot(direction, normal)
        if abs(dot_dir_normal) < 1e-6:
            return np.zeros(3)  # Direction parallel to plane
            
        # For each point, calculate signed distance to plane
        signed_dists = (np.dot(points, normal) + d) / np.linalg.norm(normal)
        
        # Calculate optimal translation distance
        t = np.mean(signed_dists)
        
        # Return translation vector
        return direction * t
    
    def build_orthogonal_vectors(self,v1, v2):
        """
        Build orthogonal vectors using Gram-Schtargett process
        
        Args:
            v1, v2, v3: np.ndarray - Input vectors
            
        Returns:
            tuple of three np.ndarray - Orthonormal vectors
        """
        # Convert to numpy arrays
        v1 = np.asarray(v1)
        v2 = np.asarray(v2)
        
        # Normalize first vector
        u1 = v1 / np.linalg.norm(v1)
        
        # Get second orthogonal vector
        u2 = v2 - np.dot(v2, u1) * u1
        u2 = u2 / np.linalg.norm(u2)
        
        
        return u1, u2

    def GetTF_Matrix(self):
        """Calculate transformation matrices from target.pcd to source.pcd and right.pcd"""
        try:
            # Process each point cloud file
            for file in self.folder:
                print(f"Processing {file}...")

                extractor = Extractor(visualize=False)

                pcd = o3d.io.read_point_cloud(file)
                
                if extractor.process_point_cloud(pcd):
                        features = extractor.get_results()
                        if not isinstance(features, dict):
                            print(f"处理 {file} 失败: 特征必须是字典")
                            return None  # 处理失败时返回 None

                        if os.path.basename(file) == 'target.pcd':
                            self.source_features = features

                        elif os.path.basename(file) == 'source.pcd':
                            self.target_features = features
                   
                else:
                    print(f"处理 {file} 失败")
                    return None  # 处理失败时返回 None
                
        
            # Calculate transformation matrices
            self.transform= self.calculate_transformation_matrix(
            self.source_features, 
            self.target_features)


            return self.transform

            
        except Exception as e:
            print(f"Failed to get transformation matrix: {str(e)}")
            import traceback
            traceback.print_exc()
            return None
        


    def calculate_transformation_matrix(self,source_features, target_features):
        """
        计算从源点云到目标点云的变换矩阵
        
        Args:
            source_features: 源点云特征 (plane1_eq, plane2_eq, direction, point, head)
            target_features: 目标点云特征 (plane1_eq, plane2_eq, direction, point, head)
            
        Returns:
            transform: 4x4变换矩阵
        """
        try:
            # 输入验证
            if not isinstance(source_features, dict) or not isinstance(target_features, dict):
                raise ValueError("Features must be dictionaries")
                
            # 解包特征
            s_p1 = source_features['plane1_equation']
            s_p2 = source_features['plane2_equation']
            s_p3 = source_features['plane3_equation']
            s_head = source_features['head_top_coordinate']
            print(f"找到target头顶点坐标: {s_head}")
            spoint1 = source_features['points_on_plane1']
            spoint2 = source_features['points_on_plane2']
            spoint3 = source_features['points_on_plane3']
            transform0=np.eye(4)
            transform1=np.eye(4)
            transform2=np.eye(4)
            transform3=np.eye(4)        
            transform4=np.eye(4)
            transform5=np.eye(4)
            transform6=np.eye(4)
            transform7=np.eye(4)
            
            # 获取平面法向量并归一化
            s_n1 = s_p1[:3]
            s_n2 = s_p2[:3]
            s_n3 = s_p3[:3]
            

            pcd1=copy_point_cloud(self.pcd_source)

            # 首先把点云翻转
            Extractor1 = Extractor(visualize=False)
            if not Extractor1.process_point_cloud(pcd1):
                raise ValueError("Failed to process transformed source point cloud")
            R_list = [
                        np.array([[1, 0, 0],  # 不旋转
                                [0, 1, 0],
                                [0, 0, 1]]),
                        
                        np.array([[1, 0, 0],  # 绕x轴顺时针旋转90度
                                [0, 0, 1],
                                [0, -1, 0]]),
                        
                        np.array([[1, 0, 0],  # 绕x轴顺时针旋转180度
                                [0, -1, 0],
                                [0, 0, -1]]),
                        
                        np.array([[1, 0, 0],  # 绕x轴逆时针旋转90度
                                [0, 0, -1],
                                [0, 1, 0]])
                    ]
            R0=R_list[3]
            transform0[:3,:3]=R0
            pcd1.transform(transform0)

            # 第一次旋转 - 对齐第一个平面法向量
            Extractor1 = Extractor(visualize=False)
            if not Extractor1.process_point_cloud(pcd1):
                raise ValueError("Failed to process transformed source point cloud")
            features1 = Extractor1.get_results()
            t_p1 = features1['plane1_equation']
            t_n1 = t_p1[:3]
            R1 = self.align_planes(s_n1, t_n1)
            transform1[:3,:3] = R1
            pcd1.transform(transform1)


            #利用平面2计算旋转矩阵
            Extractor1 = Extractor(visualize=False)  # 改为False避免多次显示
            if not Extractor1.process_point_cloud(pcd1):
                raise ValueError("Failed to process transformed source point cloud")
            features1 = Extractor1.get_results()
            t_p2 = features1['plane2_equation']
            t_n2 = t_p2[:3]
            R2 = self.align_planes(s_n2, t_n2)
            transform2[:3,:3] = R2
            pcd1.transform(transform2)

           #利用平面3计算旋转矩阵
            Extractor1 = Extractor(visualize=False)  # 改为False避免多次显示
            if not Extractor1.process_point_cloud(pcd1):
                raise ValueError("Failed to process transformed source point cloud")
            features1 = Extractor1.get_results()
            t_p3 = features1['plane3_equation']
            t_n3 = t_p3[:3]
            R3 = self.align_planes(s_n3, t_n3)
            transform3[:3,:3] = R3
            pcd1.transform(transform3)



            # 利用平面1和2计算正交向量sm1,sm2,sm3
            sm1, sm2 = self.build_orthogonal_vectors(s_n1, s_n2)
            sm3=np.cross(sm1,sm2)
            sm3=sm3/np.linalg.norm(sm3)


            #利用平面2计算平移向量
            Extractor1 = Extractor(visualize=False)  # 改为False避免多次显示
            if not Extractor1.process_point_cloud(pcd1):
                raise ValueError("Failed to process transformed source point cloud")
            features1 = Extractor1.get_results()
            plane2 = features1['plane2_equation']
            transform4 = np.eye(4)
            transform4[:3,3] =  self.get_optimal_translation_vector(spoint2, plane2, sm2)
            pcd1.transform(transform4)

            # 利用平面1计算平移向量  
            Extractor1 = Extractor(visualize=False)  # 改为False避免多次显示
            if not Extractor1.process_point_cloud(pcd1):
                raise ValueError("Failed to process transformed source point cloud")
            features1 = Extractor1.get_results()
            plane1 = features1['plane1_equation']
            transform5 = np.eye(4)
            transform5[:3,3] =  self.get_optimal_translation_vector(spoint1, plane1, sm1)
            pcd1.transform(transform5)
            

            #利用平面3计算平移向量 
            Extractor1 = Extractor(visualize=False)  # 改为False避免多次显示
            if not Extractor1.process_point_cloud(pcd1):
                raise ValueError("Failed to process transformed source point cloud")
            features1 = Extractor1.get_results()
            plane3 = features1['plane3_equation']
            transform6 = np.eye(4)
            transform6[:3,3] = self.get_optimal_translation_vector(spoint3, plane3, sm3)
            pcd1.transform(transform6)
            print(f"平面3平移向量: {transform5[:3,3]}")

            #利用头顶坐标计算平移向量
            # Extractor1 = Extractor(visualize=False)  # 改为False避免多次显示
            # if not Extractor1.process_point_cloud(pcd1):
            #     raise ValueError("Failed to process transformed source point cloud")
            # features1 = Extractor1.get_results()
            # t_head = features1['head_top_coordinate']
            # print(f"找到source头顶点坐标: {t_head}")

            # transform7 = np.eye(4)
            # transform7[:3,3] = (s_head-t_head)*1.2
            # pcd1.transform(transform7)

            #不使用icp
            transform = transform7@transform6@transform5@transform4@transform3@transform2@transform1@transform0
            #    #transform5 @transform4 @transform3@transform2 @如果头顶坐标不准，就用这个
             
            
            return transform  # 返回单个矩阵而不是tuple

        except Exception as e:
            print(f"Failed to calculate transformation matrix: {str(e)}")
            raise  # 抛出异常而不是返回None

    def calculate_advanced_icp(self, source, target, init_transform=np.eye(4),):
        # """
        # 使用多级ICP进行高精度配准
        # """
        # # 1. 计算点云法向量
        # source.estimate_normals(
        #     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        # target.estimate_normals(
        #     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        # # 2. 多尺度ICP配准
        # current_transform = init_transform
        # voxel_sizes = [0.05, 0.025, 0.0125]  # 从粗到精的配准尺度
        
        # for voxel_size in voxel_sizes:
        #     # 下采样点云
        #     source_down = source.voxel_down_sample(voxel_size)
        #     target_down = target.voxel_down_sample(voxel_size)
            
        #     # 估计法向量
        #     source_down.estimate_normals(
        #         search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2, max_nn=30))
        #     target_down.estimate_normals(
        #         search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2, max_nn=30))

        #     # 点到面ICP
        #     reg_p2l = o3d.pipelines.registration.registration_icp(
        #         source_down, target_down,
        #         max_correspondence_distance=voxel_size*2,
        #         init=current_transform,
        #         estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        #         criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
        #             max_iteration=50,
        #             relative_fitness=1e-6,
        #             relative_rmse=1e-6
        #         )
        #     )
        #     current_transform = reg_p2l.transformation
        #     print(f"Scale {voxel_size}, Fitness: {reg_p2l.fitness}, RMSE: {reg_p2l.inlier_rmse}")

        # 点到点ICP
        voxel_sizes = [0.05]  # 从粗到精的配准尺度
        current_transform = init_transform
        
        for voxel_size in voxel_sizes:
            reg_p2p = o3d.pipelines.registration.registration_icp(
                source,
                target,
                max_correspondence_distance=voxel_size * 2,
                init=current_transform,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=100,
                    relative_fitness=1e-8,
                    relative_rmse=1e-8
                )
            )
            current_transform = reg_p2p.transformation
            print(f"Scale {voxel_size}, Fitness: {reg_p2p.fitness}, RMSE: {reg_p2p.inlier_rmse}")
            

        return current_transform
            
    def apply_transforms(self):
        try:
            # 检查变换矩阵和点云是否存在
            if self.transform is None:
                print("变换矩阵未计算，请先调用GetTF_Matrix()")
                return False
                
            if self.pcd_source is None:
                print("点云数据未加载")
                return False
                
            # 应用变换
            if self.pcd_source is not None and self.transform is not None:
                self.pcd_source.transform(self.transform)
                
            
            output_dir='data2'    
            # 创建输出目录
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
                
            # 保存中间点云
            if self.pcd_target is not None:
                o3d.io.write_point_cloud(
                    os.path.join(output_dir, "target.pcd"), 
                    self.pcd_target
                )
                
            # 保存变换后的左侧点云
            if self.pcd_source is not None:
                o3d.io.write_point_cloud(
                    os.path.join(output_dir, "source.pcd"),
                    self.pcd_source
                )
                
            
            return True
        
        except Exception as e:
            print(f"应用变换失败: {str(e)}")
            return False

    
    def visualize(self):
        """可视化原始和变换后的点云"""
        # 创建不同颜色的点云
        if self.pcd_target is not None:
            self.pcd_target.paint_uniform_color([1, 0, 0])  # 红色
        if self.pcd_source is not None:
            self.pcd_source.paint_uniform_color([0, 1, 0])  # 绿色


        # 显示点云
        pcds = [pc for pc in [self.pcd_target, self.pcd_source] 
                if pc is not None]
        o3d.visualization.draw_geometries(pcds)  


if __name__ == "__main__":
    def format_matrix(matrix):
        """Format matrix with comma-separated elements"""
        if matrix is None:
            return "None"
            
        formatted_lines = []
        for row in matrix:
            # Format each number and join with commas
            formatted_row = ", ".join(f"{x:10.8f}" for x in row)
            formatted_lines.append(f"    [{formatted_row}]")
        
        return "[\n" + ",\n".join(formatted_lines) + "\n]"

    # 创建PointCloudTransformer实例
    transformer=PointCloudTransformer()
    if transformer.process_pcd_files():
        transform=transformer.GetTF_Matrix()

        # Update print statements
        print(f"变换矩阵:\n{format_matrix(transform)}\n")
  

        # 应用变换并可视化
        transformer.apply_transforms()
        transformer.visualize()       
    else:
        print("Failed to process pcd files")
    

    
