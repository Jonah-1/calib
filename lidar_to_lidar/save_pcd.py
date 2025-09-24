import os
import rosbag
from tqdm import tqdm
import shutil
import subprocess
from pathlib import Path

class ExtractPointCloudData(object):

    def __init__(self, bagfile_path, pointcloud_topics, root, storage_path):
        self.bagfile_path = bagfile_path
        self.pointcloud_topics = pointcloud_topics
        self.root = root
        self.pointcloud_dirs = {
            "source": os.path.join(root, "source"),
            "target": os.path.join(root, "target"),
        }
        self.storage_path = storage_path
        
        # 创建提取点云的目录
        for dir_path in self.pointcloud_dirs.values():
            os.makedirs(dir_path, exist_ok=True)
        os.makedirs(f"{self.storage_path}/data", exist_ok=True)
        os.makedirs(f"{self.storage_path}/data1", exist_ok=True)

    def is_db3_file(self):
        """检查是否为db3文件（ROS2格式）"""
        return self.bagfile_path.endswith('.db3') or Path(self.bagfile_path).is_dir()
    
    def extract_pointcloud_topics(self):
        """根据文件格式选择不同的提取方法"""
        if self.is_db3_file():
            print(f"检测到DB3格式文件: {self.bagfile_path}")
            return self.extract_from_db3()
        else:
            print(f"检测到BAG格式文件: {self.bagfile_path}")
            return self.extract_from_bag()
    
    def extract_from_bag(self):
        """从ROS1 bag文件提取点云"""
        try:
            bag = rosbag.Bag(self.bagfile_path, "r")
            
            for dir_key, topic in self.pointcloud_topics.items():
                pointcloud_dir = self.pointcloud_dirs[dir_key]
                # 清空输出目录
                clear_output_directory(pointcloud_dir)
                
                cmd = f"rosrun pcl_ros bag_to_pcd {self.bagfile_path} {topic} {pointcloud_dir}"
                print(f"执行命令: {cmd}")
                os.system(cmd)

                print(f"提取 {topic} 到 {pointcloud_dir}")
                
                # 找到第一个pcd文件并复制到storage_path/data
                self._copy_first_pcd(dir_key, pointcloud_dir)
            
            bag.close()
            return True
        except Exception as e:
            print(f"BAG文件处理失败: {e}")
            return False
    
    def extract_from_db3(self):
        """从ROS2 db3文件提取点云"""
        print("使用ROS2方法处理DB3文件...")
        
        # 检查ROS2环境
        if not self._check_ros2_environment():
            print("ROS2环境不可用，无法处理DB3文件")
            return False
        
        for dir_key, topic in self.pointcloud_topics.items():
            pointcloud_dir = self.pointcloud_dirs[dir_key]
            clear_output_directory(pointcloud_dir)
            
            # 尝试使用ros2 run pcl_ros bag_to_pcd
            try:
                cmd = [
                    'ros2', 'run', 'pcl_ros', 'bag_to_pcd',
                    self.bagfile_path, topic, pointcloud_dir
                ]
                print(f"执行命令: {' '.join(cmd)}")
                result = subprocess.run(cmd, capture_output=True, text=True, check=True)
                print(f"成功提取 {topic} 到 {pointcloud_dir}")
                
            except subprocess.CalledProcessError as e:
                print(f"ROS2 pcl_ros命令失败: {e}")
                print("尝试转换DB3到BAG格式...")
                
                # 尝试转换db3到bag格式
                if self._convert_db3_to_bag_and_extract(dir_key, topic, pointcloud_dir):
                    print(f"转换方法成功提取 {topic}")
                else:
                    print(f"所有方法都失败了，无法提取 {topic}")
                    continue
            
            # 复制第一个pcd文件
            self._copy_first_pcd(dir_key, pointcloud_dir)
        
        return True
    
    def _check_ros2_environment(self):
        """检查ROS2环境"""
        try:
            result = subprocess.run(['ros2', '--version'], capture_output=True, text=True)
            return result.returncode == 0
        except FileNotFoundError:
            return False
    
    def _convert_db3_to_bag_and_extract(self, dir_key, topic, pointcloud_dir):
        """将db3转换为bag格式再提取"""
        import tempfile
        
        with tempfile.TemporaryDirectory() as temp_dir:
            converted_bag_path = os.path.join(temp_dir, "converted.bag")
            
            try:
                # 转换命令
                cmd = [
                    'ros2', 'bag', 'convert',
                    '--input', self.bagfile_path,
                    '--output-format', 'rosbag_v2', 
                    '--output', converted_bag_path
                ]
                
                result = subprocess.run(cmd, capture_output=True, text=True, check=True)
                print("DB3转换为BAG成功!")
                
                # 使用原有方法处理转换后的bag文件
                cmd = f"rosrun pcl_ros bag_to_pcd {converted_bag_path} {topic} {pointcloud_dir}"
                print(f"执行转换后的提取命令: {cmd}")
                os.system(cmd)
                
                return True
                
            except subprocess.CalledProcessError as e:
                print(f"转换失败: {e}")
                return False
    
    def _copy_first_pcd(self, dir_key, pointcloud_dir):
        """复制第一个pcd文件到存储目录"""
        pcd_files = [f for f in os.listdir(pointcloud_dir) if f.endswith('.pcd')]
        if pcd_files:
            pcd_files.sort()
            first_pcd = pcd_files[0]
            source_file = os.path.join(pointcloud_dir, first_pcd)
            target_file = os.path.join(self.storage_path, "data", f"{dir_key}.pcd")
            
            shutil.copy2(source_file, target_file)
            print(f"复制 {first_pcd} 到 {target_file}")
        else:
            print(f"在 {pointcloud_dir} 中未找到PCD文件") 


def clear_output_directory(directory_path):
    # 检查目录是否存在
    if os.path.exists(directory_path):
        # 遍历目录中的所有文件和文件夹
        for filename in os.listdir(directory_path):
            file_path = os.path.join(directory_path, filename)
            try:
                # 如果是文件夹，递归删除
                if os.path.isdir(file_path):
                    shutil.rmtree(file_path)
                # 如果是文件，删除文件
                else:
                    os.remove(file_path)
            except Exception as e:
                print(f'Failed to delete {file_path}. Reason: {e}')
    else:
        print(f'The directory {directory_path} does not exist.')

if __name__ == '__main__':
    # 支持BAG和DB3文件的配置
    bag_configs = {
        # ROS1 BAG文件
        "lidar-front.bag": {
            "source_topic": "/front/rslidar_points_unique",
            "storage_subdir": "front"
        },
        "lidar-back.bag": {
            "source_topic": "/back/rslidar_points_unique",
            "storage_subdir": "back"
        },
        # ROS2 DB3文件
        "lidar-front.db3": {
            "source_topic": "/front/rslidar_points_unique",
            "storage_subdir": "front"
        },
        "lidar-back.db3": {
            "source_topic": "/back/rslidar_points_unique",
            "storage_subdir": "back"
        }
    }

    target_topic = "/middle_helios/rslidar_points_unique"

    for bagfile, config in bag_configs.items():
        # 检查文件是否存在
        if not os.path.exists(bagfile):
            print(f"文件 {bagfile} 不存在，跳过...")
            continue
    
        bagfile_path = bagfile
        source_topic = config["source_topic"]
        storage_subdir = config["storage_subdir"]
        
        pointcloud_topics = {
            "source": source_topic,
            "target": target_topic
        }
        
        storage_path = f"./storaged-data/{storage_subdir}"
        
        print(f"\n=== 处理文件: {bagfile_path} ===")
        extract_bag = ExtractPointCloudData(bagfile_path, pointcloud_topics, './', storage_path)
        success = extract_bag.extract_pointcloud_topics()
        
        if success:
            print(f"✅ 成功处理 {bagfile_path}")
        else:
            print(f"❌ 处理 {bagfile_path} 失败")
            
    # 清理临时目录
    shutil.rmtree("./source", ignore_errors=True)
    shutil.rmtree("./target", ignore_errors=True)
    print("\n清理临时目录完成")

