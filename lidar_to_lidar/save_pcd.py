import os
import rosbag
from tqdm import tqdm
import shutil

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

    def extract_pointcloud_topics(self):
        bag = rosbag.Bag(self.bagfile_path, "r")
        
        for dir_key, topic  in self.pointcloud_topics.items():
            pointcloud_dir = self.pointcloud_dirs[dir_key]
            # 清空输出目录
            clear_output_directory(pointcloud_dir)
            
            cmd = f"rosrun pcl_ros bag_to_pcd {self.bagfile_path} {topic} {pointcloud_dir}"
            os.system(cmd)

            print(f"Extracted {topic} to {pointcloud_dir}")
            
            # 找到第一个pcd文件并复制到storage_path/data
            pcd_files = [f for f in os.listdir(pointcloud_dir) if f.endswith('.pcd')]
            if pcd_files:
                # 按文件名排序，取第一个
                pcd_files.sort()
                first_pcd = pcd_files[0]
                source_file = os.path.join(pointcloud_dir, first_pcd)
                target_file = os.path.join(self.storage_path, "data", f"{dir_key}.pcd")
                
                # 复制文件
                shutil.copy2(source_file, target_file)
                print(f"Copied {first_pcd} to {target_file}") 


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
    bag_configs = {
        "lidar-front.bag": 
            {
                "source_topic": "/front/rslidar_points_unique",
                "storage_subdir": "front"
            },
        "lidar-back.bag": {
            "source_topic": "/back/rslidar_points_unique",
            "storage_subdir": "back"
        }
    }

    target_topic = "/middle_helios/rslidar_points_unique"

    for bagfile, config in bag_configs.items():
    
        bagfile_path = bagfile
        source_topic = config["source_topic"]
        storage_subdir = config["storage_subdir"]
        
        pointcloud_topics = {
            "source": source_topic,
            "target": target_topic
        }
        
        storage_path = f"./storaged-data/{storage_subdir}"
        
        extract_bag = ExtractPointCloudData(bagfile_path, pointcloud_topics, './', storage_path)
        extract_bag.extract_pointcloud_topics()
            
    shutil.rmtree("./source")
    shutil.rmtree("./target")

