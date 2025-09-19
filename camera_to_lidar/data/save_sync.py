import os
import rosbag
import cv2
from cv_bridge import CvBridge
from tqdm import tqdm
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import struct
import shutil

class BagExtractor:
<<<<<<< HEAD
    def __init__(self, bag_path, lidar_topics, image_topics, output_dirs):
        self.bag_path = bag_path
        self.lidar_topics = lidar_topics
        self.image_topics = image_topics
        self.output_dirs = output_dirs

        self.pointcloud_dirs = {
            "front": os.path.join(output_dirs[0], "pointclouds"),
            "back": os.path.join(output_dirs[1], "pointclouds"),
            "left": os.path.join(output_dirs[2], "pointclouds"),
            "right": os.path.join(output_dirs[3], "pointclouds"),
            "front_middle": os.path.join(output_dirs[4], "pointclouds")
        }
        self.images_dirs = {"front": os.path.join(output_dirs[0], "images"),
                            "back": os.path.join(output_dirs[1], "images"),
                            "left": os.path.join(output_dirs[2], "images"),
                            "right": os.path.join(output_dirs[3], "images"),
                            "front_middle": os.path.join(output_dirs[4], "images")}
        self.names=["front", "back", "left", "right", "front_middle"]
        # 💡 运行前清空旧目录
        for _, pointcloud_dir in self.pointcloud_dirs.items():
            if os.path.exists(pointcloud_dir):
                shutil.rmtree(pointcloud_dir)
            os.makedirs(pointcloud_dir, exist_ok=True)

        for _, image_dir in self.images_dirs.items():
            if os.path.exists(image_dir):
                shutil.rmtree(image_dir)
            os.makedirs(image_dir, exist_ok=True)

=======
    def __init__(self, bag_path, lidar_topic, image_topic, output_dir):
        self.bag_path = bag_path
        self.lidar_topic = lidar_topic
        self.image_topic = image_topic
        self.lidar_dir = os.path.join(output_dir, "pointclouds")
        if "pinhole" in output_dir:
            camera_type = "pinhole"
        else:
            camera_type = "fisheye"
        self.image_dir = os.path.join(output_dir, f"{camera_type}-images")
        
        # 如果目录已存在，先删除
        if os.path.exists(self.lidar_dir):
            shutil.rmtree(self.lidar_dir)
        if os.path.exists(self.image_dir):
            shutil.rmtree(self.image_dir)

        # 再重新创建
        os.makedirs(self.lidar_dir, exist_ok=True)
        os.makedirs(self.image_dir, exist_ok=True)
>>>>>>> 38a7e901473e2eb7cd9f5f07a3e750ff014220b6

    def extract_sync_data(self, time_tolerance=0.03):
        print("Opening bag file...")
        bag = rosbag.Bag(self.bag_path)

<<<<<<< HEAD
        # 使用临时变量存储不同topic的消息
        lidar_front_msgs = []
        lidar_back_msgs = []
        image_front_msgs = []
        image_back_msgs = []
        image_left_msgs = []
        image_right_msgs = []
        image_front_middle_msgs = []
        
        print("Reading messages...")
        for topic, msg, t in tqdm(bag.read_messages(topics=list(self.lidar_topics.values()) + list(self.image_topics.values()))):
            if topic == self.lidar_topics["lidar_front_topic"]:
                lidar_front_msgs.append((t.to_sec(), msg))
            elif topic == self.lidar_topics["lidar_back_topic"]:
                lidar_back_msgs.append((t.to_sec(), msg))
            elif topic == self.image_topics["image_front_topic"]:
                image_front_msgs.append((t.to_sec(), msg))
            elif topic == self.image_topics["image_back_topic"]:
                image_back_msgs.append((t.to_sec(), msg))
            elif topic == self.image_topics["image_left_topic"]:
                image_left_msgs.append((t.to_sec(), msg))
            elif topic == self.image_topics["image_right_topic"]:
                image_right_msgs.append((t.to_sec(), msg))
            elif topic == self.image_topics["image_front_middle_topic"]:
                image_front_middle_msgs.append((t.to_sec(), msg))

        lidar_all_msgs = {
            "front": lidar_front_msgs,
            "back": lidar_back_msgs
        }
        image_all_msgs = {
            "front": image_front_msgs,
            "back": image_back_msgs,
            "left": image_left_msgs,
            "right": image_right_msgs,
            "front_middle": image_front_middle_msgs
        }
        
        print(f"Found {len(lidar_front_msgs)} lidar front messages and {len(lidar_back_msgs)} lidar back messages and {len(image_front_msgs)} image front messages and {len(image_back_msgs)} image back messages and {len(image_left_msgs)} image left messages and {len(image_right_msgs)} image right messages and {len(image_front_middle_msgs)} image front middle messages")
        bag.close()
=======
        # 获取消息
        lidar_msgs = []
        image_msgs = []
        
        print("Reading messages...")
        for topic, msg, t in tqdm(bag.read_messages(topics=[self.lidar_topic, self.image_topic])):
            if topic == self.lidar_topic:
                lidar_msgs.append((t.to_sec(), msg))
            elif topic == self.image_topic:
                image_msgs.append((t.to_sec(), msg))

        print(f"Found {len(lidar_msgs)} lidar messages and {len(image_msgs)} image messages")
        
        if len(lidar_msgs) == 0:
            print("Error: No lidar messages found")
            return

        if len(image_msgs) == 0:
            print("Error: No image messages found")
            return
>>>>>>> 38a7e901473e2eb7cd9f5f07a3e750ff014220b6

        def save_pointcloud_to_pcd(pc_msg, output_path):
            """将点云保存为PCD格式"""
            points = []
            for point in pc2.read_points(pc_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
                x, y, z, intensity = point
                points.append((x, y, z, intensity))

            # 写入PCD文件
            with open(output_path, 'w') as f:
                f.write("# .PCD v0.7 - Point Cloud Data\n")
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z intensity\n")
                f.write("SIZE 4 4 4 4\n")
                f.write("TYPE F F F F\n")
                f.write("COUNT 1 1 1 1\n")
                f.write(f"WIDTH {len(points)}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {len(points)}\n")
                f.write("DATA ascii\n")

                for point in points:
                    x, y, z, intensity = point
                    f.write(f"{x} {y} {z} {intensity}\n")

        def save_image_to_jpeg(img_msg, output_path):
            """将图像保存为JPEG格式"""
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            cv2.imwrite(output_path, cv_image)

        # 保存点云和图像数据
        print(f"Saving synchronized point clouds and images...")
<<<<<<< HEAD
        for name in self.names:
            lidar_dir = self.pointcloud_dirs[name]
            image_dir = self.images_dirs[name]
            lidar_msgs = lidar_all_msgs["back"] if name == "left" or name == "back" else lidar_all_msgs["front"]
            image_msgs = image_all_msgs[name]
            lidar_idx, img_idx, file_idx = 0, 0, 0
            while lidar_idx < len(lidar_msgs) and img_idx < len(image_msgs):
                lidar_time, pc_msg = lidar_msgs[lidar_idx]
                img_time, img_msg = image_msgs[img_idx]
                
                # 检查时间戳是否在时间容差范围内
                if abs(lidar_time - img_time) <= time_tolerance:
                    print(f"Matching lidar time: {lidar_time}, image time: {img_time}")
                    pcd_file = os.path.join(lidar_dir, f"{file_idx:04d}.pcd")
                    img_file = os.path.join(image_dir, f"{file_idx:04d}.png")
                    save_pointcloud_to_pcd(pc_msg, pcd_file)
                    save_image_to_jpeg(img_msg, img_file)
                    lidar_idx += 1
                    img_idx += 1
                    file_idx += 1
                elif lidar_time < img_time:
                    lidar_idx += 1
                else:
                    img_idx += 1

            
            print(f"Extraction of {name} complete!")

if __name__ == "__main__":
    bag_configs = {
        "bag_path":"camera-lidar.bag",
        "lidar_topics": {
            "lidar_front_topic": "/front/rslidar_points_unique",
            "lidar_back_topic": "/back/rslidar_points_unique"
        },
        "image_topics": {
            "image_front_topic": "/miivii_gmsl/image4",
            "image_back_topic": "/miivii_gmsl/image6",
            "image_left_topic": "/miivii_gmsl/image5",
            "image_right_topic": "/miivii_gmsl/image7",
            "image_front_middle_topic": "/miivii_gmsl/image0"
        },
        "output_dirs": ["fisheye-front", "pinhole-back", "fisheye-left", "fisheye-right", "pinhole-front"]
    }

    extractor = BagExtractor(bag_configs["bag_path"], bag_configs["lidar_topics"], bag_configs["image_topics"], bag_configs["output_dirs"])
    extractor.extract_sync_data()
=======
        lidar_idx, img_idx, file_idx = 0, 0, 0
        while lidar_idx < len(lidar_msgs) and img_idx < len(image_msgs):
            lidar_time, pc_msg = lidar_msgs[lidar_idx]
            img_time, img_msg = image_msgs[img_idx]
            
            # 检查时间戳是否在时间容差范围内
            if abs(lidar_time - img_time) <= time_tolerance and lidar_time - img_time <= 0:
                print(f"Matching lidar time: {lidar_time}, image time: {img_time}")
                pcd_file = os.path.join(self.lidar_dir, f"{file_idx:04d}.pcd")
                img_file = os.path.join(self.image_dir, f"{file_idx:04d}.png")
                save_pointcloud_to_pcd(pc_msg, pcd_file)
                save_image_to_jpeg(img_msg, img_file)
                lidar_idx += 1
                img_idx += 1
                file_idx += 1
            elif lidar_time < img_time:
                lidar_idx += 1
            else:
                img_idx += 1

        bag.close()
        print("Extraction complete!")

if __name__ == "__main__":
    bag_configs = {
        "camera-front.bag":[
            {
                "lidar_topic": "/rslidar_front_middle_points",
                "image_topic": "/miivii_gmsl/image4",
                "output_dir": "fisheye-front"
            },
            {
                "lidar_topic": "/rslidar_front_middle_points",
                "image_topic": "/miivii_gmsl/image0",
                "output_dir": "pinhole-front"
            }],
        "camera-back.bag":[
            {
                "lidar_topic": "/rslidar_back_middle_points",
                "image_topic": "/miivii_gmsl/image6",
                "output_dir": "pinhole-back"
            }],
        "camera-left.bag":[
            {
                "lidar_topic": "/rslidar_left_middle_points",
                "image_topic": "/miivii_gmsl/image5",
                "output_dir": "fisheye-left"
            }],
        "camera-right.bag":[
            {
                "lidar_topic": "/rslidar_right_middle_points",
                "image_topic": "/miivii_gmsl/image7",
                "output_dir": "fisheye-right"
            }]
    }
    for bag_path, configs in bag_configs.items():
        for config in configs:
            output_dir = config["output_dir"]
            camera_type = output_dir.split('-')[0]
            extractor = BagExtractor(bag_path, config["lidar_topic"], config["image_topic"], output_dir)
            extractor.extract_sync_data()
>>>>>>> 38a7e901473e2eb7cd9f5f07a3e750ff014220b6
