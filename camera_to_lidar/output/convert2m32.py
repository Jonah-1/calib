import numpy as np
import json

def load_json(file_path):
    """加载JSON文件并返回字典"""
    with open(file_path, 'r') as file:
        return json.load(file)

def get_matrices(camera2lidar, lidar2m32, camera_key, lidar_key):
    """根据指定的键获取相应的矩阵"""
    camera_matrix = np.array(camera2lidar[camera_key])
    lidar_matrix = np.array(lidar2m32[lidar_key])
    return camera_matrix, lidar_matrix

def compute_frontfisheye2m(camera2lidar, lidar2m32, output_path, camera_key, lidar_key, output_key):
    # 使用新函数获取矩阵
    camera_matrix, lidar_matrix = get_matrices(camera2lidar, lidar2m32, camera_key, lidar_key)

    # 计算frontfisheye2m矩阵
    frontfisheye2m_matrix = np.dot(lidar_matrix, camera_matrix)

    # 读取现有数据
    try:
        with open(output_path, 'r') as outfile:
            output_data = json.load(outfile)
    except FileNotFoundError:
        output_data = {}

    # 更新数据
    output_data[output_key] = frontfisheye2m_matrix.tolist()

    # 将结果保存到输出文件中
    with open(output_path, 'w') as outfile:
        json.dump(output_data, outfile, indent=4)

    print(f"{output_key} Matrix has been saved to {output_path}")

# 加载JSON文件
camera2lidar = load_json('camera2lidar.json')
lidar2m32 = load_json('lidar2m32.json')

# 定义键和输出路径列表
camera_keys = ["front-fisheye", "left-fisheye", "right-fisheye","front-pinhole","back-pinhole"]
lidar_keys = ["front", "back", "front","front","back"]
output_keys = ["front-fisheye2m32", "left-fisheye2m32", "right-fisheye2m32","front-pinhole2m32","back-pinhole2m32"]
output_path = "camera2m32.json"

# 清空输出文件
with open(output_path, 'w') as f:
    json.dump({}, f)
# 分别调用函数
for camera_key, lidar_key, output_key in zip(camera_keys, lidar_keys, output_keys):
    compute_frontfisheye2m(camera2lidar, lidar2m32, output_path, camera_key, lidar_key, output_key)