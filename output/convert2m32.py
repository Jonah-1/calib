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

def compute_camera2m32(camera2lidar, lidar2m32, output_path, camera_key, lidar_key, output_key):
    # 使用新函数获取矩阵
    camera_matrix, lidar_matrix = get_matrices(camera2lidar, lidar2m32, camera_key, lidar_key)

    # 计算camera2m32矩阵
    camera2m32_matrix = np.dot(lidar_matrix, camera_matrix)

    # 读取现有数据
    try:
        with open(output_path, 'r') as outfile:
            output_data = json.load(outfile)
    except FileNotFoundError:
        output_data = {}

    # 更新数据
    output_data[output_key] = camera2m32_matrix.tolist()

    # 将结果保存到输出文件中
    with open(output_path, 'w') as outfile:
        json.dump(output_data, outfile, indent=4)

    print(f"{output_key} Matrix has been saved to {output_path}")

def invert_matrices_in_json(input_file, output_path):
    # 读取 JSON 文件
    with open(input_file, 'r') as f:
        data = json.load(f)
    
    # 遍历每个键，计算矩阵的逆
    for key, matrix in data.items():
        # 转换为 numpy 数组
        matrix_np = np.array(matrix)
        try:
            # 计算逆矩阵
            inverse_matrix = np.linalg.inv(matrix_np)
            # 将结果转换为列表并更新
            data[key] = inverse_matrix.tolist()
            print(f"{key} invert Matrix has been saved to {output_path}")
        except np.linalg.LinAlgError:
            print(f"Matrix for {key} is not invertible.")
    
    # 写回 JSON 文件
    with open(output_path, 'w') as f:
        json.dump(data, f, indent=4)

# 加载JSON文件
camera2lidar = load_json('camera2lidar.json')
lidar2m32 = load_json('lidar2m32.json')

# 定义键和输出路径列表
camera_keys = ["CAM_FRONT_8M","CAM_FRONT_3M", "CAM_LEFT_3M", "CAM_RIGHT_3M","CAM_BACK_3M"]
lidar_keys = ["LIDAR_FRONT", "LIDAR_FRONT", "LIDAR_BACK","LIDAR_FRONT","LIDAR_BACK"]
output_keys = ["CAM_FRONT_8M", "CAM_FRONT_3M", "CAM_LEFT_3M", "CAM_RIGHT_3M","CAM_BACK_3M"]
output_path = "camera2m32.json"
invert_path="32m2cameras.json"

# 清空输出文件
with open(output_path, 'w') as f:
    json.dump({}, f)
# 分别调用函数
for camera_key, lidar_key, output_key in zip(camera_keys, lidar_keys, output_keys):
    compute_camera2m32(camera2lidar, lidar2m32, output_path, camera_key, lidar_key, output_key)

invert_matrices_in_json(output_path, invert_path)