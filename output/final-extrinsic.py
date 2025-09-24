import numpy as np
import json

def load_json(file_path):
    """加载JSON文件并返回字典"""
    with open(file_path, 'r') as file:
        return json.load(file)

def get_matrices(file_data, input_key):
    """根据指定的键获取相应的矩阵"""
    matrix = np.array(file_data[input_key])
    return matrix

def compute_sensor2mcenter(camera2m32, lidar2m32, output_path, input_key, output_key):
    #工装值
    center_matrix = np.array([[1, 0, 0, 1.2], [0, 1, 0, 0], [0, 0, 1, 1.76], [0, 0, 0, 1]])

    if 'CAM' in input_key:
        input_data = camera2m32
    elif input_key == "LIDAR_ZHU":
        input_data = np.eye(4)
    else:
        input_data = lidar2m32

    # 使用新函数获取矩阵
    if input_key == "LIDAR_ZHU":
        matrix = input_data
    else:
        matrix = get_matrices(input_data, input_key)

    # 计算sensor2mcenter矩阵
    sensor2mcenter_matrix = np.dot(center_matrix, matrix)

    # 读取现有数据
    try:
        with open(output_path, 'r') as outfile:
            output_data = json.load(outfile)
    except FileNotFoundError:
        output_data = {}

    # 更新数据
    output_data[output_key] = sensor2mcenter_matrix.tolist()

    # 将结果保存到输出文件中
    with open(output_path, 'w') as outfile:
        json.dump(output_data, outfile, indent=4)

    print(f"{output_key} Matrix has been saved to {output_path}")

# 加载JSON文件
camera2m32 = load_json('camera2m32.json')
lidar2m32 = load_json('lidar2m32.json')

# 定义键和输出路径列表
camera_keys = ["CAM_FRONT_8M", "CAM_FRONT_3M", "CAM_LEFT_3M", "CAM_RIGHT_3M","CAM_BACK_3M"]
lidar_keys = ["LIDAR_FRONT","LIDAR_BACK"]
output_keys = ["CAM_FRONT_8M", "CAM_FRONT_3M", "CAM_LEFT_3M", "CAM_RIGHT_3M","CAM_BACK_3M","LIDAR_FRONT", "LIDAR_BACK","LIDAR_ZHU"]
output_path = "final-extrinsic.json"

# 清空输出文件
with open(output_path, 'w') as f:
    json.dump({}, f)
# 分别调用函数
all_keys = camera_keys + lidar_keys+ ["LIDAR_ZHU"]
for input_key, output_key in zip(all_keys, output_keys):
    compute_sensor2mcenter(camera2m32, lidar2m32, output_path, input_key, output_key)
