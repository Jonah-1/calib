import numpy as np
import math
import json

# 输入欧拉角和平移向量，用一行表示
input_data = "4.442716 -5.604157 0 0.026938 -0.054964 0.135280"
data = list(map(float, input_data.split()))

# 解析输入数据
roll_deg, pitch_deg, yaw_deg = data[:3]
t = np.array(data[3:])

# 将角度转换为弧度
roll = math.radians(roll_deg)
pitch = math.radians(pitch_deg)
yaw = math.radians(yaw_deg)

# 计算旋转矩阵
R_x = np.array([
    [1, 0, 0],
    [0, math.cos(roll), -math.sin(roll)],
    [0, math.sin(roll), math.cos(roll)]
])

R_y = np.array([
    [math.cos(pitch), 0, math.sin(pitch)],
    [0, 1, 0],
    [-math.sin(pitch), 0, math.cos(pitch)]
])

R_z = np.array([
    [math.cos(yaw), -math.sin(yaw), 0],
    [math.sin(yaw), math.cos(yaw), 0],
    [0, 0, 1]
])

# 组合旋转矩阵
R = np.dot(R_z, np.dot(R_y, R_x))

# 构建旋转平移矩阵
T = np.eye(4)
T[:3, :3] = R
T[:3, 3] = t

# 定义一个函数来处理非常接近于零的浮点数
def round_small_numbers(num):
    if abs(num) < 1e-6:
        return 0.0
    return num

# 应用阈值处理
T = np.vectorize(round_small_numbers)(T)

# 转换为 JSON 格式
output = {
    "transformation_matrix": T.tolist()
}

# 输出 JSON
print(json.dumps(output, indent=4))

# 定义 JSON 文件路径
json_path = "/home/ljh/project/factory/camera_to_lidar/lidar2camera/manual_calib/data/0000/top_center_lidar-to-center_camera-extrinsic.json"

# 打开并读取 JSON 文件
with open(json_path, 'r') as f:
    data = json.load(f)

print(data)

# 更新 JSON 文件中的 'data' 键
data['top_center_lidar-to-center_camera-extrinsic']['param']['sensor_calib']['data'] = T.tolist()

# 将更改后的数据保存回文件
with open(json_path, 'w') as f:
    json.dump(data, f, indent=4)

print("JSON 文件已更新。")

# 输出矩阵按一行排列，不加逗号的格式
flattened = T.flatten()
one_line = " ".join(f"{num:.6f}" for num in flattened)
print(one_line)