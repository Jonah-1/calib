import json
import numpy as np

def invert_matrices_in_json(input_file, output_file):
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
        except np.linalg.LinAlgError:
            print(f"Matrix for {key} is not invertible.")
    
    # 写回 JSON 文件
    with open(output_file, 'w') as f:
        json.dump(data, f, indent=4)

# 使用示例
input_file = 'lidar2camera.json'
output_file = 'camera2lidar.json'
invert_matrices_in_json(input_file, output_file)