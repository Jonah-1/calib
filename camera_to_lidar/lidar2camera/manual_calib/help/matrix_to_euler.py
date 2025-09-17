import numpy as np
import math
import json
import argparse

def matrix_to_euler(transformation_matrix):
    """
    将4x4变换矩阵转换为欧拉角(ZYX顺序，即yaw-pitch-roll)和平移向量
    """
    # 提取旋转矩阵部分
    R = transformation_matrix[:3, :3]
    
    # 提取平移向量
    t = transformation_matrix[:3, 3]
    
    # 从旋转矩阵提取欧拉角
    # 使用ZYX欧拉角顺序 (yaw->pitch->roll)
    
    # 计算pitch角度 (绕Y轴)
    pitch = -math.asin(R[2, 0])
    
    # 检查是否接近奇异点 (万向锁)
    if abs(R[2, 0]) < 0.99999:
        # 正常情况
        yaw = math.atan2(R[1, 0], R[0, 0])  # 绕Z轴
        roll = math.atan2(R[2, 1], R[2, 2])  # 绕X轴
    else:
        # 接近奇异点 (pitch接近±90度)
        # 在这种情况下，yaw和roll不能唯一确定 (万向锁)
        # 通常将一个角度设为0，并计算另一个
        yaw = math.atan2(-R[0, 1], R[1, 1])
        roll = 0.0
    
    # 将弧度转换为角度
    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    yaw_deg = math.degrees(yaw)
    
    # 规范化角度到[-180, 180)区间
    roll_deg = (roll_deg + 180) % 360 - 180
    pitch_deg = (pitch_deg + 180) % 360 - 180
    yaw_deg = (yaw_deg + 180) % 360 - 180
    
    return roll_deg, pitch_deg, yaw_deg, t

def main():
    parser = argparse.ArgumentParser(description="将变换矩阵转换为欧拉角和平移向量")
    parser.add_argument("--matrix", default=None, help="16个浮点数表示的4x4变换矩阵，以空格分隔")
    parser.add_argument("--json_file", default=None, help="包含变换矩阵的JSON文件路径")
    args = parser.parse_args()
    
    if args.matrix:
        # 从命令行参数读取矩阵
        matrix_data = list(map(float, args.matrix.split()))
        matrix = np.array(matrix_data).reshape(4, 4)
    elif args.json_file:
        # 从JSON文件读取矩阵
        with open(args.json_file, 'r') as f:
            data = json.load(f)
            
        # 尝试从常见的键层次结构中找到变换矩阵
        if 'transformation_matrix' in data:
            matrix = np.array(data['transformation_matrix'])
        elif 'data' in data:
            matrix = np.array(data['data'])
        elif args.json_file.endswith('extrinsic.json'):
            try:
                # 根据参考程序中的路径推测结构
                key = next(iter(data.keys()))
                matrix = np.array(data[key]['param']['sensor_calib']['data'])
            except (KeyError, StopIteration):
                print("无法从JSON文件中提取变换矩阵，请指定正确的路径")
                return
        else:
            print("无法从JSON文件中提取变换矩阵，请指定正确的路径")
            return
    else:
        # 如果没有指定输入，尝试读取默认JSON文件
        try:
            json_path = "/home/ljh/project/factory/camera_to_lidar/lidar2camera/manual_calib/data/0001/top_center_lidar-to-center_camera-extrinsic.json"
            with open(json_path, 'r') as f:
                data = json.load(f)
            
            key = next(iter(data.keys()))
            matrix = np.array(data[key]['param']['sensor_calib']['data'])
        except (FileNotFoundError, KeyError, StopIteration):
            # 如果无法读取默认文件，使用一个示例矩阵
            print("未提供输入矩阵，使用示例矩阵...")
            # 这是原始代码中 "0 0 180 0 0 -20" 对应的矩阵
            matrix = np.array([
      [-0.10616063, -0.02158067, -0.99411496, -1.62700955],
      [-0.99424816, 0.0165518, 0.10581554, 0.09839354],
      [0.01417081, 0.99963008, -0.02321369, -0.39988954],
      [0.0, 0.0, 0.0, 1.0]
            ])
    
    # 提取欧拉角和平移向量
    roll, pitch, yaw, translation = matrix_to_euler(matrix)
    
    # 输出结果
    print(f"变换矩阵:\n{matrix}")
    print("\n欧拉角 (度) 和平移向量:")
    print(f"Roll (X轴): {roll:.6f}°")
    print(f"Pitch (Y轴): {pitch:.6f}°")
    print(f"Yaw (Z轴): {yaw:.6f}°")
    print(f"平移向量: [{translation[0]:.6f}, {translation[1]:.6f}, {translation[2]:.6f}]")
    
    # 按照与原始程序相同的格式输出一行
    one_line = f"{roll:.6f} {pitch:.6f} {yaw:.6f} {translation[0]:.6f} {translation[1]:.6f} {translation[2]:.6f}"
    print(f"\n一行格式输出: {one_line}")

if __name__ == "__main__":
    main() 