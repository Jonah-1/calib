import cv2
import numpy as np
import os
import glob
import re
from pathlib import Path
from itertools import chain
import json
import argparse
import random
import shutil

def update_camera_config(camera_config_path, param_files, input_dirs):
    # 初始化空的配置数组
    camera_config = []

    # 遍历每个相机配置
    for i, (param_file, input_dir) in enumerate(zip(param_files, input_dirs)):
        # 获取内参
        if "pinhole-front" in input_dir: # Special case for pinhole-front
            fx = 1910.3417311410
            fy = 1910.3058674355
            cx = 1917.7001038394
            cy = 1081.4421265044
        else:
            internal_params = read_camera_parameters(param_file)
            fx = internal_params.get("FX")
            fy = internal_params.get("FY")
            cx = internal_params.get("CX")
            cy = internal_params.get("CY")

        # 构建单个相机配置 (移除外参)
        camera_entry = {
            "camera_internal": {
                "fx": fx,
                "fy": fy,
                "cx": cx,
                "cy": cy
            },
            "width": 1920,
            "height": 1536,
            "rowMajor": True
        }

        # 添加到配置列表
        camera_config.append(camera_entry)

    # 写入新文件
    with open(camera_config_path, 'w') as f:
        json.dump(camera_config, f, indent=4)

    print(f"✅ 已成功创建并写入 {camera_config_path}")

def generate_camera_config_dir(camera_config_path, output_dirs):
    # This function seems to depend on scene_dir which is not well defined
    # in this script version. Also depends on lidar point clouds which might
    # not be present. I will comment out the call to it.
    # If you need this functionality, we can fix it separately.
    pass

def read_camera_parameters(param_file):
    """从参数文件中读取相机内参和畸变系数"""
    params = {}
    with open(param_file, 'r', encoding='utf-8') as f:
        lines = f.readlines()
        
    for line in lines:
        # 使用冒号分割键值
        if ':' in line:
            key, value = line.split(':', 1)
            key = key.strip()
            value = value.strip()
            
            # 忽略值为'/'或'null'的参数
            if value != '/' and value != 'null' and value != '':
                try:
                    # 尝试将值转换为浮点数
                    params[key] = float(value)
                except ValueError:
                    # 如果不能转换为浮点数，则保存为字符串
                    params[key] = value
    
    return params


def undistort_fisheye_images(param_file, input_dir, output_dir, selection_mode='random', num_frames=3, camera_name=None, frame_selection_dict=None):
    """对鱼眼相机拍摄的图像进行去畸变处理"""
    # 读取参数
    params = read_camera_parameters(param_file)
    
    # 提取相机内参
    fx = params.get('FX')
    fy = params.get('FY')
    cx = params.get('CX')
    cy = params.get('CY')
    
    # 提取畸变系数
    k1 = params.get('K1', 0.0)
    k2 = params.get('K2', 0.0)
    k3 = params.get('K3', 0.0)
    k4 = params.get('K4', 0.0)
    
    # 检查必要的参数是否存在
    required_params = ['FX', 'FY', 'CX', 'CY']
    if not all(param in params for param in required_params):
        missing = [param for param in required_params if param not in params]
        raise ValueError(f"缺少必要的相机参数: {', '.join(missing)}")
    
    # 创建相机矩阵
    camera_matrix = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ])
    
    # 畸变系数 (k1, k2, p1, p2, k3, k4, k5, k6)
    # OpenCV的鱼眼相机模型使用k1, k2, k3, k4作为畸变系数
    dist_coeffs = np.array([k1, k2, k3, k4])
    
    # 清空并重新创建输出目录
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir, exist_ok=True)
    print(f"已清空并重新创建输出目录: {output_dir}")
    
    # 获取输入目录中的所有图像并按数字排序
    all_image_files = glob.glob(os.path.join(input_dir, '*.png')) + glob.glob(os.path.join(input_dir, '*.jpg'))
    # 按文件名中的数字进行排序
    all_image_files.sort(key=lambda x: int(re.findall(r'\d+', os.path.basename(x))[0]) if re.findall(r'\d+', os.path.basename(x)) else 0)
    
    # 根据选择模式处理图片
    if selection_mode == 'random':
        # 随机选择指定数量的图片
        if len(all_image_files) > num_frames:
            image_files = random.sample(all_image_files, num_frames)
            print(f"从 {len(all_image_files)} 张图片中随机选择了 {num_frames} 张进行处理")
        else:
            image_files = all_image_files
            print(f"总共只有 {len(image_files)} 张图片，全部处理")
    elif selection_mode == 'select' and camera_name and frame_selection_dict and camera_name in frame_selection_dict:
        # 从字典中选择特定帧
        frame_indices = frame_selection_dict[camera_name]['frames']
        
        # 过滤出有效的索引
        valid_indices = [idx for idx in frame_indices if idx < len(all_image_files)]
        image_files = [all_image_files[idx] for idx in valid_indices]
        
        print(f"从 {len(all_image_files)} 张图片中选择了索引 {valid_indices} 的图片进行处理")
        
        if len(valid_indices) < len(frame_indices):
            missing_indices = [idx for idx in frame_indices if idx >= len(all_image_files)]
            print(f"警告: 索引 {missing_indices} 超出范围，已跳过")
    else:
        # 如果执行到这里说明参数配置有问题，直接退出
        print(f"错误: 无效的选择模式 '{selection_mode}' 或缺少必要的参数")
        raise ValueError(f"无效的图片选择配置: mode={selection_mode}, camera={camera_name}")
    
    # 处理选中的图像
    for image_file in image_files:
        # 读取图像
        img = cv2.imread(image_file)
        if img is None:
            print(f"无法读取图像: {image_file}")
            continue
        
        # 获取图像尺寸
        h, w = img.shape[:2]
        
        # 计算新的相机矩阵
        new_camera_matrix = camera_matrix.copy()
        
        # 使用OpenCV的鱼眼相机模型进行去畸变
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            camera_matrix, 
            dist_coeffs, 
            np.eye(3), 
            new_camera_matrix, 
            (w, h), 
            cv2.CV_16SC2
        )
        undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT)
        
        # 打印去畸变后的图像尺寸
        # print(f"去畸变后的图像尺寸: {undistorted_img.shape[1]}x{undistorted_img.shape[0]}")
        
        # 构建输出文件路径
        filename = os.path.basename(image_file)
        output_file = os.path.join(output_dir, filename)
        
        # 保存去畸变后的图像
        cv2.imwrite(output_file, undistorted_img)
        print(f"已处理: {image_file}")

def crop_image(image, cx, cy, crop_percent=1):
    """从光学中心裁剪图像"""
    width=3840
    height=2160
    
    # 计算裁剪区域
    crop_width = 1920  # 直接指定目标宽度
    crop_height = 1536  # 直接指定目标高度
    
    # 计算裁剪区域的边界（从中心开始）
    left = int((width - crop_width) / 2)
    right = left + crop_width
    top = int((height - crop_height) / 2)
    bottom = top + crop_height
    
    # 裁剪图像
    cropped = image[top:bottom, left:right]
    # print(f"\n裁剪后的图像尺寸: {cropped.shape[1]}x{cropped.shape[0]}")
    return cropped

def calculate_new_camera_matrix(params, input_dir, crop_percent=1):
    """计算裁剪后的相机内参矩阵"""
    fx = params.get('FX')
    fy = params.get('FY')
    cx = params.get('CX')
    cy = params.get('CY')
    
    # 读取一张图片来获取尺寸，支持jpg和png格式
    input_path = Path(input_dir)
    sample_image = next(chain(input_path.glob("*.jpg"), input_path.glob("*.png")), None)
    if sample_image:
        image = cv2.imread(str(sample_image))
        if image is not None:
            width=3840
            height=2160

            
            # 计算裁剪区域
            crop_width = width-1920
            crop_height = height-1536
            # 计算裁剪边界
            left = max(0, int(cx - crop_width/2))
            top = max(0, int(cy - crop_height/2))
            
            # 计算裁剪后的主点坐标
            # 新的主点坐标需要减去裁剪的偏移量
            new_cx = cx - left  # left是裁剪的起始x坐标
            new_cy = cy - top   # top是裁剪的起始y坐标
            
            # 构建原始相机矩阵
            camera_matrix = np.array([
                [fx, 0, cx],
                [0, fy, cy],
                [0, 0, 1]
            ])
            
            # 构建新的相机矩阵
            new_camera_matrix = np.array([
                [fx, 0, new_cx],
                [0, fy, new_cy],
                [0, 0, 1]
            ])
            
            return camera_matrix, new_camera_matrix
    
    return None, None

def process_fisheye_camera(param_file, input_dir, output_dir, selection_mode='random', num_frames=3, camera_name=None, frame_selection_dict=None):
    try:
        print(f'开始处理camera: {input_dir}中的图片')
        undistort_fisheye_images(param_file, input_dir, output_dir, selection_mode, num_frames, camera_name, frame_selection_dict)
        print(f"所有图像已处理完成并保存到 {output_dir} 目录")
    except Exception as e:
        print(f"处理出错: {e}")

def undistort_pinhole_image(image_path, params, input_dir):
    # 读取图像
    img = cv2.imread(image_path)
    if img is None:
        raise ValueError(f"无法读取图像: {image_path}")

    # 获取原始图像尺寸
    h, w = img.shape[:2]
    
    # 构建相机矩阵
    camera_matrix = np.array([
        [params['FX'], 0, params['CX']],
        [0, params['FY'], params['CY']],
        [0, 0, 1]
    ])
    
    # 构建畸变系数
    dist_coeffs = np.array([
        params['K1'], params['K2'], params['P1'], params['P2'],
        params['K3'], params['K4'], params['K5'], params['K6']
    ])
    
    # 去畸变
    undistorted_img = cv2.undistort(img, camera_matrix, dist_coeffs)

    if input_dir==f"pinhole-front/pinhole-images":
          undistorted_img=crop_image(undistorted_img,params['CX'],params['CY'])
          camera_matrix, new_camera_matrix=calculate_new_camera_matrix(params, input_dir)
        #   print("\n原始相机矩阵:")
        #   print(camera_matrix)
        #   print("\n新的相机矩阵:")
        #   print(new_camera_matrix)
    
    return undistorted_img, camera_matrix

def process_pinhole_image(param_file, input_dir, output_dir, selection_mode='random', num_frames=3, camera_name=None, frame_selection_dict=None):
        try:
            # 清空并重新创建输出目录
            if os.path.exists(output_dir):
                shutil.rmtree(output_dir)
            os.makedirs(output_dir, exist_ok=True)
            print(f"已清空并重新创建输出目录: {output_dir}")
            
            params = read_camera_parameters(param_file)
            all_image_files = glob.glob(os.path.join(input_dir, '*.jpg'))+glob.glob(os.path.join(input_dir, '*.png'))
            # 按文件名中的数字进行排序
            all_image_files.sort(key=lambda x: int(re.findall(r'\d+', os.path.basename(x))[0]) if re.findall(r'\d+', os.path.basename(x)) else 0)
            
            # 根据选择模式处理图片
            if selection_mode == 'random':
                # 随机选择指定数量的图片
                if len(all_image_files) > num_frames:
                    image_files = random.sample(all_image_files, num_frames)
                    print(f"从 {len(all_image_files)} 张图片中随机选择了 {num_frames} 张进行处理")
                else:
                    image_files = all_image_files
                    print(f"总共只有 {len(image_files)} 张图片，全部处理")
            elif selection_mode == 'select' and camera_name and frame_selection_dict and camera_name in frame_selection_dict:
                # 从字典中选择特定帧
                frame_indices = frame_selection_dict[camera_name]['frames']
                
                # 过滤出有效的索引
                valid_indices = [idx for idx in frame_indices if idx < len(all_image_files)]
                image_files = [all_image_files[idx] for idx in valid_indices]
                
                print(f"从 {len(all_image_files)} 张图片中选择了索引 {valid_indices} 的图片进行处理")
                
                if len(valid_indices) < len(frame_indices):
                    missing_indices = [idx for idx in frame_indices if idx >= len(all_image_files)]
                    print(f"警告: 索引 {missing_indices} 超出范围，已跳过")
            else:
                # 如果执行到这里说明参数配置有问题，直接退出
                print(f"错误: 无效的选择模式 '{selection_mode}' 或缺少必要的参数")
                raise ValueError(f"无效的图片选择配置: mode={selection_mode}, camera={camera_name}")
            
            # 处理选中的图片
            for image_path in image_files:
                try:
                    cropped_img, new_camera_matrix = undistort_pinhole_image(image_path, params,input_dir)
                    # 获取原始文件名
                    filename = os.path.basename(image_path)
                    # 构建输出文件路径
                    output_path = os.path.join(output_dir, f'{filename}')
                    # 保存处理后的图片
                    cv2.imwrite(output_path, cropped_img)
                    print(f"已保存处理后的图片: {output_path}")


                except Exception as e:
                    print(f"处理图像 {image_path} 时出错: {str(e)}")

            # 打印新的相机参数
            print("新的相机参数:")
            print(f"FX: {new_camera_matrix[0, 0]:.10f}")
            print(f"FY: {new_camera_matrix[1, 1]:.10f}")
            print(f"CX: {new_camera_matrix[0, 2]:.10f}")
            print(f"CY: {new_camera_matrix[1, 2]:.10f}\n")
            
        except Exception as e:
            print(f"处理图像时出错: {str(e)}")


def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='相机图像去畸变处理工具')
    parser.add_argument('--mode', choices=['random', 'select'], default='random',
                        help='图片选择模式: random(随机选择), select(选择特定帧)')
    parser.add_argument('--frames', type=int, default=3,
                        help='每个相机处理的图片数量 (默认: 3)')
    parser.add_argument('--cameras', nargs='+', 
                        choices=['pinhole-back', 'pinhole-front', 'fisheye-front', 'fisheye-left', 'fisheye-right'],
                        default=['pinhole-back', 'pinhole-front', 'fisheye-front', 'fisheye-left', 'fisheye-right'],
                        help='指定要处理的相机 (默认: 处理所有相机)')
    return parser.parse_args()

if __name__ == "__main__":
    # 解析命令行参数
    args = parse_arguments()

    camera_frame_selection = {
        'pinhole-back': {
            'frames': [0, 5, 10],
        },
        'pinhole-front': {
            'frames': [2, 7, 12],
        },
        'fisheye-front': {
            'frames': [1, 6, 11],
        },
        'fisheye-left': {
            'frames': [3, 8, 13],
        },
        'fisheye-right': {
            'frames': [4, 9, 14],
        }
    }

    
    # 相机配置
    camera_configs = {
        'pinhole-back': {
            'param_file': "Parameters/pinhole-back.txt",
            'input_dir': "pinhole-back/images",
            'output_dir': "pinhole-back/undistorted"
        },
        'pinhole-front': {
            'param_file': "Parameters/pinhole-front.txt",
            'input_dir': "pinhole-front/images",
            'output_dir': "pinhole-front/undistorted"
        },
        'fisheye-front': {
            'param_file': "Parameters/fisheye-front.txt",
            'input_dir': "fisheye-front/images",
            'output_dir': "fisheye-front/undistorted"
        },
        'fisheye-left': {
            'param_file': "Parameters/fisheye-left.txt",
            'input_dir': "fisheye-left/images",
            'output_dir': "fisheye-left/undistorted"
        },
        'fisheye-right': {
            'param_file': "Parameters/fisheye-right.txt",
            'input_dir': "fisheye-right/images",
            'output_dir': "fisheye-right/undistorted"
        }
    }
    
    print(f"图片选择模式: {args.mode}")
    if args.mode == 'select':
        print("📋 使用字典中预定义的帧选择配置")
    else:
        print(f"每个相机处理图片数量: {args.frames}")
    print(f"处理的相机: {', '.join(args.cameras)}")
    print("-" * 50)
    
    # 处理指定的相机
    processed_cameras = []
    for camera_name in args.cameras:
        if camera_name in camera_configs:
            config = camera_configs[camera_name]
            print(f"\n处理相机: {camera_name}")
            
            if "pinhole" in camera_name:
                process_pinhole_image(config['param_file'], config['input_dir'], 
                                    config['output_dir'], args.mode, args.frames, camera_name, camera_frame_selection)
            else:
                process_fisheye_camera(config['param_file'], config['input_dir'], 
                                     config['output_dir'], args.mode, args.frames, camera_name, camera_frame_selection)
            processed_cameras.append(camera_name)
        else:
            print(f"警告: 未找到相机配置 {camera_name}")
    
    # 更新相机配置文件
    if processed_cameras:
        param_files = [camera_configs[cam]['param_file'] for cam in processed_cameras]
        input_dirs = [camera_configs[cam]['input_dir'] for cam in processed_cameras]
        
        camera_config_path = "Parameters/camera_config.json"
        update_camera_config(camera_config_path, param_files, input_dirs)
    
    print(f"\n✅ 处理完成! 共处理了 {len(processed_cameras)} 个相机: {', '.join(processed_cameras)}")
