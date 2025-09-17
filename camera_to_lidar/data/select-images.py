import os
import glob
import random
import shutil

def copy_random_images(input_dir, output_dir, num_images=3):
    """直接随机复制图片到指定文件夹"""
    # 清空并重新创建输出目录
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir, exist_ok=True)
    print(f"已清空并重新创建输出目录: {output_dir}")
    
    # 获取输入目录中的所有图像
    image_files = glob.glob(os.path.join(input_dir, '*.png')) + glob.glob(os.path.join(input_dir, '*.jpg'))
    
    if not image_files:
        print(f"在 {input_dir} 中没有找到图片文件")
        return
    
    # 随机选择指定数量的图片
    if len(image_files) > num_images:
        selected_files = random.sample(image_files, num_images)
        print(f"从 {len(image_files)} 张图片中随机选择了 {num_images} 张进行复制")
    else:
        selected_files = image_files
        print(f"总共只有 {len(image_files)} 张图片，全部复制")
    
    # 复制选中的图片
    for image_file in selected_files:
        filename = os.path.basename(image_file)
        output_file = os.path.join(output_dir, filename)
        try:
            shutil.copy2(image_file, output_file)
            print(f"已复制: {filename}")
        except Exception as e:
            print(f"复制文件 {filename} 时出错: {e}")

if __name__ == "__main__":
    # 定义输入输出目录（根据实际情况修改）
    input_dirs = [
        "pinhole-back/images",
        "pinhole-front/images",
        "fisheye-front/images",
        "fisheye-left/images",
        "fisheye-right/images"
    ]
    output_dirs = [
        "pinhole-back/undistorted",
        "pinhole-front/undistorted",
        "fisheye-front/undistorted",
        "fisheye-left/undistorted",
        "fisheye-right/undistorted"
    ]
    
    # 固定复制 3 张
    num_images = 3
    print(f"模式：直接复制 {num_images} 张随机图片（不去畸变）")
    
    for input_dir, output_dir in zip(input_dirs, output_dirs):
        if os.path.exists(input_dir):
            print(f"\n开始处理 camera: {input_dir} 中的图片")
            copy_random_images(input_dir, output_dir, num_images)
        else:
            print(f"⚠️ 警告：输入目录不存在 - {input_dir}")
    
    print("\n✅ 所有图片复制完成")