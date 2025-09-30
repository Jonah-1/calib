## 0 环境配置

进入根目录下 camera_to_lidar文件夹

```
conda activate calib
pip install -r requirement.txt
```



### **运行过程如果遇到报错**

1.ImportError: /usr/lib/x86_64-linux-gnu/libp11-kit.so.0: undefined symbol: ffi_type_pointer, version LIBFFI_BASE_7.0

```
rm ${CONDA_PREFIX}/lib/libffi.7.so
rm ${CONDA_PREFIX}/lib/libffi.so.7
```

参考https://blog.csdn.net/CCCDeric/article/details/142342421?fromshare=blogdetail&sharetype=blogdetail&sharerId=142342421&sharerefer=PC&sharesource=2301_77479117&sharefrom=from_link

2.ImportError: /lib/libgdal.so.26: undefined symbol: TIFFReadRGBATileExt, version LIBTIFF_4.0

```
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libtiff.so.5
```

参考https://blog.csdn.net/qq_39779233/article/details/140839443?fromshare=blogdetail&sharetype=blogdetail&sharerId=140839443&sharerefer=PC&sharesource=2301_77479117&sharefrom=from_link



## 1 数据采集

首先分别使用前后左右的各相机（前面有两个）同时与各自最近的前后左右激光雷达同时采集点云图片，保存到一个文件夹，文件夹目录结构为

```
raw-data    #保存的名字，可更改
├── CAM_FRONT_8M
├── CAM_FRONT_3M
├── CAM_LEFT_3M
├── CAM_RIGHT_3M
├── CAM_BACK_3M
├── LIDAR_FRONT
├── LIDAR_REAR
├── LIDAR_TOP_32
```

## 2 数据处理

### 2.1 数据读取

进入camera_to_lidar/data文件夹，首先运行程序合并点云
```
python merge_pcd.py --path raw-data
```

然后运行程序，把下载好的数据转到到各对应名字的文件夹中（fisheye-front，fisheye-left，fisheye-right, pinhole-back, pinhole-front）

```
python process.py --dir raw-data/ --sync --time-tolerance 0.1
```

通过dir指定下载好的数据文件夹，--sync 同步pcd和png时间戳，--time-tolerance 同步最小容忍时间

### 2.2 去畸变

运行程序对每个鱼眼相机和针孔相机的图片去畸变，并保存到对应文件夹的undistorted文件夹中

```
python undistort.py --mode select
```

select模式下可以在代码里选择特征明显的帧来标定，改为random的话则随机选2张

```

if __name__ == "__main__":
    # 解析命令行参数
    args = parse_arguments()

    camera_frame_selection = {
        'pinhole-front': {
            'frames': [1, 2],
        },
        'fisheye-front': {
            'frames': [1, 2],
        },
        'fisheye-left': {
            'frames': [2,3],
        },
        'fisheye-right': {
            'frames': [1,4],
        },
        'pinhole-back': {
            'frames': [0, 2],
        }
    }
```

### 2.3 生成掩码

进入camera_to_lidar/data/segment-anything文件夹，首先编译

```
conda create -n seg python=3.8
conda activate seg

pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install -e .
pip install opencv-python pycocotools matplotlib onnxruntime onnx
```

下载权重文件到当前文件夹目录,链接为：

```
链接: https://pan.baidu.com/s/1aJeCegG7I8UOaVgL5qczFQ?pwd=fhp9 提取码: fhp9 
```

运行代码，生成每个相机对应的文件夹对应的去畸变图片的掩码，并保存到对应的mask文件夹

```
python scripts/amg.py --checkpoint sam_vit_l_0b3195.pth --model-type vit_l --stability-score-thresh 0.9 --box-nms-thresh 0.4 --stability-score-offset 0.9 --points-per-batch 32
```

如果显存不够，就调小 --points-per-batch的值

### 2.4 打包数据

回到camera_to_lidar/data文件夹，运行程序获得每个相机对应mannua-calib和auto-calib文件夹，为后面标定作准备

```
python organize_files.py 
```

最终获得的目录结构如下

```
data
├── pinhole-front
├── pinhole-back
├── fisheye-left
├── fisheye-right
├── fisheye-front
	├──pointclouds
	├──undistorted
	├──masks
	├──images
	├──auto-calib
		├──0010
			├──masks
			├──0010.pcd
			├──0010.png
			├──calib.txt
		├──0015
		├──0030
	├──mannual-calib
		├──0010
			├──0010.pcd
			├──0010.png
			├──center_camera-intrinsic.json
			├──top_center_lidar-to-center_camera-extrinsic.json

```


## 3标定

运行下列程序，针对不同的相机把上面准备好的auto-calib和mannual-calib数据分别送到自动标定和手动标定文件夹

```
 python transfer-files.py --sort pinhole-front
```

--sort 为要进行标定的相机名称

### 3.1手动标定

进入camera_to_lidar/lidar2camera/manual_calib文件夹，如果需要重新编译

```
cd build
rm -rf ./*
cmake ..
make
```

如果在本地编译失败，可以尝试以下命令

```
# 拉取镜像
docker pull xiaokyan/opencalib:v1

#创立容器编译
docker run -it \
--gpus all \
--env="DISPLAY=$DISPLAY" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="./:/workspace:rw" \
xiaokyan/opencalib:v1
```

运行程序开始手工标定

```
chmod +x manual-calib.sh #第一次使用才用输
./manual-calib.sh 97
```

命令中的数字代表第几帧，实际运行时全部替换为实际选中的帧数，运行完成后camera_to_lidar/lidar2camera/manual_calib/calibration_0.txt中的矩阵就是粗标定好的激光雷达到相机的矩阵，运行下面的程序把它更新到所有数据的初始外参程序中

```
python update.py
```

### 3.2 自动标定

进入camera_to_lidar/lidar2camera/auto_calib文件夹，

如果需要编译，操作如上

运行程序自动标定

```
chmod +x auto-calib.sh #第一次使用才用输
bash auto-calib.sh
```

查看效果，第一次标可能效果不好，可以重复手动标以及自动标多次，直到满意为止，标完后也可以在后续标注平台根据实际3d标注框的偏差回到这里来改标定参数继续改进

## 4数据保存

标好后把camera_to_lidar/lidar2camera/manual_calib/data下任意一帧文件夹的top_center_lidar-to-center_camera-extrinsic.json中的data矩阵复制到根目录下 output/lidar2camera.json中，替换对应相机的外参矩阵

在五个雷达到相机的外参矩阵计算好后，在output下依次运行下面的程序

```
python convert2lidar.py
python convert2m32.py
```

就得到了五个相机到主激光雷达的外参矩阵

运行下列代码就获得了激光雷达和相机到车中心的外参矩阵

```
python final-extrinsic.py
```





