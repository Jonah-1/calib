## 0 环境配置

进入calibration/camera_to_lidar文件夹

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

首先分别使用前后左右的各相机（前面有两个）同时与各自最近的前后左右激光雷达同时采集点云图片，保存为对应的四个bag包（camera-front.bag，camera-back.bag，camera-left.bag, camera-right.bag）

## 2 数据处理

### 2.1 数据读取

进入camera_to_lidar/data文件夹，运行程序，读取每个bag包里的点云和相机图片数据，并会保存到各对应名字的文件夹中（fisheye-front，fisheye-left，fisheye-right, pinhole-back, pinhole-front）

```
python save_sync.py
```

### 2.2 选图片生成掩码

运行程序对每个鱼眼相机和针孔相机的图片随机选择，并保存到对应文件夹的undistorted文件夹中

```
python select-images.py
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

运行代码，生成每个相机对应的文件夹对应的去畸变图片的掩码，并保存到对应的mask文件夹

```
python scripts/amg.py --checkpoint sam_vit_l_0b3195.pth --model-type vit_l --stability-score-thresh 0.9 --box-nms-thresh 0.4 --stability-score-offset 0.9 --points-per-batch 32

conda activate calib  #回到原环境
```

如果显存不够，就调小 --points-per-batch的值

### 2.4 打包数据

回到camera_to_lidar/data文件夹，运行程序获得每个相机对应mannua-calib和auto-calib文件夹，为后面标定作准备

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

每标一个相机和激光雷达，便将camera_to_lidar/data对应相机文件夹下的mannua-calib和auto-calib文件夹中的文件保存到对应标定程序中，首先以fisheye-front举例.

将camera_to_lidar/data/fisheye-front/mannua-calib下的所有子文件夹保存到camera_to_lidar/lidar2camera/manual_calib/data下；同时将camera_to_lidar/data/fisheye-front/auto-calib下的所有子文件夹保存到camera_to_lidar/lidar2camera/auto_calib/data下，下面进行标定

### 3.1手动标定

进入camera_to_lidar/lidar2camera/manual_calib文件夹，如果需要重新编译且在本地编译失败，可以尝试以下命令

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
./manual-calib.sh 190
```

命令中的数字代表第几帧，实际运行时全部替换为实际选中的帧数，运行完成后camera_to_lidar/lidar2camera/manual_calib/calibration_0.txt中的矩阵就是粗标定好的激光雷达到相机的矩阵，运行下面的程序把它更新到所有数据的初始外参程序中

```
python update.py
```

### 3.2 自动标定

进入camera_to_lidar/lidar2camera/auto_calib文件夹，

如果需要重新编译且在本地编译失败，可以尝试以下命令

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

运行程序自动标定

```
chmod +x auto-calib.sh #第一次使用才用输
bash auto-calib.sh
```

查看效果，第一次标可能效果不好，可以重复手动标以及自动标多次，直到满意为止，标完后也可以在后续标注平台根据实际3d标注框的偏差回到这里来改标定参数继续改进

## 4数据保存

标好后把camera_to_lidar/lidar2camera/auto_calib/extrinsic.txt中的Extrinsic矩阵复制到camera_to_lidar/output/lidar2camera.json中，替换对应相机的外参矩阵



