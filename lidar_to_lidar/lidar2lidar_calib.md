



## 0 环境配置

**1** 安装ros，可按照这个网址安装https://blog.csdn.net/m0_73745340/article/details/135281023?spm=1001.2014.3001.5506，目前使用的是ubuntu20.04版本对应的noetic版本

**2** 安装cloudcompare，可参考https://blog.csdn.net/weixin_43821819/article/details/130744779

**3** 构建虚拟环境

进入calibration/lidar_to_lidar文件夹

```
conda create -n calib python=3.10
conda activate calib

pip install -r requirement.txt
```



## 1 数据采集

分别使用右前和左后的激光雷达与前激光雷达同时采集点云，保存为对应的四个2包，注意采集过程要保证两个激光雷达都能采集到地面（lidar-front.bag，lidar-back.bag）

使用rosbag转换，从ros2转到ros1

```
rosbags-convert --src front_middle --dst camera-lidar.bag
```



## 2 数据处理

### 2.1数据读取

运行save_pcd.py，读取每个bag包里的点云数据，并会保存到storaged-data文件夹下对应名字的文件夹中，目录结构如下

```
storaged-data
├── back
	├──data
		├──source.pcd
		├──target.pcd
	├──data1
├── front
```

其中，back，front代表是哪个激光雷达与顶部激光雷达一起录的，data文件夹代表未经处理的原始点云数据，data1代表经过cloudcompare软件裁剪后的点云数据，具体裁剪流程如下



### 2.2点云裁剪

打开cloudcompare，依次将storaged-data文件夹下2个子文件夹的data文件夹下source.pcd和target.pcd导入cloudcompare，进行点云裁剪，并把裁剪好的点云保存到对应激光雷达文件夹的data1子文件夹中

****

## 3 标定

### 3.1粗标定

首先把storaged-data下back文件夹的data和data1文件夹复制到Coarse Calibration文件夹中替换对应的两个文件，然后进入Coarse Calibration文件夹

提取对应激光雷达到顶部激光雷达的变换矩阵

```
python .\get_tranform_matrix.py
```

把计算得到的结果放到get_total_matrix.py中替换transform1，并运行该程序

```
python get_total_matrix.py
```

应用变换矩阵，获得粗标定矩阵矫正后的点云，保存在data3文件夹中

```
python apply_trasoform.py
```



### 3.2 精标定

进入Refine Calibration文件夹进行编译

prerequistites：

- cmeke

- make

- eigen3

- Pcl1.9

- Pangolin

  其中,Pangolin需要特定版本，详细见https://blog.csdn.net/hooksten/article/details/138043687?spm=1001.2014.3001.5506

```
mkdir -p build && cd build
cmake .. && make
```

把Coarse Calibration/data3文件夹下的所有pcd文件复制到Refine Calibration/data中然后将其放入总的data目录中，进行精细标定

```
./bin/run_lidar2lidar data/lidar_cloud_path.txt data/initial_extrinsic.txt
```

执行成功后会获得calibration_results.txt,将其中第一个矩阵复制到Coarse Calibration/get_total_matrix.py中，替换transform_source2，并在Coarse Calibration文件夹下运行

```
python get_final_matrix.py
```

即可获得最终标定好的对应激光雷达到顶部激光雷达的变换矩阵，将对应矩阵保存到  *lidar2m128 .json*中

然后继续对剩下的front文件夹重复上述操作



