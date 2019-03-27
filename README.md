# VO (Visual Odometry)

#### 项目概述
+ 在已有先验点云和一组相机图像的情况下做一个比较精确的视觉里程计，使用的先验信息包括一个精确的点云图，一个精确的初始位姿。
+ dataswt: EuRoc V1_02_medium cam0;
+ 特点，可以得到非常准确的相机位姿（在点云地图下
+ 初始帧需要自己获得位姿。


#### 依赖

+ OpenCV
+ libpcl
+ Eigen
+ cmake
+ g2o

#### 构建

1. mkdir build
2. cd build
3. make -j4
4. ./main
