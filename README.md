# 3D-reconstruction-PCL
1. 利用C++ PCL库对KITTI数据集进行三维重建
2. 利用livox激光雷达和hikvision相机进行三维重建

## 一、环境：
VS2017 + PCL1.8.1

## 二、实验步骤：
### 2.1 KITTI三维重建
1. 用KITTI的配准信息将点云投影到图像上获得真彩色点云
2. 滤波  
3. 重采样平滑 
4. 法线计算 
5. 将点云坐标、颜色、法线信息合在一起 
6. 网格化（贪心三角化）

### 2.2 Livox-hikvision三维重建
1. 读取.lvx文件，转换成pcd文件
2. 读取hikvision拍摄视频
3. 后续方法同2.1


## 三、结果:
### 1. 真彩色点云(KITTI)

![image](./result/rgb_pc.png)

### 2. 贪婪投影三角化(KITTI)

![image](./result/greedy_tri.png)

## 四、参考
1. 解析.lvx文件的代码参考[pylvx](https://github.com/Jaesirky/pylvx)
