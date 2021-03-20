# 3D-reconstruction-PCL
利用C++ PCL库对KITTI数据集进行三维重建

# 一、环境：
VS2017 + PCL1.8.1

# 二、实验步骤：
1. 用KITTI的配准信息将点云投影到图像上获得真彩色点云
2. 滤波  
3. 重采样平滑 
4. 法线计算 
5. 将点云坐标、颜色、法线信息合在一起 
6. 网格化（贪心三角化）


# 三、结果:
## 1. 真彩色点云
![image](./result/rgb_pc.png)
## 2. 贪婪投影三角化
![image](./result/greedy_tri.png)