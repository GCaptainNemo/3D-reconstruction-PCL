#pragma once


#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

// typedef pcl::PointXYZ PointT;

// 1. 下采样和滤波  2. 重采样平滑 3. 法线计算 4.将点云坐标、颜色、发现信息合在一起 5. 贪心投影网格化

//[1]计算点云法向, 并将法向量指向内部
//[2]将点云法向信息叠加在原点云上，生成pcl::PointXYZRGBNormal格式的点云
//[3]使用泊松重建（poisson reconstruction）建立无颜色mesh。
//[4]使用kdtree将原点云的信息映射在无颜色mesh上，并生成彩色mesh。

class pc_operator
{
public:
	
	static void down_sample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled, const float & voxel_size);

	static void statistical_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, const int & neighbour, const float & proportion);
	
	static void resampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed, const float &searchRadius);

	static void upsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_upsampled);
	
	static void random_sampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_randomsampled, const int & number);


	static void estimate_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		pcl::PointCloud<pcl::Normal>::Ptr normals, const int &nPoints);

	static void triangular(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgb_cloud_with_normals, 
		pcl::PolygonMesh &triangles);
	
	static void triangular(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals, 
		pcl::PolygonMesh &triangles);
	
	static void poisson_reconstruction(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgb_cloud_with_normals, 
		pcl::PolygonMesh &mesh);

	static void color_mesh(pcl::PolygonMesh mesh, pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud);



	/*pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_downSampled(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_smoothed(new pcl::PointCloud<PointT>);*/
	
	
	
};





