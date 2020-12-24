#pragma once


#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

// typedef pcl::PointXYZ PointT;

// 1. �²������˲�  2. �ز���ƽ�� 3. ���߼��� 4.���������ꡢ��ɫ��������Ϣ����һ�� 5. ̰��ͶӰ����

//[1]������Ʒ���, ����������ָ���ڲ�
//[2]�����Ʒ�����Ϣ������ԭ�����ϣ�����pcl::PointXYZRGBNormal��ʽ�ĵ���
//[3]ʹ�ò����ؽ���poisson reconstruction����������ɫmesh��
//[4]ʹ��kdtree��ԭ���Ƶ���Ϣӳ��������ɫmesh�ϣ������ɲ�ɫmesh��

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





