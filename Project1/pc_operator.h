#pragma once

#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

// typedef pcl::PointXYZ PointT;

// 1. 下采样和滤波  2. 重采样平滑 3. 法线计算 4.将点云坐标、颜色、发现信息合在一起 5. 贪心投影网格化

//[1]计算点云法向, 并将法向量指向内部
//[2]将点云法向信息叠加在原点云上，生成pcl::PointXYZRGBNormal格式的点云
//[3]使用泊松重建（poisson reconstruction）建立无颜色mesh。
//[4]使用kdtree将原点云的信息映射在无颜色mesh上，并生成彩色mesh。

typedef pcl::PointXYZ Point;

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

	// surface reconstruction algorithm
	static void triangular(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgb_cloud_with_normals, 
		pcl::PolygonMesh &triangles);
	
	static void triangular(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals, 
		pcl::PolygonMesh &triangles);
	
	static void poisson_reconstruction(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgb_cloud_with_normals, 
		pcl::PolygonMesh &mesh);

	static void color_mesh(pcl::PolygonMesh mesh, pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud);

	static void pc2range_image(pcl::RangeImage& range_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_xyzrgb);

	static void range_image_reconstruct(pcl::PolygonMesh &triangles, boost::shared_ptr<pcl::RangeImage> range_image_ptr);

	static void bspline_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud);

	/*static void visualizeCurve(ON_NurbsCurve &curve, ON_NurbsSurface &surface, pcl::visualization::PCLVisualizer &viewer);
	
	static void PointCloud2Vector3d(pcl::PointCloud<Point>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data);
*/
	
};

void visualizeCurve(ON_NurbsCurve &curve, ON_NurbsSurface &surface, pcl::visualization::PCLVisualizer &viewer);
void PointCloud2Vector3d(pcl::PointCloud<Point>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data);






