#pragma once

#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> 


typedef pcl::PointXYZ Point;

class PcOperator
{
public:

	// pointcloud preprocessing
	
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

	static void normal_oriented(const pcl::PointXYZ &viewport, const pcl::PointCloud<pcl::PointXYZ>::Ptr pos, pcl::PointCloud<pcl::Normal>::Ptr normal);

	// surface reconstruction algorithm
	static void triangular(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgb_cloud_with_normals, 
		pcl::PolygonMesh &triangles);
	
	static void triangular(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals, 
		pcl::PolygonMesh &triangles);
	
	/*!
	* \brief decimateMesh  Performs post-processing on the form of quadric decimation to generate a mesh
	*                      that has a higher density in areas with a lot of structure.
	* \param reduction_factor   Reduce vertices proportion    
	* \param mesh_         Origin mesh
	*/
	static void decimateMesh(const float &reduction_factor, pcl::PolygonMeshPtr mesh);

	static void poisson_reconstruction(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgb_cloud_with_normals, 
		pcl::PolygonMeshPtr mesh);

	static void pc2range_image(pcl::RangeImage& range_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_xyzrgb);

	static void range_image_reconstruct(pcl::PolygonMesh &triangles, boost::shared_ptr<pcl::RangeImage> range_image_ptr);

	static void bspline_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud);
	

};

void visualizeCurve(ON_NurbsCurve &curve, ON_NurbsSurface &surface, pcl::visualization::PCLVisualizer &viewer);
void PointCloud2Vector3d(pcl::PointCloud<Point>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data);









