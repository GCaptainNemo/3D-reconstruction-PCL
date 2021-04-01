#pragma once
#include <pcl/surface/texture_mapping.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> 


class texturing
{
public:

	/*!
	 * \brief color_mesh       Give each polygon's vertex color according to KNN PointXYZRGB
	 */
	static void color_mesh(pcl::PolygonMesh &mesh, pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud);


	static void texture_mesh(pcl::PolygonMesh &mesh, pcl::TextureMeshPtr texturemesh, cv::Mat &transform_matrix, cv::Mat &image);

	/*!
	 * \brief loadCamera       Loads camera with corresponding image.
	 */
	static void loadCamera();
};