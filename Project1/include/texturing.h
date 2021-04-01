#pragma once

// STL
#include <iostream>
#include <fstream>

// PCL
#include <pcl/surface/texture_mapping.h>
#include <pcl/point_types.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> 


class texturing
{
public:
	
	texturing();
	~texturing();
	
	/*!
	 * \brief loadMesh          Convert mesh to texture_mesh.
	 */
	void loadMesh(pcl::PolygonMeshPtr mesh);

	/*!
	 * \brief loadCamera       Loads camera with corresponding image.
	 */
	void loadCamera();

	/*!
	 * \brief color_mesh       Give each polygon's vertex color according to KNN PointXYZRGB
	 */
	static void color_mesh(pcl::PolygonMesh &mesh, pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud);


	static void texture_mesh(pcl::PolygonMesh &mesh, pcl::TextureMeshPtr texturemesh, cv::Mat &transform_matrix, cv::Mat &image);

	
public:
	pcl::TextureMesh::Ptr texture_mesh_;
	pcl::texture_mapping::CameraVector cameras_;    /**< The vector containing all cameras. */

};