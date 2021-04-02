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


class Texturing
{
public:
	
	Texturing();
	~Texturing();
	
	/*!
	 * \brief load_mesh               Convert mesh to texture_mesh.
	 * \param mesh                    Origin mesh.
	 */
	void load_mesh(pcl::PolygonMeshPtr mesh);

	/*!
	 * \brief load_camera             Loads camera with corresponding image.
	 */
	void load_camera();


	/*!
	 * \brief mesh_image_match        Assigns optimal camera to mesh. 
	 */
	void mesh_image_match();

	/*!
	 * \brief color_mesh              Give each polygon's vertex color according to KNN PointXYZRGB
	 */
	static void color_mesh(pcl::PolygonMesh &mesh, pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud);


	static void texture_mesh(pcl::PolygonMesh &mesh, pcl::TextureMeshPtr texturemesh, cv::Mat &transform_matrix, cv::Mat &image);

	/*!
	 * \brief is_face_projected         Calculate whether a face could project to photo and photo coordinates.
	 * \param camera                  Camera Pose
	 * \param p1, p2, p3              Triangular face's vertices.
	 * \param proj1, proj2, proj3     Image coordinates.
	 */
	static bool Texturing::is_face_projected(const pcl::TextureMapping<pcl::PointXYZ>::Camera &camera, const pcl::PointXYZ &p1, const pcl::PointXYZ &p2, const pcl::PointXYZ &p3, 
		pcl::PointXY &proj1, pcl::PointXY &proj2, pcl::PointXY &proj3);

	/*!
	 * \brief getPixelCoordinates     Calculate UV coordinates.
	 * \param camera                  Camera Pose
	 * \return  
	 */
	static bool getPixelCoordinates(const pcl::PointXYZ &pt, const pcl::TextureMapping<pcl::PointXYZ>::Camera &cam, pcl::PointXY &UV_coordinates);


	/*!
	 * \brief get_triangle_centroid     Calculate circumscribed circle centroid and radius.
	 */
	static void Texturing::get_triangle_centroid(const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3, pcl::PointXY &circumcenter, double &radius);

	
public:
	pcl::TextureMesh::Ptr texture_mesh_;            /**< Texture mesh to deal with. */
	pcl::texture_mapping::CameraVector cameras_;    /**< The vector containing all cameras. */
	std::vector<int> tTIA_;                         /**< The vector containing the optimal cameras for all faces. */


};