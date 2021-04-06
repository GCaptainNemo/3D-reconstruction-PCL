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


/*!
 * \brief The Coords struct     Coordinate class used in recursiveFindCoordinates for OdmTexturing::sortPatches().
 */
struct Coords
{
	// Coordinates for row and column
	float r_, c_;

	// If coordinates have been placed
	bool success_;

	Coords()
	{
		r_ = 0.0;
		c_ = 0.0;
		success_ = false;
	}
};


/*!
 * \brief The Patch struct      Struct to hold all faces connected and with the same optimal camera.
 */
struct Patch
{
	std::vector<size_t> faces_;
	float minu_, minv_, maxu_, maxv_;
	Coords c_;
	bool placed_;
	int materialIndex_;
	int optimalCameraIndex_;

	Patch()
	{
		placed_ = false;
		faces_ = std::vector<size_t>(0);
		minu_ = std::numeric_limits<double>::infinity();
		minv_ = std::numeric_limits<double>::infinity();
		maxu_ = 0.0;
		maxv_ = 0.0;
		optimalCameraIndex_ = -1;
		materialIndex_ = 0;
	}
};

/*!
 * \brief The Node struct       Node class for acceleration structure in OdmTexturing::sortPatches().
 */
struct Node
{
	float r_, c_, width_, height_;
	bool used_;
	Node* rgt_;
	Node* lft_;

	Node()
	{
		r_ = 0.0;
		c_ = 0.0;
		width_ = 1.0;
		height_ = 1.0;
		used_ = false;
		rgt_ = NULL;
		lft_ = NULL;
	}

	Node(const Node &n)
	{
		r_ = n.r_;
		c_ = n.c_;
		used_ = n.used_;
		width_ = n.width_;
		height_ = n.height_;
		rgt_ = n.rgt_;
		lft_ = n.lft_;
	}
};


/*!
 * \brief   The Texturing class is used to create textures to a welded ply-mesh using the camera
 *          positions from pmvs as input. The result is stored in an obj-file with corresponding
 *          mtl-file and the textures saved as jpg.
 */
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
	 * \brief calculate_patches      Arrange faces into patches as a prestep to arranging UV-mapping.
	 */
	void calculate_patches();

	/*!
	 * \brief sortPatches       Sorts patches into UV-containers to be used in createTextures() using a rectangle packer approach.
	 */
	void sortPatches();

	/*!
	 * \brief recursiveFindCoords   Recursive function used in sortPatches() to find free area to place patch.
	 * \param n                     The container in which to check for free space in.
	 * \param w                     The width of the box to place.
	 * \param h                     The height of the box to place.
	 * \return                      The coordinates where the patch has been placed.
	 */
	Coords recursiveFindCoords(Node &n, float w, float h);

	/*!
	 * \brief createTextures    Creates textures to the mesh.
	 */
	void createTextures();


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
	static bool is_face_projected(const pcl::TextureMapping<pcl::PointXYZ>::Camera &camera, const pcl::PointXYZ &p1, const pcl::PointXYZ &p2, const pcl::PointXYZ &p3, 
		pcl::PointXY &proj1, pcl::PointXY &proj2, pcl::PointXY &proj3);

	/*!
	 * \brief get_pixel_coordinates     Calculate UV coordinates.
	 * \param camera                  Camera Pose
	 * \return  
	 */
	static bool get_pixel_coordinates(const pcl::PointXYZ &pt, const pcl::TextureMapping<pcl::PointXYZ>::Camera &cam, pcl::PointXY &UV_coordinates);


	/*!
	 * \brief get_triangle_centroid     Calculate circumscribed circle centroid and radius.
	 */
	static void get_triangle_centroid(const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3, pcl::PointXY &circumcenter, double &radius);

	static bool check_point_in_triangle(const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3, const pcl::PointXY &pt);

	
public:
	pcl::TextureMesh::Ptr texture_mesh_;            /**< Texture mesh to deal with. */
	pcl::texture_mapping::CameraVector cameras_;    /**< The vector containing all cameras. */
	std::vector<int> tTIA_;                         /**< The vector containing the optimal cameras for all faces. */
	std::vector<Patch> patches_;    /**< The vector containing all patches */
	double textureResolution_;      /**< The resolution of each texture. */
	double padding_;                /**< A padding used to handle edge cases. */
	int nrTextures_;             /**< The number of textures created. */

};



