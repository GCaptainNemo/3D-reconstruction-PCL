#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

class LvxObj 
{
public:
	LvxObj();
	~LvxObj();

	/*!
	* \brief set_calib  Loads calibration parameters according to calib files.
	*/
	void set_calib();

	/*!
	* \brief read_image		  Loads image.
	* \param filename         File name.
	* \param correct          Image correct or not.
	*/
	void read_image(const char *filename, const bool & correct);
	
	/*!
	 * \brief read_pcd_xyz    Read point cloud pcd file.
	 * \param filename        File name.
	 * \param iscrop          Argument crop or not.
	 */
	void read_pcd_xyz(const char *filename, const bool &iscrop);
	
	/*!
	 * \brief read_pcds_xyz   Read point cloud pcd file from a dir.
	 * \param dir             Folder name.
	 * \param frame_num       Number of pcd file load in.
	 */
	void read_pcds_xyz(const std::string &dir, const bool &iscrop, const int & frame_num);
	
	/*
	brief project_get_rgb     Project pointcloud to image to get color.
	*/
	void project_get_rgb();

	/*
	brief lvx2pcd             Convert lvx file to pcd file according to frame.
	*/
	static void lvx2pcd(const char * filedir, const char * option);
	
	/*
	brief openPCDfile         Open .pcd file to test.
	*/
	static void openPCDfile(const char * file_dir, const bool &show);

public:
	cv::Mat transform_matrix;    /**< from globol to image coordinate */
	cv::Mat intrinsic_matrix;    /**< from local to image coordinate  */
	cv::Mat extrinsic_matrix;    /**< from global to local coordinate */
	cv::Mat dist_matrix;         /**< dist parameters  */
	cv::Mat image;               /**< texture image    */


public:
	pcl::PointCloud<pcl::PointXYZI>::Ptr points_xyzi;         /**< points xyz and intensity   */
	pcl::PointCloud<pcl::PointXYZ>::Ptr points_xyz;           /**< points xyz  */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_xyzrgb;     /**< points xyz and rgb*/

};
void dealwith_lvx(const bool &preprocess, const char * option, const bool & save);


