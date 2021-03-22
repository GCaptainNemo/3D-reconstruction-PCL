#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include <pcl/io/ply_io.h>

class LvxObj 
{
public:
	LvxObj();
	~LvxObj();
	void project_get_rgb();
	void set_calib();
	void read_image(const char *filename);
	void read_pcd_xyz(const char *filename, const bool &iscrop);
	static void lvx2pcd(const char * filedir, const char * option);
	static void openPCDfile(const char * file_dir, const bool &show);

public:
	cv::Mat transform_matrix;
	cv::Mat image;

public:
	pcl::PointCloud<pcl::PointXYZI>::Ptr points_xyzi;
	pcl::PointCloud<pcl::PointXYZ>::Ptr points_xyz;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_xyzrgb;

};


