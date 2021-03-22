//#pragma once
//#ifndef _IO_OBJ_H_
//#define _IO_OBJ_H_
#define _CRT_SECURE_NO_WARNINGS
#include <string>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include <pcl/io/ply_io.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> 


class KittiObj 
{
public:
	KittiObj();
	~KittiObj();
public:
	void bin2pcd(const char *filenameInput, const char *filenameOutput);   
	void bin2pcd2(const std::string & infile, const std::string & outfile);
	void read_bin_xyzi(const std::string & filename, const bool &iscrop);
	void read_bin_xyz(const char *filename, const bool &iscrop);
	void project_get_rgb();
	void read_calib(const char *filename);
	void read_pointcloud(const char *filename);
	void read_image(const char *filename);


public:
	pcl::PointCloud<pcl::PointXYZI>::Ptr points_xyzi;
	pcl::PointCloud<pcl::PointXYZ>::Ptr points_xyz;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_xyzrgb;



public:
	cv::Mat transform_matrix;
	cv::Mat image;

};


//#endif // _IO_OBJ_H_




