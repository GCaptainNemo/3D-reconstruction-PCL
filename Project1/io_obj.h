//#pragma once
//#ifndef _IO_OBJ_H_
//#define _IO_OBJ_H_
#define _CRT_SECURE_NO_WARNINGS
#include <string>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���
#include <pcl/io/ply_io.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> 


class ioOBJ 
{
public:
	ioOBJ();
	~ioOBJ();
public:
	void bin2pcd(const char *filenameInput, const char *filenameOutput);   
	void bin2pcd2(const std::string & infile, const std::string & outfile);
	void read_bin_xyzi(const std::string & filename, bool crop);
	void read_bin_xyzrgb(const std::string &filename);
	void read_calib(const char *filename);
	void read_pointcloud(const char *filename);
	void read_image(const char *filename);


public:
	pcl::PointCloud<pcl::PointXYZI>::Ptr points_xyzi;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_xyzrgb;


public:
	cv::Mat transform_matrix;
	cv::Mat image;

};


//#endif // _IO_OBJ_H_




