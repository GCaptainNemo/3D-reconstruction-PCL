#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> 

void dealwith_lvx(const char * filedir, const char * option);
void openPCDfile(const char * file_dir, const bool &show);

class LvxObj 
{
public:
	void read_pcd_xyz(const char *filename, const bool &iscrop);
	void project_get_rgb();
	void read_calib(const char *filename);
	void read_pointcloud(const char *filename);
	void read_image(const char *filename);

public:
	cv::Mat transform_matrix;
	cv::Mat image;
};


