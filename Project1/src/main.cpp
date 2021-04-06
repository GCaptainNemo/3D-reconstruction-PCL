#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/common/float_image_utils.h>

#include "../include/kitti_io_obj.h"
#include "../include/pc_operator.h"
#include "../include/lvx_io_obj.h"
#include <iostream>
#include <string>

using namespace pcl;
#define PI 3.1415926535



int main()
{
	//dealwith_kitti(false, "pc");
	//const char * filedir = "../resources/livox_hikvision/test.lvx";
	//LvxObj::lvx2pcd(filedir, "topcd");
	
	//bool show = true;
	//LvxObj::openPCDfile("./output/test.pcd", show);
	bool save = false;
	bool preprocess = false;
	dealwith_lvx(preprocess, "mc", save);
	return 0;
}

