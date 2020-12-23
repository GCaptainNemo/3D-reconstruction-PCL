#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/common/io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
#include <pcl/visualization/cloud_viewer.h>//点云查看窗口头文件
#include "io_obj.h"

#include <iostream>
#include <string>
using namespace pcl;


	


int main()
{
	ioOBJ io_obj;
	
	io_obj.read_image("../resources/image/000006.png");
	io_obj.read_calib("../resources/calib/000006.txt");
	io_obj.read_bin_xyzi("../resources/Lidar/000006.bin", true);
	//io_obj.read_bin_xyzrgb("../resources/Lidar/000006.bin");

	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");//直接创造一个显示窗口
	viewer.showCloud(io_obj.points_xyzi);
	//viewer.showCloud(io_obj.points_xyzrgb);

	while (!viewer.wasStopped())
	{
	}
	
	return 0;
}

