#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <pcl/io/pcd_io.h> //PCL��PCD��ʽ�ļ����������ͷ�ļ�
#include <pcl/common/io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h> //PCL�Ը��ָ�ʽ�ĵ��֧��ͷ�ļ�
#include <pcl/visualization/cloud_viewer.h>//���Ʋ鿴����ͷ�ļ�
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

	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");//ֱ�Ӵ���һ����ʾ����
	viewer.showCloud(io_obj.points_xyzi);
	//viewer.showCloud(io_obj.points_xyzrgb);

	while (!viewer.wasStopped())
	{
	}
	
	return 0;
}

