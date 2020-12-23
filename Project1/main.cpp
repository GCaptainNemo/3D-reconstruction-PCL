#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <pcl/io/pcd_io.h> //PCL��PCD��ʽ�ļ����������ͷ�ļ�
#include <pcl/common/io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h> //PCL�Ը��ָ�ʽ�ĵ��֧��ͷ�ļ�
#include <pcl/visualization/cloud_viewer.h>//���Ʋ鿴����ͷ�ļ�
#include "io_obj.h"
#include "pc_operator.h"
#include <iostream>
#include <string>
using namespace pcl;



int main()
{
	ioOBJ io_obj;
	io_obj.read_image("../resources/image/000000.png");
	io_obj.read_calib("../resources/calib/000000.txt");
	//io_obj.read_bin_xyzi("../resources/Lidar/000006.bin", true);
	io_obj.read_bin_xyz("../resources/Lidar/000000.bin", true);

	// pcl::PointCloud<pcl::PointXYZ>::Ptr filter_points_xyz(new pcl::PointCloud<pcl::PointXYZ>());
	std::cout << "before filter size = " << io_obj.points_xyz->size() << std::endl;

	pc_operator::statistical_filter(io_obj.points_xyz, io_obj.points_xyz, 5, 1.0);
	std::cout << "after filter size = " << io_obj.points_xyz->size() << "\n";
	
	io_obj.project_get_rgb();
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");//ֱ�Ӵ���һ����ʾ����
	//viewer.showCloud(io_obj.points_xyzi);
	//viewer.showCloud(io_obj.points_xyzrgb);
	viewer.showCloud(io_obj.points_xyzrgb);
	while (!viewer.wasStopped())
	{
	}
	return 0;
}

