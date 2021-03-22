#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>//点云查看窗口头文件
#include "kitti_io_obj.h"
#include "pc_operator.h"
#include "lvx_io_obj.h"
#include <iostream>
#include <string>

using namespace pcl;

void dealwith_kitti(const bool &preprocess, const std::string & option) 
{
	ioOBJ io_obj;
	io_obj.read_image("../resources/image/000000.png");
	io_obj.read_calib("../resources/calib/000000.txt");
	io_obj.read_bin_xyz("../resources/Lidar/000000.bin", true);
	std::cout << "before filter size = " << io_obj.points_xyz->size() << std::endl;

	// 点云预处理
	if (preprocess) {
		pc_operator::down_sample(io_obj.points_xyz, io_obj.points_xyz, 0.01);
		pc_operator::statistical_filter(io_obj.points_xyz, io_obj.points_xyz, 50, 3.0);
		// pcl::io::savePCDFile("/pcd", *cloud);
		pc_operator::resampling(io_obj.points_xyz, io_obj.points_xyz, 0.05); // 平滑
		pc_operator::upsampling(io_obj.points_xyz, io_obj.points_xyz);
		pc_operator::random_sampling(io_obj.points_xyz, io_obj.points_xyz, 60000);
	}
	
	// 直接显示真彩色点云(不网格化)
	io_obj.project_get_rgb();
	if(option == "pc")
	{
		pcl::visualization::CloudViewer viewer("Simple Cloud Viewer"); //创造一个显示窗口
		//viewer.showCloud(io_obj.points_xyzi);
		viewer.showCloud(io_obj.points_xyzrgb);
		while (!viewer.wasStopped())
		{
		}
	}
	else{
		// 点云网格化(贪婪投影三角形 和 poisson重建都需要点云 + 点云法向量)
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pc_operator::estimate_normal(io_obj.points_xyz, normals, 10); // estimate point cloud normal vector
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgbcloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::concatenateFields(*(io_obj.points_xyzrgb), *normals, *rgbcloud_with_normals);
		pcl::PolygonMesh mesh;
		if (option == "poisson") {
			pc_operator::poisson_reconstruction(rgbcloud_with_normals, mesh);  // poisson reconstruction
		}
		else if (option == "greedy") {
			pc_operator::triangular(rgbcloud_with_normals, mesh);  // greedy projection triangulation
		}
		pc_operator::color_mesh(mesh, io_obj.points_xyzrgb);
		// 显示网格化结果
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("mesh"));
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPolygonMesh(mesh, "mesh");
		viewer->initCameraParameters();
		viewer->addCoordinateSystem();
		while (!viewer->wasStopped()) {
			viewer->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		std::cout << "success" << std::endl;
	}
	//pcl::io::savePLYFileBinary("D:/pg_cpp/3D-reconstruction-PCL/result/mesh.ply", triangles);
}
void dealwith_lvx(const bool &preprocess, const char * option)
{
	LvxObj lvx_obj;
	lvx_obj.read_image("../resources/livox_hikvision/test.png");
	lvx_obj.set_calib();
	lvx_obj.read_pcd_xyz("./output/test.pcd", true);
	std::cout << "before filter size = " << lvx_obj.points_xyz->size() << std::endl;

	// 点云预处理
	if (preprocess) {
		pc_operator::down_sample(lvx_obj.points_xyz, lvx_obj.points_xyz, 0.01);
		pc_operator::statistical_filter(lvx_obj.points_xyz, lvx_obj.points_xyz, 50, 3.0);
		// pcl::io::savePCDFile("/pcd", *cloud);
		pc_operator::resampling(lvx_obj.points_xyz, lvx_obj.points_xyz, 0.05); // 平滑
		pc_operator::upsampling(lvx_obj.points_xyz, lvx_obj.points_xyz);
		pc_operator::random_sampling(lvx_obj.points_xyz, lvx_obj.points_xyz, 60000);
	}

	// 直接显示真彩色点云(不网格化)
	lvx_obj.project_get_rgb();
	if (strcmp(option, "pc")==0)
	{
		pcl::visualization::CloudViewer viewer("Simple Cloud Viewer"); //创造一个显示窗口
		//viewer.showCloud(io_obj.points_xyzi);
		viewer.showCloud(lvx_obj.points_xyzrgb);
		while (!viewer.wasStopped())
		{
		}
	}
	else {
		// 点云网格化(贪婪投影三角形 和 poisson重建都需要点云 + 点云法向量)
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pc_operator::estimate_normal(lvx_obj.points_xyz, normals, 10); // estimate point cloud normal vector
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgbcloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::concatenateFields(*(lvx_obj.points_xyzrgb), *normals, *rgbcloud_with_normals);
		pcl::PolygonMesh mesh;
		if (strcmp(option, "poisson") == 0) {
			pc_operator::poisson_reconstruction(rgbcloud_with_normals, mesh);  // poisson reconstruction
		}
		else if (strcmp(option, "greedy") == 0) {
			pc_operator::triangular(rgbcloud_with_normals, mesh);  // greedy projection triangulation
		}
		pc_operator::color_mesh(mesh, lvx_obj.points_xyzrgb);
		// 显示网格化结果
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("mesh"));
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPolygonMesh(mesh, "mesh");
		viewer->initCameraParameters();
		viewer->addCoordinateSystem();
		while (!viewer->wasStopped()) {
			viewer->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		std::cout << "success" << std::endl;
	}
	//pcl::io::savePLYFileBinary("D:/pg_cpp/3D-reconstruction-PCL/result/mesh.ply", triangles);
}

int main()
{
	// dealwith_kitti(false, "pc");
	//const char * filedir = "../resources/livox_hikvision/test.lvx";
	//LvxObj::lvx2pcd(filedir, "topcd");
	
	//bool show = true;
	//LvxObj::openPCDfile("./output/test.pcd", show);
	dealwith_lvx(false, "pc");

	return 0;
}

