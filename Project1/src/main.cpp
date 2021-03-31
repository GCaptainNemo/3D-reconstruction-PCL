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



void dealwith_kitti(const bool &preprocess, const char * option) 
{
	KittiObj kitti_obj;
	kitti_obj.read_image("../resources/image/000000.png");
	kitti_obj.read_calib("../resources/calib/000000.txt");
	kitti_obj.read_bin_xyz("../resources/Lidar/000000.bin", true);
	std::cout << "before filter size = " << kitti_obj.points_xyz->size() << std::endl;

	// 点云预处理
	if (preprocess) {
		pc_operator::down_sample(kitti_obj.points_xyz, kitti_obj.points_xyz, 0.01);
		pc_operator::statistical_filter(kitti_obj.points_xyz, kitti_obj.points_xyz, 50, 3.0);
		// pcl::io::savePCDFile("/pcd", *cloud);
		pc_operator::resampling(kitti_obj.points_xyz, kitti_obj.points_xyz, 0.05); // 平滑
		pc_operator::upsampling(kitti_obj.points_xyz, kitti_obj.points_xyz);
		pc_operator::random_sampling(kitti_obj.points_xyz, kitti_obj.points_xyz, 60000);
	}
	
	// 直接显示真彩色点云(不网格化)
	kitti_obj.project_get_rgb();
	if(strcmp(option,"pc") == 0)
	{
		pcl::visualization::CloudViewer viewer("Simple Cloud Viewer"); //创造一个显示窗口
		//viewer.showCloud(kitti_obj.points_xyzi);
		viewer.showCloud(kitti_obj.points_xyzrgb);
		while (!viewer.wasStopped())
		{
		}
	}
	else{
		// 点云网格化(贪婪投影三角形 和 poisson重建都需要点云 + 点云法向量)
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pc_operator::estimate_normal(kitti_obj.points_xyz, normals, 10); // estimate point cloud normal vector
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgbcloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::concatenateFields(*(kitti_obj.points_xyzrgb), *normals, *rgbcloud_with_normals);
		pcl::PolygonMesh mesh;
		if (strcmp(option, "poisson") == 0) {
			pc_operator::poisson_reconstruction(rgbcloud_with_normals, mesh);  // poisson reconstruction
			pc_operator::color_mesh(mesh, kitti_obj.points_xyzrgb);
		}
		else if (strcmp(option, "greedy") == 0) {
			pc_operator::triangular(rgbcloud_with_normals, mesh);  // greedy projection triangulation
		}
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
	//pcl::io::savePLYFileBinary("D:/pg_cpp/3D-reconstruction-PCL/result/mesh.ply", mesh);
}

void dealwith_lvx(const bool &preprocess, const char * option, const bool & save)
{
	LvxObj lvx_obj;
	const std::string dir = "../output";
	lvx_obj.set_calib();
	lvx_obj.read_image("../../resources/livox_hikvision/test.png", true);
	lvx_obj.read_pcds_xyz(dir, true, 200);
	// lvx_obj.read_pcd_xyz("./output/test.pcd", true);
	std::cout << "before filter size = " << lvx_obj.points_xyz->size() << std::endl;

	// 点云预处理
	if (preprocess) {
		pc_operator::down_sample(lvx_obj.points_xyz, lvx_obj.points_xyz, 0.01);
		pc_operator::statistical_filter(lvx_obj.points_xyz, lvx_obj.points_xyz, 50, 3.0);
		pc_operator::resampling(lvx_obj.points_xyz, lvx_obj.points_xyz, 0.05); // 平滑
		pc_operator::upsampling(lvx_obj.points_xyz, lvx_obj.points_xyz);
		pc_operator::random_sampling(lvx_obj.points_xyz, lvx_obj.points_xyz, 60000);
	}

	// show colored point cloud(use point-image mapping)
	lvx_obj.project_get_rgb();
	if (strcmp(option, "pc")==0)
	{
		pcl::visualization::CloudViewer viewer("Simple Cloud Viewer"); //创造一个显示窗口
		//viewer.showCloud(kitti_obj.points_xyzi);
		viewer.showCloud(lvx_obj.points_xyzrgb);
		while (!viewer.wasStopped())
		{
		}
		if (save) { pcl::io::savePCDFile("./linshi/color_pc.pcd", *lvx_obj.points_xyzrgb); }
	}
	else if (strcmp(option, "rangeImage") == 0) 
	{
		boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
		pcl::RangeImage& range_image = *range_image_ptr;
		// pc2rangeImage 
		pc_operator::pc2range_image(range_image, lvx_obj.points_xyzrgb);
		pcl::visualization::RangeImageVisualizer range_image_widget("RangeImage");
		range_image_widget.showRangeImage(range_image);
		range_image_widget.setWindowTitle("RangeImage");
		
		//meshing based on range image
		pcl::PolygonMesh mesh;
		pc_operator::range_image_reconstruct(mesh, range_image_ptr);
		pc_operator::color_mesh(mesh, lvx_obj.points_xyzrgb);

		// viewer
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("RangeImage"));
		viewer->setBackgroundColor(0., 0., 0.);
		viewer->addPolygonMesh(mesh, "mesh");
		viewer->addCoordinateSystem();
		while (!range_image_widget.wasStopped() && !viewer->wasStopped())
		{
			range_image_widget.spinOnce();
			Sleep(0.01);
			viewer->spinOnce();
		}
		if (save){pcl::io::savePLYFileBinary("./linshi/mesh_rangeimage.ply", mesh);}
	}
	else if(strcmp(option, "bspline")==0)
	{
		std::cout << "sizexyz  = " << lvx_obj.points_xyz->size();
		pc_operator::bspline_reconstruction(lvx_obj.points_xyz);
	}
	else {
		// 用点云meshing(贪婪投影三角形和poisson重建)
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pc_operator::estimate_normal(lvx_obj.points_xyz, normals, 10); // estimate point cloud normal vector
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgbcloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::concatenateFields(*(lvx_obj.points_xyzrgb), *normals, *rgbcloud_with_normals);
		pcl::PolygonMesh mesh;
		if (strcmp(option, "poisson") == 0) {
			pc_operator::poisson_reconstruction(rgbcloud_with_normals, mesh);  // poisson reconstruction
			pc_operator::color_mesh(mesh, lvx_obj.points_xyzrgb);
			if (save) { pcl::io::savePLYFileBinary("./linshi/poisson_mesh.ply", mesh); }

		}
		else if (strcmp(option, "greedy") == 0) {
			pc_operator::triangular(rgbcloud_with_normals, mesh);  // greedy projection triangulation
			if (save) { pcl::io::savePLYFileBinary("./linshi/greedy_mesh.ply", mesh); }
		}
		// 显示网格化结果
		
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("mesh"));
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPolygonMesh(mesh, "mesh");
		if (strcmp(option, "poisson") == 0) 
		{
			//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(lvx_obj.points_xyzrgb);
			//viewer->addPointCloud(lvx_obj.points_xyzrgb, rgb, "sample cloud");
			pcl::visualization::PointCloudColorHandlerCustom<Point> handler(lvx_obj.points_xyz, 0, 255, 0);
			viewer->addPointCloud<Point>(lvx_obj.points_xyz, handler, "cloud_cylinder");
		}

		viewer->initCameraParameters();
		viewer->addCoordinateSystem();
		while (!viewer->wasStopped()) {
			viewer->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		std::cout << "success" << std::endl;
	}
	
}



int main()
{
	//dealwith_kitti(false, "pc");
	//const char * filedir = "../resources/livox_hikvision/test.lvx";
	//LvxObj::lvx2pcd(filedir, "topcd");
	
	//bool show = true;
	//LvxObj::openPCDfile("./output/test.pcd", show);
	dealwith_lvx(false, "poisson", true);
	return 0;
}

