#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/surface/impl/organized_fast_mesh.hpp> 
#include <pcl/surface/organized_fast_mesh.h>
#include "kitti_io_obj.h"
#include "pc_operator.h"
#include "lvx_io_obj.h"
#include <pcl/io/png_io.h>
#include <iostream>
#include <string>

using namespace pcl;
#define M_PI 3.1415926535

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
		}
		else if (strcmp(option, "greedy") == 0) {
			pc_operator::triangular(rgbcloud_with_normals, mesh);  // greedy projection triangulation
		}
		pc_operator::color_mesh(mesh, kitti_obj.points_xyzrgb);
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
	const std::string dir = "./output";
	lvx_obj.read_image("../resources/livox_hikvision/test.png");
	lvx_obj.set_calib();
	lvx_obj.read_pcds_xyz(dir, true, 200);
	// lvx_obj.read_pcd_xyz("./output/test.pcd", true);
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
		//viewer.showCloud(kitti_obj.points_xyzi);
		viewer.showCloud(lvx_obj.points_xyzrgb);
		while (!viewer.wasStopped())
		{
		}
	}
	else if (strcmp(option, "rangeImage") == 0) 
	{ 
		// 点云生成深度图（球面投影）
		// 用boost是为了显示
		boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
		pcl::RangeImage& range_image = *range_image_ptr;
		
		//noiseLevel设置周围点对当前点深度值的影响：
		//noiseLevel = 0.05，深度距离值是通过查询点半径为 Scm 的圆内包含的点用来平均计算而得到的。

		float noise_level = 0.0;      //各种参数的设置
		float min_range = 0.0f;
		int border_size = 1;
		Eigen::Affine3f sensor_pose = Eigen::Affine3f::Identity();

		float angularResolution = pcl::deg2rad(0.03f);  // 0.03度转弧度
		float maxAngleWidth = pcl::deg2rad(81.7f);
		float maxAngleHeight = pcl::deg2rad(25.1f);
		pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
		range_image.createFromPointCloud(*lvx_obj.points_xyz, 
			angularResolution, 
			maxAngleWidth, 
			maxAngleHeight,
			sensor_pose, 
			coordinate_frame, 
			noise_level, 
			min_range, 
			border_size);
		
		// save
		float *ranges = range_image.getRangesArray();
		unsigned char *rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(ranges, range_image.width, range_image.height);
		pcl::io::saveRgbPNGFile("../result/rangeRGBImage.png", rgb_image, range_image.width, range_image.height);

		// visualizer
		pcl::visualization::RangeImageVisualizer range_image_widget("Range image");        
		range_image_widget.showRangeImage(range_image); //图像可视化方式显示深度图像
		pcl::visualization::PCLVisualizer viewer("3D Viewer");
		viewer.setBackgroundColor(1, 1, 1);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);
		viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
		viewer.initCameraParameters();
		bool live_update = false;
		while (!viewer.wasStopped())
		{
			range_image_widget.spinOnce();
			viewer.spinOnce();
			Sleep(0.01);
			if (live_update)
			{
				sensor_pose = viewer.getViewerPose();
				range_image.createFromPointCloud(*lvx_obj.points_xyz, 
					angularResolution, 
					maxAngleWidth, 
					maxAngleHeight,
					sensor_pose, 
					coordinate_frame, 
					noise_level, 
					min_range, 
					border_size);
				range_image_widget.showRangeImage(range_image);
			}
		}	
		std::cout << "success" << std::endl;
	}
	else if (strcmp(option, "rangeImagePlanar") == 0) 
	{
		// 点云生成深度图（球面投影）
		// 用boost是为了显示
		boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
		pcl::RangeImage& range_image = *range_image_ptr;

		//noiseLevel设置周围点对当前点深度值的影响：
		//noiseLevel = 0.05，深度距离值是通过查询点半径为 Scm 的圆内包含的点用来平均计算而得到的。

		float noise_level = 0.0;      //各种参数的设置
		float min_range = 0.0f;
		int border_size = 1;
		Eigen::Affine3f sensor_pose = Eigen::Affine3f::Identity();

		float angularResolution = pcl::deg2rad(0.03f);  // 0.03度转弧度
		float maxAngleWidth = pcl::deg2rad(81.7f);
		float maxAngleHeight = pcl::deg2rad(25.1f);
		pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
		range_image.createFromPointCloud(*lvx_obj.points_xyzrgb,
			angularResolution,
			maxAngleWidth,
			maxAngleHeight,
			sensor_pose,
			coordinate_frame,
			noise_level,
			min_range,
			border_size);
		std::cout << "range_image finish" << "\n";
		std::cout << "range_image_size = " << range_image.size()<< std::endl;
		//viusalization of range image
		pcl::visualization::RangeImageVisualizer range_image_widget("RangeImage");
		range_image_widget.showRangeImage(range_image);
		range_image_widget.setWindowTitle("RangeImage");

		//triangulation based on range image
		pcl::OrganizedFastMesh<pcl::PointWithRange>::Ptr tri(new pcl::OrganizedFastMesh<pcl::PointWithRange>);
		pcl::search::KdTree<pcl::PointWithRange>::Ptr tree(new pcl::search::KdTree<pcl::PointWithRange>);
		tree->setInputCloud(range_image_ptr);
		
		pcl::PolygonMesh triangles;
		tri->setTrianglePixelSize(3);
		tri->setInputCloud(range_image_ptr);
		tri->setSearchMethod(tree);
		//tri->setTriangulationType(pcl::OrganizedFastMesh<pcl::PointWithRange>::TRIANGLE_RIGHT_CUT);
		tri->setTriangulationType(pcl::OrganizedFastMesh<pcl::PointWithRange>::TRIANGLE_ADAPTIVE_CUT);

		tri->reconstruct(triangles);
		pcl::io::savePLYFileBinary("D:/pg_cpp/3D-reconstruction-PCL/result/imageRange.ply", triangles);

		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("RangeImage"));
		viewer->setBackgroundColor(0., 0., 0.);
		viewer->addPolygonMesh(triangles, "tin");
		viewer->addCoordinateSystem();
		while (!range_image_widget.wasStopped() && !viewer->wasStopped())
		{
			range_image_widget.spinOnce();
			Sleep(0.01);
			viewer->spinOnce();
		}
	}
	else {
		// 用点云meshing(贪婪投影三角形和poisson重建：点云 + 点云法向量)
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
		//pcl::io::savePLYFileBinary("D:/pg_cpp/3D-reconstruction-PCL/result/mesh.ply", mesh);
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
	dealwith_lvx(false, "rangeImagePlanar");
	return 0;
}

