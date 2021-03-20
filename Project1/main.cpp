#define _CRT_SECURE_NO_WARNINGS
#include <iostream>

#include <pcl/visualization/cloud_viewer.h>//���Ʋ鿴����ͷ�ļ�
#include "kitti_io_obj.h"
#include "pc_operator.h"
#include <iostream>
#include <string>
using namespace pcl;

void dealwith_kitti(bool preprocess, std::string option) 
{
	ioOBJ io_obj;
	io_obj.read_image("../resources/image/000000.png");
	io_obj.read_calib("../resources/calib/000000.txt");
	io_obj.read_bin_xyz("../resources/Lidar/000000.bin", true);
	std::cout << "before filter size = " << io_obj.points_xyz->size() << std::endl;

	// ����Ԥ����
	if (preprocess) {
		pc_operator::down_sample(io_obj.points_xyz, io_obj.points_xyz, 0.01);
		pc_operator::statistical_filter(io_obj.points_xyz, io_obj.points_xyz, 50, 3.0);
		// pcl::io::savePCDFile("/pcd", *cloud);
		pc_operator::resampling(io_obj.points_xyz, io_obj.points_xyz, 0.05); // ƽ��
		pc_operator::upsampling(io_obj.points_xyz, io_obj.points_xyz);
		pc_operator::random_sampling(io_obj.points_xyz, io_obj.points_xyz, 60000);
	}
	
	// ֱ����ʾ���ɫ����(������)
	io_obj.project_get_rgb();
	if(option == "pc")
	{
		pcl::visualization::CloudViewer viewer("Simple Cloud Viewer"); //����һ����ʾ����
		viewer.showCloud(io_obj.points_xyzi);
		viewer.showCloud(io_obj.points_xyzrgb);
		viewer.showCloud(io_obj.points_xyzrgb);
		while (!viewer.wasStopped())
		{
		}
	}
	else{
		// ��������(̰��ͶӰ������ �� poisson�ؽ�����Ҫ���� + ���Ʒ�����)
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
		// ��ʾ���񻯽��
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
	dealwith_kitti(0, "pc");
	return 0;
}

