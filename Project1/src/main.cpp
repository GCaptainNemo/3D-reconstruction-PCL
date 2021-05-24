#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/common/float_image_utils.h>

#include "../include/kitti_io_obj.h"
#include "../include/pc_operator.h"
#include "../include/lvx_io_obj.h"
#include "../include/texturing.h"
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
	
	// //////////////////////////////////////////////////////////////

	/*bool save = true;
	bool preprocess = false;
	dealwith_lvx(preprocess, "mc", save);
	return 0;*/
	

	// ///////////////////////////////////////////////////////////////
	//bool color = true;
	//if (color)
	//{

	//	// project each point to get color(low resolution) 
	//	//Texturing::color_mesh(*mesh.get(), lvx_obj.points_xyzrgb);
	//	Texturing texturing;
	//	pcl::PolygonMeshPtr mesh = pcl::PolygonMeshPtr(new pcl::PolygonMesh);
	//	
	//	if (pcl::io::loadPLYFile("../../dataset/mesh_without_color.ply", *mesh) == -1) {
	//		std::cout << "couldn't read file" << std::endl;
	//		return 0;
	//	};
	//	texturing.load_mesh(mesh);
	//	std::cout << "load-mesh-finish\n";
	//	std::string bundlePath = "../../dataset/bundle.rd_xinyang.out";
	//	texturing.load_camera(bundlePath);
	//	std::cout << "load-camera-finish\n";
	//	texturing.mesh_image_match();
	//	std::cout << "match-finish\n";
	//	texturing.calculate_patches();
	//	std::cout << "calculate-patches-finish\n";
	//	texturing.sort_patches();
	//	std::cout << "sort-patches-finish\n";
	//	texturing.create_textures();
	//	std::cout << "create textures-finish\n";
	//	const char * name = "texture_shapan.obj";
	//	texturing.write_obj_file(name);
	//	std::cout << "write obj file-finish\n";
	//}
}

