#define _CRT_SECURE_NO_WARNINGS
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> 
#include <boost/random.hpp>  
#include <fstream>  
#include <string>  
#include <vector> 
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <opencv2/opencv.hpp>
#include "../include/kitti_io_obj.h"
#include "../include/utils.h"
#include "../include/pc_operator.h"
#include "../include/texturing.h"

KittiObj::KittiObj() 
{
	this->points_xyzi = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	this->points_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	this->points_xyz = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

}

KittiObj::~KittiObj() {

}

void dealwith_kitti(const bool &preprocess, const char * option)
{

	// load image, calibration information and .bin file of Kitti dataset.
	KittiObj kitti_obj;
	kitti_obj.read_image("../resources/image/000000.png");
	kitti_obj.read_calib("../resources/calib/000000.txt");
	kitti_obj.read_bin_xyz("../resources/Lidar/000000.bin", true);
	std::cout << "before filter size = " << kitti_obj.points_xyz->size() << std::endl;

	// point cloud preprocessing
	if (preprocess) {
		pc_operator::down_sample(kitti_obj.points_xyz, kitti_obj.points_xyz, 0.01);
		pc_operator::statistical_filter(kitti_obj.points_xyz, kitti_obj.points_xyz, 50, 3.0);
		// pcl::io::savePCDFile("/pcd", *cloud);
		pc_operator::resampling(kitti_obj.points_xyz, kitti_obj.points_xyz, 0.05); // 平滑
		pc_operator::upsampling(kitti_obj.points_xyz, kitti_obj.points_xyz);
		pc_operator::random_sampling(kitti_obj.points_xyz, kitti_obj.points_xyz, 60000);
	}

	// use transform matrix and image to cull point cloud and give each point color.
	kitti_obj.project_get_rgb();
	if (strcmp(option, "pc") == 0)
	{

		// show colored point cloud directly(without meshing)
		pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
		viewer.showCloud(kitti_obj.points_xyzrgb);
		while (!viewer.wasStopped())
		{
		}
	}
	else {

		// meshing greedy projection triangulation and poisson reconstrucion
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

		// estimate point cloud normal vector
		pc_operator::estimate_normal(kitti_obj.points_xyz, normals, 10);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgbcloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::concatenateFields(*(kitti_obj.points_xyzrgb), *normals, *rgbcloud_with_normals);
		pcl::PolygonMeshPtr mesh;
		if (strcmp(option, "poisson") == 0)
		{
			// poisson reconstruction
			pc_operator::poisson_reconstruction(rgbcloud_with_normals, mesh);

			// mesh decimation reduce 20% vertex
			pc_operator::decimateMesh(0.2, mesh);

			// use 1nn to give poisson mesh texture(based on point)
			texturing::color_mesh(*mesh, kitti_obj.points_xyzrgb);
		}
		else if (strcmp(option, "greedy") == 0)
		{
			// greedy projection triangulation
			pc_operator::triangular(rgbcloud_with_normals, *mesh);
		}

		// visualize meshing results
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("mesh"));
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPolygonMesh(*mesh, "mesh");
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


void KittiObj::bin2pcd2(const std::string & infile, const std::string & outfile)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);
	
	fstream input(infile.c_str(), ios::in | ios::binary);
	if (!input.good())
	{
		cerr << "Could not read file: " << infile << endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, ios::beg);

	int i;
	for (i = 0; input.good() && !input.eof(); i++)
	{
		pcl::PointXYZI point;
		input.read((char *)&point.x, 3 * sizeof(float));
		input.read((char *)&point.intensity, sizeof(float));
		points->push_back(point);
	}
	input.close();
	std::cout << "Read KTTI point cloud with " << i << " points, writing to " << outfile << endl;
	pcl::io::savePCDFileBinary(outfile, *points);

}


void KittiObj::bin2pcd(const char *filenameInput, const char *filenameOutput) 
{ 
	//批量转换，filenameList用于保存文件名
	int32_t num = 1000000;
	float *data = (float*)malloc(num * sizeof(float));
	// 点
	float *px = data + 0;
	float *py = data + 1;
	float *pz = data + 2;
	float *pr = data + 3;//反射强度
	// 读取点云数据
	FILE *stream;
	fopen_s(&stream, filenameInput, "rb");
	num = fread(data, sizeof(float), num, stream) / 4;//读入点云数据，大概10万+个点
	fclose(stream);
	//转换成PCD
	//写文件声明
	/*string previous = "G:/pcl_source/pcl_test/pcl_test/pcd/data/00000000";
	string tmp1 = previous.append(to_string(version));
	string tmp2 = tmp1.append(".pcd");
	const char* outputFilename = tmp2.c_str();*/
	FILE *writePCDStream;
	fopen_s(&writePCDStream, filenameOutput, "wb+");
	fprintf(writePCDStream, "VERSION 0.7\n");//版本说明
	fprintf(writePCDStream, "FIELDS x y z intensity\n");//维度说明
	fprintf(writePCDStream, "SIZE 4 4 4 4\n");//占用字节说明
	fprintf(writePCDStream, "TYPE F F F F\n");//具体数据类型定义
	fprintf(writePCDStream, "WIDTH %d\n", num);//点数量
	fprintf(writePCDStream, "HEIGHT 1\n");//无序点云默认为1
	fprintf(writePCDStream, "POINTS %d\n", num);//点数量
	fprintf(writePCDStream, "DATA ascii\n");//文档使用字符类型shuom
	//写点云数据
	for (int32_t i = 0; i < num; i++)
	{
		fprintf(writePCDStream, "%f %f %f %f\n", *px, *py, *pz, *pr);
		px += 4; py += 4; pz += 4; pr += 4;
	}
	fclose(writePCDStream);
}

void KittiObj::read_calib(const char * filename)
{
	ifstream in(filename);
	std::string s;
	int line_index = 0;
	std::string string_p2;
	std::string string_r0;
	std::string string_Tr;

	while (getline(in, s))//着行读取数据并存于s中，直至数据全部读取
	{
		if (line_index == 2) {
			string_p2 = s;
		}
		else if (line_index == 4) {
			string_r0 = s;
		}
		else if (line_index == 5) {
			string_Tr = s;
		}
		line_index++;
	}


	double _a[4] = { 0, 0, 0, 1 };
	cv::Mat ones(1, 4, CV_64F, _a);

	// /////////////////////////////////////////////////////////////////////////
	// p2 matrix
	// /////////////////////////////////////////////////////////////////////////
	std::vector<std::string> p2_vector = utils::split(string_p2, " ");
	double p2_vector_double[3][4];
	for (int i = 1; i < p2_vector.size(); i++)
	{
		p2_vector_double[(i - 1) / 4][(i - 1) % 4] = stod(p2_vector[i]);
	}
	cv::Mat P2_matrix(3, 4, CV_64F, p2_vector_double);

	// /////////////////////////////////////////////////////////////////////////
	// r0 matrix
	// /////////////////////////////////////////////////////////////////////////
	std::vector<std::string> r0_vector = utils::split(string_r0, " ");
	double r0_vector_double[3][3];
	for (int i = 1; i < r0_vector.size(); i++)
	{
		r0_vector_double[(i - 1) / 3][(i - 1) % 3] = stod(r0_vector[i]);
	}
	cv::Mat r0_matrix(3, 3, CV_64F, r0_vector_double);

	
	double _b[3] = { 0, 0, 0 };
	cv::Mat zeros(3, 1, CV_64F, _b);
	cv::hconcat(r0_matrix, zeros, r0_matrix);
	cv::vconcat(r0_matrix, ones, r0_matrix);
	

	// /////////////////////////////////////////////////////////////////////////
	// Tr matrix
	// /////////////////////////////////////////////////////////////////////////
	std::vector<std::string> Tr_vector = utils::split(string_Tr, " ");
	double Tr_vector_double[3][4];
	for (int i = 1; i < Tr_vector.size(); i++)
	{
		Tr_vector_double[(i - 1) / 4][(i - 1) % 4] = stod(Tr_vector[i]);
	}
	cv::Mat Tr_matrix(3, 4, CV_64F, Tr_vector_double);
	cv::vconcat(Tr_matrix, ones, Tr_matrix);



	// ////////////////////////////////////////////////////////////////////////////////
	// Transform matrix
	// ////////////////////////////////////////////////////////////////////////////////
	this->transform_matrix = cv::Mat(P2_matrix * r0_matrix * Tr_matrix).clone();
	
}



void KittiObj::read_image(const char * filename)
{
	this->image = cv::imread(filename).clone();
	/*cv::namedWindow("测试opencv");
	cv::imshow("测试opencv", img);
	cv::waitKey(6000);
	*/
}

void KittiObj::read_bin_xyz(const char * filename, const bool &iscrop)
{
	fstream input(filename, ios::in | ios::binary);
	if (!input.good())
	{
		cerr << "Could not read file: " << filename << endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, ios::beg);
	this->points_xyz->clear();
	if (!iscrop)
	{
		for (int i = 0; input.good() && !input.eof(); i++)
		{
			pcl::PointXYZ point;
			//pcl::PointXYZRGB point;
			float _itensity; // useless
			input.read((char *)&point.x, 3 * sizeof(float));
			input.read((char *)&_itensity, sizeof(float));
			this->points_xyz->push_back(point);
		}
	}
	else 
	{
		// 根据transform-matrix裁剪一部分
		int row_bound = this->image.rows;
		int column_bound = this->image.cols;
		for (int i = 0; input.good() && !input.eof(); i++)
		{
			pcl::PointXYZ point;
			float _itensity;
			input.read((char *)&point.x, 3 * sizeof(float));
			input.read((char *)&_itensity, sizeof(float));
			double a_[4] = { point.x, point.y, point.z, 1.0 };
			cv::Mat pos(4, 1, CV_64F, a_);
			cv::Mat newpos(this->transform_matrix * pos);
			float x = (float)(newpos.at<double>(0, 0) / newpos.at<double>(2, 0));
			float y = (float)(newpos.at<double>(1, 0) / newpos.at<double>(2, 0));
			if (point.x >= 0)
			{
				if (x >= 0 && x < column_bound && y >= 0 && y < row_bound)
				{

					this->points_xyz->push_back(point);
				}
			}
		}	
	}
	input.close();
}


void KittiObj::project_get_rgb() {
	
	int size = this->points_xyz->size();
	int row_bound = this->image.rows;
	int column_bound = this->image.cols;
	this->points_xyzrgb->clear();
	pcl::PointCloud<pcl::PointXYZ>::Ptr linshi_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	linshi_xyz->clear();
	for (int i = 0; i < size; i++)
	{
		pcl::PointXYZRGB pointRGB;
		pcl::PointXYZ point = this->points_xyz->points[i];
		pointRGB.x = point.x;
		pointRGB.y = point.y;
		pointRGB.z = point.z;
		double a_[4] = { point.x, point.y, point.z, 1.0 };
		cv::Mat pos(4, 1, CV_64F, a_);
		cv::Mat newpos(this->transform_matrix * pos);
		float x = (float)(newpos.at<double>(0, 0) / newpos.at<double>(2, 0));
		float y = (float)(newpos.at<double>(1, 0) / newpos.at<double>(2, 0));

		if (point.x >= 0)
		{
			if (x >= 0 && x < column_bound && y >= 0 && y < row_bound)
			{
				//  imread是BGR（BITMAP）
				int row = int(y);
				int column = int(x);
				pointRGB.r = this->image.at<cv::Vec3b>(row, column)[2];
				pointRGB.g = this->image.at<cv::Vec3b>(row, column)[1];
				pointRGB.b = this->image.at<cv::Vec3b>(row, column)[0];
				this->points_xyzrgb->push_back(pointRGB);
				linshi_xyz->push_back(point);
			}
		}
	}
	pcl::copyPointCloud(*linshi_xyz, *(this->points_xyz));
	std::cout << "after_projected_get_rgb = " << this->points_xyz->size();
};

void KittiObj::read_bin_xyzi(const std::string & filename, const bool &crop)
{
	// bug exists, don't use.
	fstream input(filename.c_str(), ios::in | ios::binary);
	if (!input.good())
	{
		cerr << "Could not read file: " << filename << endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, ios::beg);
	this->points_xyzi->clear();
	if (!crop)
	{	
		for (int i = 0; input.good() && !input.eof(); i++)
		{
			pcl::PointXYZI point;
			input.read((char *)&point.x, 3 * sizeof(float));
			input.read((char *)&point.intensity, sizeof(float));
			this->points_xyzi->push_back(point);
		}
		input.close();
	}
	else {
		// 根据transform-matrix裁剪一部分
		int row_bound = this->image.rows;
		int column_bound = this->image.cols;
		
		for (int i = 0; input.good() && !input.eof(); i++)
		{
			pcl::PointXYZI point;
			input.read((char *)&point.x, 3 * sizeof(float));
			input.read((char *)&point.intensity, sizeof(float));
			double a_[4] = {point.x, point.y, point.z, 1.0};
			cv::Mat pos(4, 1, CV_64F, a_);
			cv::Mat newpos(this->transform_matrix * pos);
			float x = (float) (newpos.at<double>(0, 0) / newpos.at<double>(2, 0));
			float y = (float) (newpos.at<double>(1, 0) / newpos.at<double>(2, 0));
			if(point.x >= 0)
			{
				if (x >= 0 && x < column_bound && y >= 0 && y < row_bound) 
				{
					
					this->points_xyzi->push_back(point);
				}
			}
		}
		input.close();
	}
}


void read_pcd(const std::string &  pcd_path)
{
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
		std::cout << "Couldn't read file" << "\n";
	}	
}

void read_pcd_file(std::string pcd_path, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1)
	{
		std::cout << "Couldn't read file " << pcd_path << "\n";
	}
}






	