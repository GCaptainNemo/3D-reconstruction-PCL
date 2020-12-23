#define _CRT_SECURE_NO_WARNINGS
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>//标准C++库中的输入输出类相关头文件。
#include <pcl/point_types.h> //PCL中支持的点类型头文件。
#include <boost/random.hpp>  //高斯噪点测试
#include <fstream>  
#include <string>  
#include <vector> 
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>

#include "io_obj.h"
#include "utils.h"
#include <opencv2/opencv.hpp>



ioOBJ::ioOBJ() 
{
	this->points_xyzi = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	this->points_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
}

ioOBJ::~ioOBJ() {

}

void ioOBJ::bin2pcd2(const std::string & infile, const std::string & outfile)
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


void ioOBJ::bin2pcd(const char *filenameInput, const char *filenameOutput) 
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

void ioOBJ::read_calib(const char * filename)
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
		std::cout << stod(p2_vector[i]) << "\n";
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
		std::cout << stod(r0_vector[i]) << "\n";
		r0_vector_double[(i - 1) / 3][(i - 1) % 3] = stod(r0_vector[i]);
	}
	cv::Mat r0_matrix(3, 3, CV_64F, r0_vector_double);

	
	double _b[3] = { 0, 0, 0 };
	cv::Mat zeros(3, 1, CV_64F, _b);
	cv::hconcat(r0_matrix, zeros, r0_matrix);
	cv::vconcat(r0_matrix, ones, r0_matrix);
	std::cout << "r0_matrix = \n";
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			std::cout << r0_matrix.at<double>(i, j) << "     ";
		}
		std::cout << "\n";
	}
	


	// /////////////////////////////////////////////////////////////////////////
	// Tr matrix
	// /////////////////////////////////////////////////////////////////////////
	std::vector<std::string> Tr_vector = utils::split(string_Tr, " ");
	double Tr_vector_double[3][4];
	for (int i = 1; i < Tr_vector.size(); i++)
	{
		std::cout << stod(Tr_vector[i]) << "\n";
		Tr_vector_double[(i - 1) / 4][(i - 1) % 4] = stod(Tr_vector[i]);
	}
	cv::Mat Tr_matrix(3, 4, CV_64F, Tr_vector_double);
	cv::vconcat(Tr_matrix, ones, Tr_matrix);



	// ////////////////////////////////////////////////////////////////////////////////
	// Transform matrix
	// ////////////////////////////////////////////////////////////////////////////////
	this->transform_matrix = cv::Mat(P2_matrix * r0_matrix * Tr_matrix).clone();
	std::cout << "rows = " << this->transform_matrix.rows << "\n";
	std::cout << "columns = " << this->transform_matrix.cols << "\n";

}



void ioOBJ::read_image(const char * filename)
{
	
	this->image = cv::imread(filename).clone();

	/*cv::namedWindow("测试opencv");
	cv::imshow("测试opencv", img);
	cv::waitKey(6000);
	*/
}

void ioOBJ::read_bin_xyzrgb(const std::string &filename)
{
	fstream input(filename.c_str(), ios::in | ios::binary);
	if (!input.good())
	{
		cerr << "Could not read file: " << filename << endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, ios::beg);
	int row_bound = this->image.rows;
	int column_bound = this->image.cols;
	std::cout << "row_bound = " << row_bound << "\n column_bound = " << column_bound;
	std::cout << "transform_matrix = \n";
	for (int i = 0; input.good() && !input.eof(); i++)
	{
		pcl::PointXYZRGB point;
		input.read((char *)&point.x, 3 * sizeof(float));
		
		double a_[4] = { point.x, point.y, point.z, 1.0 };
		cv::Mat pos(4, 1, CV_64F, a_);
		cv::Mat newpos(this->transform_matrix * pos);
		int x = (int)(newpos.at<double>(0, 0) / newpos.at<double>(2, 0));
		int y = (int)(newpos.at<double>(1, 0) / newpos.at<double>(2, 0));
		
		if (point.x >= 0)
		{
			if (x >= 0 && x < column_bound && y >= 0 && y < row_bound)
			{
				//  imread是BGR（BITMAP）
				point.r = this->image.at<cv::Vec3b>(y, x)[2];  
				point.g = this->image.at<cv::Vec3b>(y, x)[1];
				point.b = this->image.at<cv::Vec3b>(y, x)[0];
				this->points_xyzrgb->push_back(point);
			}
		}
	}
	input.close();

}


void ioOBJ::read_bin_xyzi(const std::string & filename, bool crop)
{
	// bug exists, don't use.
	fstream input(filename.c_str(), ios::in | ios::binary);
	if (!input.good())
	{
		cerr << "Could not read file: " << filename << endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, ios::beg);
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
		std::cout << "row_bound = " << row_bound << "\n column_bound = " << column_bound;
		std::cout << "transform_matrix = \n";
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
		std::cout << "Couldn't read file rabbit.pcd" << "\n";
		//return(-1);
	}
	std::cout << cloud->points.size() << std::endl;
	//可视化
	pcl::visualization::CloudViewer viewer("cloud viewer");
	viewer.showCloud(cloud);
	//viewer.runOnVisualizationThreadOnce(viewerOneOff);
	system("pause");
}

void read_pcd_file(std::string pcd_path, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1)
	{
		std::cout << "Couldn't read file " << pcd_path << "\n";
		//PCL_ERROR("Couldn't read file rabbit.pcd\n");
		//return(-1);
	}
	std::cout << cloud->points.size() << std::endl;
}






	