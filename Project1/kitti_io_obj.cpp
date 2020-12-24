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
#include "kitti_io_obj.h"
#include "utils.h"



ioOBJ::ioOBJ() 
{
	this->points_xyzi = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	this->points_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	this->points_xyz = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

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
	//����ת����filenameList���ڱ����ļ���
	int32_t num = 1000000;
	float *data = (float*)malloc(num * sizeof(float));
	// ��
	float *px = data + 0;
	float *py = data + 1;
	float *pz = data + 2;
	float *pr = data + 3;//����ǿ��
	// ��ȡ��������
	FILE *stream;
	fopen_s(&stream, filenameInput, "rb");
	num = fread(data, sizeof(float), num, stream) / 4;//����������ݣ����10��+����
	fclose(stream);
	//ת����PCD
	//д�ļ�����
	/*string previous = "G:/pcl_source/pcl_test/pcl_test/pcd/data/00000000";
	string tmp1 = previous.append(to_string(version));
	string tmp2 = tmp1.append(".pcd");
	const char* outputFilename = tmp2.c_str();*/
	FILE *writePCDStream;
	fopen_s(&writePCDStream, filenameOutput, "wb+");
	fprintf(writePCDStream, "VERSION 0.7\n");//�汾˵��
	fprintf(writePCDStream, "FIELDS x y z intensity\n");//ά��˵��
	fprintf(writePCDStream, "SIZE 4 4 4 4\n");//ռ���ֽ�˵��
	fprintf(writePCDStream, "TYPE F F F F\n");//�����������Ͷ���
	fprintf(writePCDStream, "WIDTH %d\n", num);//������
	fprintf(writePCDStream, "HEIGHT 1\n");//�������Ĭ��Ϊ1
	fprintf(writePCDStream, "POINTS %d\n", num);//������
	fprintf(writePCDStream, "DATA ascii\n");//�ĵ�ʹ���ַ�����shuom
	//д��������
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

	while (getline(in, s))//���ж�ȡ���ݲ�����s�У�ֱ������ȫ����ȡ
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



void ioOBJ::read_image(const char * filename)
{
	
	this->image = cv::imread(filename).clone();

	/*cv::namedWindow("����opencv");
	cv::imshow("����opencv", img);
	cv::waitKey(6000);
	*/
}

void ioOBJ::read_bin_xyz(const std::string &filename, bool iscrop)
{
	fstream input(filename.c_str(), ios::in | ios::binary);
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
		// ����transform-matrix�ü�һ����
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


void ioOBJ::project_get_rgb() {
	
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
				//  imread��BGR��BITMAP��
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
		// ����transform-matrix�ü�һ����
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






	