#include "lvx_io_obj.h"
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <Python.h>
#include <pcl/io/pcd_io.h>//pcd 读写类相关的头文件。


LvxObj::LvxObj()
{
	this->points_xyzi = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	this->points_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	this->points_xyz = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}

LvxObj::~LvxObj() 
{
}



void LvxObj::lvx2pcd(const char *filedir, const char *option)
{
	if (strcmp(option, "topcd") == 0) 
	{
		std::cout << "hello world\n";
		Py_Initialize();   //初始化
		if (!Py_IsInitialized()) {
			std::cout << "python init fail" << std::endl;
			return;
		}
		PyRun_SimpleString("import sys");
		PyRun_SimpleString("sys.path.append('./lvx_parser')");

		std::cout << "finish1\n";

		PyObject * pModule = PyImport_ImportModule("pylvx");//这里是要调用的文件名
		if (pModule == NULL) {
			std::cout << "module not found" << std::endl;
			return;
		}
		PyObject * pFunc = PyObject_GetAttrString(pModule, "topcds");//这里是要调用的函数名
		if (!pFunc) {
			std::cout << "get python function failed\n";
			return;
		}
		std::cout << "finish2\n";

		PyObject *pArgs = PyTuple_New(2);
		// 把test.lvx按帧输出pcd形式，在output文件夹中
		PyTuple_SetItem(pArgs, 0, Py_BuildValue("s", filedir)); // s代表创建python中str变量
		PyTuple_SetItem(pArgs, 1, Py_BuildValue("s", "output"));
		PyEval_CallObject(pFunc, pArgs);		//调用函数

		// 清空PyObject 
		Py_DECREF(pModule);
		Py_DECREF(pFunc);
		Py_DECREF(pArgs);
		Py_Finalize();
	}
	std::cout << "123\n";
}

//void openPCDfile(const std::string &file_dir, const bool &show)
void LvxObj::openPCDfile(const char * file_dir, const bool &show)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	if (pcl::io::loadPCDFile<pcl::PointXYZI>(file_dir, *cloud) == -1) {
		std::cout << "Couldn't read file" << "\n";
		return;
	}
	if (show) {
		pcl::visualization::CloudViewer viewer("Simple Cloud Viewer"); //创造一个显示窗口
		viewer.showCloud(cloud);
		while (!viewer.wasStopped())
		{
		}
	}
}

void LvxObj::read_image(const char *filename)
{
	this->image = cv::imread(filename).clone();
	cv::namedWindow("测试opencv");
	cv::imshow("测试opencv", this->image);
	cv::waitKey(6000);
}

void LvxObj::set_calib() 
{
	double Extrin_matrix[4][4] = { 1.6036083208305518e-02, -7.1163875152638334e-03,
		9.9984608868768832e-01, 5.0296302884817123e-02,
		-9.9985571454334576e-01, 5.4894932912082917e-03,
		1.6075308968138691e-02, -5.5230446159839630e-02,
		-5.6030465241367899e-03, -9.9995961043041048e-01,
		-7.0273307528522788e-03, 6.9595232605934143e-02, 
		0., 0., 0., 1. };

	cv::Mat ext_(4, 4, CV_64F, Extrin_matrix);
	cv::Mat invRt = ext_(cv::Rect(0, 0, 3, 3));                       // Camera extrinsic rotation matrix (Invert from world extrinsic).
	cv::Mat R = invRt.t();
	cv::Mat invT = -R * ext_(cv::Rect(3, 0, 1, 3)); // Camera extrinsic translate matrix (Invert from world extrinsic).

	//cv::Mat R = 
	double Intrinsic[3][3] = { 1.6634617699999999e+03, 0., 9.7235897999999997e+02, 0.,
	   1.6652231500000000e+03, 5.1716867000000002e+02, 0., 0., 1. };
	cv::Mat Int(3, 3, CV_64F, Intrinsic);
	cv::hconcat(R, invT, this->transform_matrix);

	this->transform_matrix = Int * this->transform_matrix;
	for (int row = 0; row < 3; row++) {
		for (int column = 0; column < 4; column++) {
			std::cout << this->transform_matrix.at<double>(row, column) <<std::endl;
		}
	}
	double dist[5] = { 4.9811000000000001e-02, -2.7529999999999998e-03,
	   -2.2499999999999998e-03, 3.9249999999999997e-03, 0. };
}


void LvxObj::project_get_rgb() {
	int size = this->points_xyz->size();
	std::cout << "need to be projected size = " << size << std::endl;
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


void LvxObj::read_pcd_xyz(const char * filename, const bool &iscrop)
{
	// c_str()转换成char *
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
		std::cout << "Couldn't read file" << "\n";
		return;
	}
	this->points_xyz->clear();
	if (!iscrop) 
	{
		pcl::copyPointCloud(*cloud, *(this->points_xyz));
		return;
	}
	else
	{
		// 根据transform-matrix裁剪一部分
		int size = cloud->size();
		std::cout << "cloud-size = " << size << std::endl;
		int row_bound = this->image.rows;
		int column_bound = this->image.cols;
		for (int i = 0; i < size; i++)
		{
			pcl::PointXYZ point = cloud->points[i];
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
}



