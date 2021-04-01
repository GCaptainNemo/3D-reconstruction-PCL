#include "../include/lvx_io_obj.h"
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <Python.h>
#include <pcl/io/pcd_io.h>
#include <io.h>


LvxObj::LvxObj()
{

	// initialize lvxobj
	this->points_xyzi = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	this->points_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	this->points_xyz = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}

void LvxObj::lvx2pcd(const char *filedir, const char *option)
{
	if (strcmp(option, "topcd") == 0) 
	{
		// initialize python
		Py_Initialize();  
		if (!Py_IsInitialized()) {
			std::cout << "python init fail" << std::endl;
			return;
		}

		// append ./lvx_parser dir to python interpreter sys.path
		PyRun_SimpleString("import sys");
		PyRun_SimpleString("sys.path.append('./lvx_parser')");

		// call python module pylvx
		PyObject * pModule = PyImport_ImportModule("pylvx");
		if (pModule == NULL) 
		{
			std::cout << "module not found" << std::endl;
			return;
		}

		// call function topcds in the module pylvx
		PyObject * pFunc = PyObject_GetAttrString(pModule, "topcds");
		if (!pFunc) 
		{
			std::cout << "get python function failed\n";
			return;
		}
		
		// change .lvx to .pcd in output dir
		PyObject *pArgs = PyTuple_New(2);
		PyTuple_SetItem(pArgs, 0, Py_BuildValue("s", filedir));  // s represent python str variable
		PyTuple_SetItem(pArgs, 1, Py_BuildValue("s", "output"));
		
		// call function
		PyEval_CallObject(pFunc, pArgs);	

		// clear PyObject 
		Py_DECREF(pModule);
		Py_DECREF(pFunc);
		Py_DECREF(pArgs);
		Py_Finalize();
	}
}

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

void LvxObj::read_image(const char *filename, const bool & correct)
{
	// load image
	cv::Mat image_ = cv::imread(filename).clone();
	if (correct) 
	{
		// use distortion parameters to correct image
		cv::undistort(image_, this->image, this->intrinsic_matrix, this->dist_matrix);
	}
	cv::namedWindow("test opencv");
	cv::imshow("test opencv", this->image);
	cv::waitKey(6000);
}

void LvxObj::set_calib() 
{
	// 5 distortion parameters
	double dist[5] = { 4.9811000000000001e-02, -2.7529999999999998e-03,
	   -2.2499999999999998e-03, 3.9249999999999997e-03, 0. };
	cv::Mat dist_array(5, 1, CV_64F, dist);
	this->dist_matrix = dist_array.clone();

	// camera intrinsic matrix
	double Intrinsic[3][3] = { 1.6634617699999999e+03, 0., 9.7235897999999997e+02, 0.,
	   1.6652231500000000e+03, 5.1716867000000002e+02, 0., 0., 1. };
	cv::Mat Int(3, 3, CV_64F, Intrinsic);
	this->intrinsic_matrix = Int.clone();

	// camera extrinsic matrix:from global coordinate to local coordinate
	double Extrin_matrix[4][4] = { 1.6036083208305518e-02, -7.1163875152638334e-03,
	9.9984608868768832e-01, 5.0296302884817123e-02,
	-9.9985571454334576e-01, 5.4894932912082917e-03,
	1.6075308968138691e-02, -5.5230446159839630e-02,
	-5.6030465241367899e-03, -9.9995961043041048e-01,
	-7.0273307528522788e-03, 6.9595232605934143e-02,
	0., 0., 0., 1. };
	cv::Mat ext_(4, 4, CV_64F, Extrin_matrix);
	cv::Mat invRt = ext_(cv::Rect(0, 0, 3, 3));
	cv::Mat R = invRt.t();
	cv::Mat invT = -R * ext_(cv::Rect(3, 0, 1, 3));
	cv::hconcat(R, invT, this->extrinsic_matrix);
	
	// transform matrix: from global coordinate to image coordinate
	this->transform_matrix = Int * this->extrinsic_matrix;
}


void LvxObj::project_get_rgb() 
{
	// according to transform matrix project point cloud to the image to get the color
	int size = this->points_xyz->size();
	std::cout << "need to be projected size = " << size << std::endl;
	int row_bound = this->image.rows;
	int column_bound = this->image.cols;
	this->points_xyzrgb->clear();
	pcl::PointCloud<pcl::PointXYZ>::Ptr linshi_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	linshi_xyz->clear();
	for (int i = 0; i < size; i++)
	{
		// project get the photo coordinate
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

		// Trims viewport according to image size
		if (point.x >= 0)
		{
			if (x >= 0 && x < column_bound && y >= 0 && y < row_bound)
			{
				//  imread BGR（BITMAP）
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

void LvxObj::read_pcds_xyz(const std::string & dir, const bool &iscrop, const int & frame_num)
{
	intptr_t handle;  // 为了跨平台适应intptr_t
	struct _finddata_t fileinfo;
	//第一次查找
	std::string p;
	handle = _findfirst(p.append(dir).append("/*.pcd").c_str(), &fileinfo);
	if (handle == -1){ 
		return; 
		std::cout << "handle == -1" << std::endl;
	}
		
	std::vector<std::string> files;
	
	do
	{
		//找到的文件的文件名
		printf("%s\n", fileinfo.name);
		files.push_back(p.assign(dir).append("/").append(fileinfo.name));
	} while (!_findnext(handle, &fileinfo));
	_findclose(handle);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	for (int frame = 0; frame < frame_num; frame++) 
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_frame(new pcl::PointCloud<pcl::PointXYZ>);
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(files[frame].c_str(), *cloud_frame) == -1) {
			std::cout << "Couldn't read file" << "\n";
		}
		else {
			*cloud += *cloud_frame;
		}
	}
	std::cout << "cloud-size = " << cloud->size();
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
		std::cout << "size = " << this->points_xyz->size();
	}
}
LvxObj::~LvxObj()
{
}
