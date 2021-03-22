#include "lvx_io_obj.h"
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <Python.h>
#include <pcl/io/pcd_io.h>//pcd 读写类相关的头文件。


void dealwith_lvx(const char *filedir, const char *option)
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
void openPCDfile(const char * file_dir, const bool &show)
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
}

void LvxObj::read_calib(const char *filename) 
{
	double Extrin_matrix[4][4] = { 1.6036083208305518e-02, -7.1163875152638334e-03,
		9.9984608868768832e-01, 5.0296302884817123e-02,
		-9.9985571454334576e-01, 5.4894932912082917e-03,
		1.6075308968138691e-02, -5.5230446159839630e-02,
		-5.6030465241367899e-03, -9.9995961043041048e-01,
		-7.0273307528522788e-03, 6.9595232605934143e-02, 0., 0., 0., 1. }
	double Intrinsic[3][3] = { 1.6634617699999999e+03, 0., 9.7235897999999997e+02, 0.,
	   1.6652231500000000e+03, 5.1716867000000002e+02, 0., 0., 1. }
	double dist[5] = { 4.9811000000000001e-02, -2.7529999999999998e-03,
	   -2.2499999999999998e-03, 3.9249999999999997e-03, 0. }
}

