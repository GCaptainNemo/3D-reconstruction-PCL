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
