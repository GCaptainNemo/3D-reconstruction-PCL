#include "lvx_io_obj.h"
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <Python.h>
#include <pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���


void dealwith_lvx(const char *filedir, const char *option)
{
	if (strcmp(option, "topcd") == 0) 
	{
		std::cout << "hello world\n";
		Py_Initialize();   //��ʼ��
		if (!Py_IsInitialized()) {
			std::cout << "python init fail" << std::endl;
			return;
		}
		PyRun_SimpleString("import sys");
		PyRun_SimpleString("sys.path.append('./lvx_parser')");

		std::cout << "finish1\n";

		PyObject * pModule = PyImport_ImportModule("pylvx");//������Ҫ���õ��ļ���
		if (pModule == NULL) {
			std::cout << "module not found" << std::endl;
			return;
		}
		PyObject * pFunc = PyObject_GetAttrString(pModule, "topcds");//������Ҫ���õĺ�����
		if (!pFunc) {
			std::cout << "get python function failed\n";
			return;
		}
		std::cout << "finish2\n";

		PyObject *pArgs = PyTuple_New(2);
		// ��test.lvx��֡���pcd��ʽ����output�ļ�����
		PyTuple_SetItem(pArgs, 0, Py_BuildValue("s", filedir)); // s������python��str����
		PyTuple_SetItem(pArgs, 1, Py_BuildValue("s", "output"));
		PyEval_CallObject(pFunc, pArgs);		//���ú���

		// ���PyObject 
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
		pcl::visualization::CloudViewer viewer("Simple Cloud Viewer"); //����һ����ʾ����
		viewer.showCloud(cloud);
		while (!viewer.wasStopped())
		{
		}
	}
}
