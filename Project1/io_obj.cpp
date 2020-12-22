#define _CRT_SECURE_NO_WARNINGS
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>//标准C++库中的输入输出类相关头文件。
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h> //PCL中支持的点类型头文件。
#include <boost/random.hpp>  //高斯噪点测试
#include <fstream>  
#include <string>  
#include <vector> 
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include "io_obj.h"
using namespace std;

ioOBJ::ioOBJ() 
{

}

ioOBJ::~ioOBJ() {

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
	fopen_s(&writePCDStream, filenameOutput, "wb");
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


}

void ioOBJ::read_image(const char * filename)
{

}
void ioOBJ::read_pointcloud(const char * filename)
{

}



	