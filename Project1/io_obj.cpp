#define _CRT_SECURE_NO_WARNINGS
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>//��׼C++���е�������������ͷ�ļ���
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h> //PCL��֧�ֵĵ�����ͷ�ļ���
#include <boost/random.hpp>  //��˹������
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
	fopen_s(&writePCDStream, filenameOutput, "wb");
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


}

void ioOBJ::read_image(const char * filename)
{

}
void ioOBJ::read_pointcloud(const char * filename)
{

}



	