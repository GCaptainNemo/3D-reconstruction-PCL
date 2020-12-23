#include "pc_operator.h"

void pc_operator::statistical_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter,
	int neighbour, float proportion)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(neighbour);   // ÁÚ¾ÓÊý
	sor.setStddevMulThresh(proportion); // thresh = mean + proportion * std
	sor.filter(*cloud_filter);

}


