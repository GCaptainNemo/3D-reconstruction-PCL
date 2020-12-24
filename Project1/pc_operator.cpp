#include "pc_operator.h"


#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/ply_io.h>

void pc_operator::down_sample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downSampled, const float & voxel_size)

{
	// 下采样
	pcl::VoxelGrid<pcl::PointXYZ> downSampled;  //创建滤波对象
	downSampled.setInputCloud(cloud);            //设置需要过滤的点云给滤波对象
	downSampled.setLeafSize(voxel_size, voxel_size, voxel_size);  //设置滤波时创建的体素体积为1cm的立方体
	downSampled.filter(*cloud_downSampled);           //执行滤波处理，存储输出
	std::cout << "downsampled point cloud = " << cloud_downSampled->size() << std::endl;
}


void pc_operator::upsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_upsampled) 
{
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	std::cout << "before filter pc = " << cloud->size() << std::endl;
	pcl::PointCloud<pcl::PointXYZ> filteredCloud;    //输出MLS
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
	filter.setSearchMethod(kdtree);
	filter.setSearchRadius(5);
	filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
	filter.setUpsamplingRadius(0.05);
	filter.setUpsamplingStepSize(0.01);
	filter.process(filteredCloud);
	std::cout << "upsampled point cloud = " << filteredCloud.size() << std::endl;
	cloud_upsampled = filteredCloud.makeShared();

}


void pc_operator::random_sampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_randomsampled, const int & number) 
{
	
	pcl::RandomSample<pcl::PointXYZ> rs;
	rs.setInputCloud(cloud);	    	// 设置输出点的数量   
	rs.setSample(number);
	rs.filter(*cloud_randomsampled);  // 下采样并输出到cloud_out
	std::cout << "cloud_randomsampled = " << cloud_randomsampled->size() << std::endl;

}

void pc_operator::resampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed, const float &searchRadius)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr treeSampling(new pcl::search::KdTree<pcl::PointXYZ>);// 创建用于最近邻搜索的KD-Tree
	pcl::PointCloud<pcl::PointXYZ> mls_point;    //输出MLS
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls; // 定义最小二乘实现的对象mls
	mls.setComputeNormals(true);               // 最小二乘计算中是否计算法线
	mls.setInputCloud(cloud);                  // 设置待处理点云
	// mls.setPolynomialOrder(2);              // 拟合2阶多项式拟合
	mls.setPolynomialFit(true);                // 设置为false可以 加速 smooth
	mls.setSearchMethod(treeSampling);         // 设置KD-Tree作为搜索方法
	mls.setSearchRadius(searchRadius);           // 单位m.设置用于拟合的K近邻半径
	mls.process(mls_point);                 //输出
	// 输出重采样结果
	cloud_smoothed = mls_point.makeShared();
	std::cout << "cloud_smoothed(resampled) =  " << cloud_smoothed->size() << std::endl;
}
void pc_operator::statistical_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter,
	const int & neighbour, const float & proportion)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(neighbour);   // 邻居数
	sor.setStddevMulThresh(proportion); // thresh = mean + proportion * std
	sor.filter(*cloud_filter);
	std::cout << "statistical filtered point: " << cloud_filter->size() << std::endl;
}




void pc_operator::estimate_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::Normal>::Ptr normals, const int &nPoints) 
{
	// 法线估计
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;             //创建法线估计的对象
	normalEstimation.setInputCloud(cloud);                         //输入点云
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);// 创建用于最近邻搜索的KD-Tree
	normalEstimation.setSearchMethod(tree);
	// pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); // 定义输出的点云法线
	// K近邻确定方法，使用k个最近点，或者确定一个以r为半径的圆内的点集来确定都可以，两者选1即可
	normalEstimation.setKSearch(nPoints);// 使用当前点周围最近的10个点
	//normalEstimation.setRadiusSearch(0.03);            //对于每一个点都用半径为3cm的近邻搜索方式
	normalEstimation.compute(*normals);               //计算法线
	// 输出法线
	std::cout << "normals: " << normals->size() << ", " << "normals fields: " << pcl::getFieldsList(*normals) << std::endl;


}


void pc_operator::triangular(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals, 
	pcl::PolygonMesh &triangles)
{
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// 三角化
	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;   // 定义三角化对象
	
	// 设置三角化参数
	gp3.setSearchRadius(0.1);  //设置搜索时的半径，也就是KNN的球半径
	gp3.setMu(2.5);  //设置样本点搜索其近邻点的最远距离为2.5倍（典型值2.5-3），这样使得算法自适应点云密度的变化
	gp3.setMaximumNearestNeighbors(100);    //设置样本点最多可搜索的邻域个数，典型值是50-100

	gp3.setMinimumAngle(M_PI / 18); // 设置三角化后得到的三角形内角的最小的角度为10°
	gp3.setMaximumAngle(2 * M_PI / 3); // 设置三角化后得到的三角形内角的最大角度为120°

	gp3.setMaximumSurfaceAngle(M_PI / 4); // 设置某点法线方向偏离样本点法线的最大角度45°，如果超过，连接时不考虑该点
	gp3.setNormalConsistency(false);  //设置该参数为true保证法线朝向一致，设置为false的话不会进行法线一致性检查

	gp3.setInputCloud(cloud_with_normals);     //设置输入点云为有向点云
	gp3.setSearchMethod(tree2);   //设置搜索方式
	gp3.reconstruct(triangles);  //重建提取三角化

	
}

void pc_operator::triangular(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals, pcl::PolygonMesh &triangles)
{
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// 三角化
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   // 定义三角化对象

	// 设置三角化参数
	gp3.setSearchRadius(0.1);  //设置搜索时的半径，也就是KNN的球半径
	gp3.setMu(2.5);  //设置样本点搜索其近邻点的最远距离为2.5倍（典型值2.5-3），这样使得算法自适应点云密度的变化
	gp3.setMaximumNearestNeighbors(100);    //设置样本点最多可搜索的邻域个数，典型值是50-100

	gp3.setMinimumAngle(M_PI / 18); // 设置三角化后得到的三角形内角的最小的角度为10°
	gp3.setMaximumAngle(2 * M_PI / 3); // 设置三角化后得到的三角形内角的最大角度为120°

	gp3.setMaximumSurfaceAngle(M_PI / 4); // 设置某点法线方向偏离样本点法线的最大角度45°，如果超过，连接时不考虑该点
	gp3.setNormalConsistency(false);  //设置该参数为true保证法线朝向一致，设置为false的话不会进行法线一致性检查

	gp3.setInputCloud(cloud_with_normals);     //设置输入点云为有向点云
	gp3.setSearchMethod(tree2);   //设置搜索方式
	gp3.reconstruct(triangles);  //重建提取三角化

	std::cout << "三角形数：" <<triangles.polygons.size() << std::endl;
}


void pc_operator::poisson_reconstruction(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgb_cloud_with_normals, 
	pcl::PolygonMesh & mesh) 
{
	// 注意PCL poisson重建表面得到的是没有颜色的mesh
	std::cout << "begin poisson reconstruction" << endl;
	pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
	//poisson.setDegree(2);
	poisson.setDepth(8);
	poisson.setSolverDivide(6);
	poisson.setIsoDivide(6);

	poisson.setConfidence(false);
	poisson.setManifold(false);
	poisson.setOutputPolygons(false);

	poisson.setInputCloud(rgb_cloud_with_normals);
	poisson.reconstruct(mesh);

	std::cout << "finish poisson reconstruction" << endl;
}



void pc_operator::color_mesh(pcl::PolygonMesh mesh, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) 
{
	// 用cloud给mesh染色
	pcl::PointCloud<pcl::PointXYZRGB> cloud_color_mesh;
	
	pcl::fromPCLPointCloud2(mesh.cloud, cloud_color_mesh);

	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud(cloud);
	// K nearest neighbor search
	int K = 5;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	for (int i = 0; i < cloud_color_mesh.points.size(); ++i)
	{
		uint8_t r = 0;
		uint8_t g = 0;
		uint8_t b = 0;
		float dist = 0.0;
		int red = 0;
		int green = 0;
		int blue = 0;
		uint32_t rgb;

		if (kdtree.nearestKSearch(cloud_color_mesh.points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			for (int j = 0; j < pointIdxNKNSearch.size(); ++j)
			{

				r = cloud->points[pointIdxNKNSearch[j]].r;
				g = cloud->points[pointIdxNKNSearch[j]].g;
				b = cloud->points[pointIdxNKNSearch[j]].b;

				red += int(r);
				green += int(g);
				blue += int(b);
				dist += 1.0 / pointNKNSquaredDistance[j];		
			}
		}
		cloud_color_mesh.points[i].r = int(red / pointIdxNKNSearch.size() + 0.5);
		cloud_color_mesh.points[i].g = int(green / pointIdxNKNSearch.size() + 0.5);
		cloud_color_mesh.points[i].b = int(blue / pointIdxNKNSearch.size() + 0.5);


	}
	pcl::toPCLPointCloud2(cloud_color_mesh, mesh.cloud);
}




