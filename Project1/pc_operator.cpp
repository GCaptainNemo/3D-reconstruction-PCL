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
	// �²���
	pcl::VoxelGrid<pcl::PointXYZ> downSampled;  //�����˲�����
	downSampled.setInputCloud(cloud);            //������Ҫ���˵ĵ��Ƹ��˲�����
	downSampled.setLeafSize(voxel_size, voxel_size, voxel_size);  //�����˲�ʱ�������������Ϊ1cm��������
	downSampled.filter(*cloud_downSampled);           //ִ���˲������洢���
	std::cout << "downsampled point cloud = " << cloud_downSampled->size() << std::endl;
}


void pc_operator::upsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_upsampled) 
{
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	std::cout << "before filter pc = " << cloud->size() << std::endl;
	pcl::PointCloud<pcl::PointXYZ> filteredCloud;    //���MLS
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
	rs.setInputCloud(cloud);	    	// ��������������   
	rs.setSample(number);
	rs.filter(*cloud_randomsampled);  // �²����������cloud_out
	std::cout << "cloud_randomsampled = " << cloud_randomsampled->size() << std::endl;

}

void pc_operator::resampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed, const float &searchRadius)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr treeSampling(new pcl::search::KdTree<pcl::PointXYZ>);// �������������������KD-Tree
	pcl::PointCloud<pcl::PointXYZ> mls_point;    //���MLS
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls; // ������С����ʵ�ֵĶ���mls
	mls.setComputeNormals(true);               // ��С���˼������Ƿ���㷨��
	mls.setInputCloud(cloud);                  // ���ô��������
	// mls.setPolynomialOrder(2);              // ���2�׶���ʽ���
	mls.setPolynomialFit(true);                // ����Ϊfalse���� ���� smooth
	mls.setSearchMethod(treeSampling);         // ����KD-Tree��Ϊ��������
	mls.setSearchRadius(searchRadius);           // ��λm.����������ϵ�K���ڰ뾶
	mls.process(mls_point);                 //���
	// ����ز������
	cloud_smoothed = mls_point.makeShared();
	std::cout << "cloud_smoothed(resampled) =  " << cloud_smoothed->size() << std::endl;
}
void pc_operator::statistical_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter,
	const int & neighbour, const float & proportion)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(neighbour);   // �ھ���
	sor.setStddevMulThresh(proportion); // thresh = mean + proportion * std
	sor.filter(*cloud_filter);
	std::cout << "statistical filtered point: " << cloud_filter->size() << std::endl;
}




void pc_operator::estimate_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::Normal>::Ptr normals, const int &nPoints) 
{
	// ���߹���
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;             //�������߹��ƵĶ���
	normalEstimation.setInputCloud(cloud);                         //�������
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);// �������������������KD-Tree
	normalEstimation.setSearchMethod(tree);
	// pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); // ��������ĵ��Ʒ���
	// K����ȷ��������ʹ��k������㣬����ȷ��һ����rΪ�뾶��Բ�ڵĵ㼯��ȷ�������ԣ�����ѡ1����
	normalEstimation.setKSearch(nPoints);// ʹ�õ�ǰ����Χ�����10����
	//normalEstimation.setRadiusSearch(0.03);            //����ÿһ���㶼�ð뾶Ϊ3cm�Ľ���������ʽ
	normalEstimation.compute(*normals);               //���㷨��
	// �������
	std::cout << "normals: " << normals->size() << ", " << "normals fields: " << pcl::getFieldsList(*normals) << std::endl;


}


void pc_operator::triangular(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals, 
	pcl::PolygonMesh &triangles)
{
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// ���ǻ�
	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;   // �������ǻ�����
	
	// �������ǻ�����
	gp3.setSearchRadius(0.1);  //��������ʱ�İ뾶��Ҳ����KNN����뾶
	gp3.setMu(2.5);  //������������������ڵ����Զ����Ϊ2.5��������ֵ2.5-3��������ʹ���㷨����Ӧ�����ܶȵı仯
	gp3.setMaximumNearestNeighbors(100);    //���������������������������������ֵ��50-100

	gp3.setMinimumAngle(M_PI / 18); // �������ǻ���õ����������ڽǵ���С�ĽǶ�Ϊ10��
	gp3.setMaximumAngle(2 * M_PI / 3); // �������ǻ���õ����������ڽǵ����Ƕ�Ϊ120��

	gp3.setMaximumSurfaceAngle(M_PI / 4); // ����ĳ�㷨�߷���ƫ�������㷨�ߵ����Ƕ�45�㣬�������������ʱ�����Ǹõ�
	gp3.setNormalConsistency(false);  //���øò���Ϊtrue��֤���߳���һ�£�����Ϊfalse�Ļ�������з���һ���Լ��

	gp3.setInputCloud(cloud_with_normals);     //�����������Ϊ�������
	gp3.setSearchMethod(tree2);   //����������ʽ
	gp3.reconstruct(triangles);  //�ؽ���ȡ���ǻ�

	
}

void pc_operator::triangular(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals, pcl::PolygonMesh &triangles)
{
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// ���ǻ�
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   // �������ǻ�����

	// �������ǻ�����
	gp3.setSearchRadius(0.1);  //��������ʱ�İ뾶��Ҳ����KNN����뾶
	gp3.setMu(2.5);  //������������������ڵ����Զ����Ϊ2.5��������ֵ2.5-3��������ʹ���㷨����Ӧ�����ܶȵı仯
	gp3.setMaximumNearestNeighbors(100);    //���������������������������������ֵ��50-100

	gp3.setMinimumAngle(M_PI / 18); // �������ǻ���õ����������ڽǵ���С�ĽǶ�Ϊ10��
	gp3.setMaximumAngle(2 * M_PI / 3); // �������ǻ���õ����������ڽǵ����Ƕ�Ϊ120��

	gp3.setMaximumSurfaceAngle(M_PI / 4); // ����ĳ�㷨�߷���ƫ�������㷨�ߵ����Ƕ�45�㣬�������������ʱ�����Ǹõ�
	gp3.setNormalConsistency(false);  //���øò���Ϊtrue��֤���߳���һ�£�����Ϊfalse�Ļ�������з���һ���Լ��

	gp3.setInputCloud(cloud_with_normals);     //�����������Ϊ�������
	gp3.setSearchMethod(tree2);   //����������ʽ
	gp3.reconstruct(triangles);  //�ؽ���ȡ���ǻ�

	std::cout << "����������" <<triangles.polygons.size() << std::endl;
}


void pc_operator::poisson_reconstruction(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgb_cloud_with_normals, 
	pcl::PolygonMesh & mesh) 
{
	// ע��PCL poisson�ؽ�����õ�����û����ɫ��mesh
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
	// ��cloud��meshȾɫ
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




