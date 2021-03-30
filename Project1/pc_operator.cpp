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
#include <pcl/surface/impl/organized_fast_mesh.hpp> 
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/io/png_io.h>


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
	normalEstimation.setKSearch(nPoints);               // ʹ�õ�ǰ����Χ�����10����
	//normalEstimation.setRadiusSearch(0.1);            //����ÿһ���㶼�ð뾶Ϊ10cm�Ľ���������ʽ
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
	gp3.setSearchRadius(1);  //��������ʱKNN����뾶
	gp3.setMu(2.5);  //������������������ڵ����Զ����Ϊ2.5��������ֵ2.5-3��������ʹ���㷨����Ӧ�����ܶȵı仯
	gp3.setMaximumNearestNeighbors(200);    //���������������������������������ֵ��50-100

	gp3.setMinimumAngle(M_PI / 36); // �������ǻ����������ڽ���С�Ƕ�Ϊ5��
	gp3.setMaximumAngle(2 * M_PI / 3); // �������ǻ���õ����������ڽǵ����Ƕ�Ϊ120��

	gp3.setMaximumSurfaceAngle(M_PI / 6); // ����ĳ�㷨�߷���ƫ�������㷨�ߵ����Ƕ�45�㣬�������������ʱ�����Ǹõ�
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
		cloud_color_mesh.points[i].r = int(red / pointIdxNKNSearch.size() + 0.5);   // PCL �汾1.9֮ǰ��float��ʾ��ɫ��1.9֮����int
		cloud_color_mesh.points[i].g = int(green / pointIdxNKNSearch.size() + 0.5);
		cloud_color_mesh.points[i].b = int(blue / pointIdxNKNSearch.size() + 0.5);
	}
	pcl::toPCLPointCloud2(cloud_color_mesh, mesh.cloud);
}

void pc_operator::pc2range_image(pcl::RangeImage& range_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_xyzrgb)
{
	//noiseLevel������Χ��Ե�ǰ�����ֵ��Ӱ�죺
			//noiseLevel = 0.05����Ⱦ���ֵ��ͨ����ѯ��뾶Ϊ Scm ��Բ�ڰ����ĵ�����ƽ��������õ��ġ�
	float noise_level = 0.0;      //���ֲ���������
	float min_range = 0.0f;
	int border_size = 1;
	Eigen::Affine3f sensor_pose = Eigen::Affine3f::Identity();

	float angularResolution = pcl::deg2rad(0.03f);  // 0.03��ת����
	float maxAngleWidth = pcl::deg2rad(81.7f);
	float maxAngleHeight = pcl::deg2rad(25.1f);
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
	range_image.createFromPointCloud(*points_xyzrgb,
		angularResolution,
		maxAngleWidth,
		maxAngleHeight,
		sensor_pose,
		coordinate_frame,
		noise_level,
		min_range,
		border_size);
	std::cout << "range_image finish" << "\n";
	std::cout << "range_image_size = " << range_image.size() << std::endl;
	

	//// save
	//float *ranges = range_image.getRangesArray();
	//unsigned char *rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(ranges, range_image.width, range_image.height);
	//pcl::io::saveRgbPNGFile("../result/rangeRGBImage.png", rgb_image, range_image.width, range_image.height);

};

void pc_operator::range_image_reconstruct(pcl::PolygonMesh &triangles, boost::shared_ptr<pcl::RangeImage> range_image_ptr)
{
	pcl::OrganizedFastMesh<pcl::PointWithRange>::Ptr tri(new pcl::OrganizedFastMesh<pcl::PointWithRange>);
	pcl::search::KdTree<pcl::PointWithRange>::Ptr tree(new pcl::search::KdTree<pcl::PointWithRange>);
	tree->setInputCloud(range_image_ptr);
	tri->setTrianglePixelSize(3);
	tri->setInputCloud(range_image_ptr);
	tri->setSearchMethod(tree);
	//tri->setTriangulationType(pcl::OrganizedFastMesh<pcl::PointWithRange>::TRIANGLE_RIGHT_CUT);
	tri->setTriangulationType(pcl::OrganizedFastMesh<pcl::PointWithRange>::TRIANGLE_ADAPTIVE_CUT);

	tri->reconstruct(triangles);
	// pcl::io::savePLYFileBinary("D:/pg_cpp/3D-reconstruction-PCL/result/imageRange.ply", triangles);
};


void pc_operator::bspline_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud) 
{
	pcl::on_nurbs::NurbsDataSurface data;
	PointCloud2Vector3d(object_cloud, data.interior);
	pcl::visualization::PCLVisualizer viewer("B����������ϵ�������");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.setSize(800, 600);
	pcl::visualization::PointCloudColorHandlerCustom<Point> handler(object_cloud, 0, 255, 0);
	viewer.addPointCloud<Point>(object_cloud, handler, "cloud_cylinder");

	unsigned order(3);
	unsigned mesh_resolution(128);
	ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(order, &data);
	pcl::on_nurbs::FittingSurface fit(&data, nurbs);
	// mesh for visualization
	pcl::PolygonMesh mesh;
	pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::Vertices> mesh_vertices;
	std::string mesh_id = "mesh_nurbs";
	pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(fit.m_nurbs, mesh, mesh_resolution);
	std::cout << "before refine" << endl;

	/*	viewer.addPolygonMesh(mesh, mesh_id);
		viewer.spin();*/

	unsigned refinement(4);
	bool two_dim = true;
	pcl::on_nurbs::FittingSurface::Parameter params;
	params.interior_smoothness = 0.2;
	params.interior_weight = 1.0;
	params.boundary_smoothness = 0.2;
	params.boundary_weight = 0.0;
	for (unsigned i = 0; i < refinement; i++)
	{
		fit.refine(0);
		if (two_dim)fit.refine(1);
		fit.assemble(params);
		fit.solve();
		pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
		//   viewer.updatePolygonMesh<pcl::PointXYZ> (mesh_cloud, mesh_vertices, mesh_id);
		std::cout << "refine: " << i << endl;
		//std::cout << "click q key to quit the visualizer and continue����" << endl;
		//viewer.spin();
	}
	std::cout << "refines: " << refinement << endl;

	/*viewer.updatePolygonMesh<pcl::PointXYZ>(mesh_cloud, mesh_vertices, mesh_id);
	viewer.spin();*/

	// surface fitting with final refinement level
	unsigned iterations(3);
	for (unsigned i = 0; i < iterations; i++)
	{
		fit.assemble(params);
		fit.solve();
		pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
		//   viewer.updatePolygonMesh<pcl::PointXYZ> (mesh_cloud, mesh_vertices, mesh_id);
		std::cout << "iteration: " << i << endl;
		//std::cout << "click q key to quit the visualizer and continue����" << endl;
		//viewer.spin();
	}
	std::cout << "surface fitting with iterations: " << iterations << endl;
	/*viewer.updatePolygonMesh<pcl::PointXYZ>(mesh_cloud, mesh_vertices, mesh_id);
	viewer.spin();*/

	// ############################################################################
		// fit B-spline curve

		// parameters
	pcl::on_nurbs::FittingCurve2dAPDM::FitParameter curve_params;
	curve_params.addCPsAccuracy = 5e-2;
	curve_params.addCPsIteration = 3;
	curve_params.maxCPs = 200;
	curve_params.accuracy = 1;
	curve_params.iterations = 100;

	curve_params.param.closest_point_resolution = 0;
	curve_params.param.closest_point_weight = 1.0;
	curve_params.param.closest_point_sigma2 = 0.1;
	curve_params.param.interior_sigma2 = 0.00001;
	curve_params.param.smooth_concavity = 1.0;
	curve_params.param.smoothness = 1.0;

	// initialisation (circular)
	printf("  curve fitting ...\n");
	pcl::on_nurbs::NurbsDataCurve2d curve_data;
	curve_data.interior = data.interior_param;
	curve_data.interior_weight_function.push_back(true);
	ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D(order, curve_data.interior);

	// curve fitting
	pcl::on_nurbs::FittingCurve2dASDM curve_fit(&curve_data, curve_nurbs);
	//curve_fit.setQuiet (false); // enable/disable debug output
	curve_fit.fitting(curve_params);
	visualizeCurve(curve_fit.m_nurbs, fit.m_nurbs, viewer);

	// ############################################################################
	// triangulation of trimmed surface

	printf("  triangulate trimmed surface ...\n");
	viewer.removePolygonMesh(mesh_id);
	pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh(fit.m_nurbs, curve_fit.m_nurbs, mesh,
		mesh_resolution);
	viewer.addPolygonMesh(mesh, mesh_id);
	viewer.spin();
	printf("finish \n");
};

void visualizeCurve(ON_NurbsCurve &curve, ON_NurbsSurface &surface, pcl::visualization::PCLVisualizer &viewer)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::on_nurbs::Triangulation::convertCurve2PointCloud(curve, surface, curve_cloud, 4);
	for (std::size_t i = 0; i < curve_cloud->size() - 1; i++)
	{
		pcl::PointXYZRGB &p1 = curve_cloud->at(i);
		pcl::PointXYZRGB &p2 = curve_cloud->at(i + 1);
		std::ostringstream os;
		os << "line" << i;
		viewer.removeShape(os.str());
		viewer.addLine<pcl::PointXYZRGB>(p1, p2, 1.0, 0.0, 0.0, os.str());
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cps(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < curve.CVCount(); i++)
	{
		ON_3dPoint p1;
		curve.GetCV(i, p1);

		double pnt[3];
		surface.Evaluate(p1.x, p1.y, 0, 3, pnt);
		pcl::PointXYZRGB p2;
		p2.x = float(pnt[0]);
		p2.y = float(pnt[1]);
		p2.z = float(pnt[2]);

		p2.r = 255;
		p2.g = 0;
		p2.b = 0;

		curve_cps->push_back(p2);
	}
	viewer.removePointCloud("cloud_cps");
	viewer.addPointCloud(curve_cps, "cloud_cps");
}

void PointCloud2Vector3d(pcl::PointCloud<Point>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data)
{
	for (unsigned i = 0; i < cloud->size(); i++)
	{
		Point &p = cloud->at(i);
		if (!pcl_isnan(p.x) && !pcl_isnan(p.y) && !pcl_isnan(p.z))
			data.push_back(Eigen::Vector3d(p.x, p.y, p.z));
	}
}




