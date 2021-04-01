#include "../include/pc_operator.h"
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
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>

#define PI 3.1415926535

void pc_operator::down_sample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downSampled, const float & voxel_size)
{
	// voxel -base downsample
	pcl::VoxelGrid<pcl::PointXYZ> downSampled;  

	// set input cloud
	downSampled.setInputCloud(cloud);            

	// set voxel_size * voxel_size * voxel_size  voxel cube
	downSampled.setLeafSize(voxel_size, voxel_size, voxel_size);  

	// filter
	downSampled.filter(*cloud_downSampled);          
	std::cout << "downsampled point cloud = " << cloud_downSampled->size() << std::endl;
}


void pc_operator::upsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_upsampled) 
{
	// set mls obj
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
	
	// set input cloud
	filter.setInputCloud(cloud);
	std::cout << "before filter pc = " << cloud->size() << std::endl;
	
	// set kd-tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
	
	// upsampling searchRadius = 0.1m
	filter.setSearchMethod(kdtree);
	filter.setSearchRadius(0.1);   
	pcl::PointCloud<pcl::PointXYZ> filteredCloud;

	// sample from local plane
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
	// create random sample object
	pcl::RandomSample<pcl::PointXYZ> rs;
	
	// set input point cloud
	rs.setInputCloud(cloud);

	// set random sampling number
	rs.setSample(number);

	// filter
	rs.filter(*cloud_randomsampled);  
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
	// create normal estimation object
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;

	// set input point cloud
	normalEstimation.setInputCloud(cloud);                         
	
	// set kd-tree object
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	
	// set search method
	normalEstimation.setSearchMethod(tree);
	
	// Knn search
	normalEstimation.setKSearch(nPoints);               
	
	// radius search 
	//normalEstimation.setRadiusSearch(0.1);            

	// calculate normal
	normalEstimation.compute(*normals);               
	std::cout << "normals: " << normals->size() << ", " << "normals fields: " << pcl::getFieldsList(*normals) << std::endl;
}


void pc_operator::triangular(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals, 
	pcl::PolygonMesh &triangles)
{
	// Set kd-tree
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr ke_tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	ke_tree->setInputCloud(cloud_with_normals);

	// Greedy projection triangulation object
	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;   
	
	// set parameters
	// Knn search radius（100mm）
	gp3.setSearchRadius(100); 
	
	//  Set the maximum distance of the sample point to search its nearest neighbor to be 2.5 times (typical value 2.5-3), 
	// so that the algorithm can adapt to the change of point cloud density
	gp3.setMu(2.5);  

	// maximum num nearest neighbour to search
	gp3.setMaximumNearestNeighbors(200);    

	// Set the minimum Angle of the triangulation triangle to 5°
	gp3.setMinimumAngle(PI / 36); 
	
	// Maximum angle of the triangulation triangle is 120°
	gp3.setMaximumAngle(2 * PI / 3); 

	// Set the maximum Angle that the normal direction of a point deviates from the normal of the sample point by 45°. 
	// If it exceeds, the point will not be considered during connection
	gp3.setMaximumSurfaceAngle(PI / 4); 

	// Set normal orientation consistency, if false will not check normal consistency
	gp3.setNormalConsistency(false);  

	// Set input pointcloud
	gp3.setInputCloud(cloud_with_normals);    

	// Set search method:kd tree
	gp3.setSearchMethod(ke_tree); 

	// reconstruct get mesh
	gp3.reconstruct(triangles);  
}

void pc_operator::decimateMesh(const float &reduction_factor, pcl::PolygonMeshPtr mesh_)
{
	if (reduction_factor <= 0 || reduction_factor > 1)
	{
		return;
	}
	else
	{
		std::cout << "before decimation vertex size = " << mesh_->cloud.width * mesh_->cloud.height << std::endl;
		std::cout << "before decimation polygon size = " << mesh_->polygons.size() << std::endl;

		// mesh decimator reduce vertex counts
		pcl::MeshQuadricDecimationVTK decimator;
		
		decimator.setInputMesh(mesh_);
		
		// reduce vertex counts
		decimator.setTargetReductionFactor(reduction_factor);
		pcl::PolygonMeshPtr decimatedMesh_ = pcl::PolygonMeshPtr(new pcl::PolygonMesh);
		
		std::cout << "start reduce vertex counts\n";
		decimator.process(*decimatedMesh_.get());
		
		mesh_ = decimatedMesh_;
		std::cout << "after decimation vertex size = " << mesh_->cloud.width * mesh_->cloud.height << std::endl;
		std::cout << "after decimation polygon size = " << mesh_->polygons.size() << std::endl;

	}
}

void pc_operator::poisson_reconstruction(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgb_cloud_with_normals, 
	pcl::PolygonMeshPtr mesh) 
{
	std::cout << "begin poisson reconstruction" << endl;
	pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
	//poisson.setDegree(2);
	
	int nPoints = rgb_cloud_with_normals->size();
	// Assume points are located (roughly) in a plane.
	double squareSide = std::sqrt(double(nPoints));

	// Calculate octree depth such that if points were equally distributed in
	// a quadratic plane, there would be at least 1 point per octree node.
	int depth = 0;
	while (std::pow<double>(2, depth) < squareSide / 2)
	{
		depth++;
	}
	std::cout << "tree_depth = " << depth << std::endl;

	poisson.setDepth(depth);
	/*poisson.setSolverDivide(6);
	poisson.setIsoDivide(6);*/

	
	poisson.setManifold(true);
	poisson.setSamplesPerNode(1);
	//poisson.setOutputPolygons(false);

	poisson.setInputCloud(rgb_cloud_with_normals);
	poisson.reconstruct(*mesh.get());

	std::cout << "finish poisson reconstruction" << std::endl;
	std::cout << "Vertex count:" << mesh->cloud.width << std::endl;
	std::cout << "Triangle count:" << mesh->polygons.size() << std::endl;
	
}



void pc_operator::pc2range_image(pcl::RangeImage& range_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_xyzrgb)
{
	//noiseLevel设置周围点对当前点深度值的影响：
			//noiseLevel = 0.05，深度距离值是通过查询点半径为 Scm 的圆内包含的点用来平均计算而得到的。
	float noise_level = 0.0;      //各种参数的设置
	float min_range = 0.0f;
	int border_size = 1;
	Eigen::Affine3f sensor_pose = Eigen::Affine3f::Identity();

	float angularResolution = pcl::deg2rad(0.03f);  // 0.03度转弧度
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
};


void pc_operator::bspline_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud) 
{
	pcl::on_nurbs::NurbsDataSurface data;
	PointCloud2Vector3d(object_cloud, data.interior);
	pcl::visualization::PCLVisualizer viewer("B样条曲面拟合点云数据");
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
		//std::cout << "click q key to quit the visualizer and continue！！" << endl;
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
		//std::cout << "click q key to quit the visualizer and continue！！" << endl;
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
	curve_params.accuracy = 0.5;
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
	//visualizeCurve(curve_fit.m_nurbs, fit.m_nurbs, viewer);

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


