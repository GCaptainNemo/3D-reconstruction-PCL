#include "../include/texturing.h"

texturing::texturing() 
{
	this->texture_mesh_ = pcl::TextureMeshPtr(new pcl::TextureMesh);
}

texturing::~texturing() 
{
}

void texturing::loadMesh(pcl::PolygonMeshPtr mesh)
{
	mesh->cloud = this->texture_mesh_->cloud;
	std::vector<pcl::Vertices> polygons;
	
	// Push faces from ply-mesh into TextureMesh
	polygons.resize(mesh->polygons.size());
	for (size_t i = 0; i < mesh->polygons.size(); ++i)
	{
		polygons[i] = mesh->polygons[i];
	}
	texture_mesh_->tex_polygons.push_back(polygons);
};
void texturing::loadCamera()
{
	std::ifstream bundleFile;
	std::string bundlePath = "../calib_par/bundle.rd.out";
	bundleFile.open(bundlePath.c_str(), std::ios_base::binary);
	// Check if file is open
	if (!bundleFile.is_open())
	{
		std::cout << "can't open .out file" << std::endl;
		return;
	}
	
	// A temporary storage for a line from the file.
	std::string dummyLine = "";
	std::getline(bundleFile, dummyLine);

	int nrCameras = 0;
	bundleFile >> nrCameras;
	bundleFile >> dummyLine;
	for (int i = 0; i < nrCameras; ++i)
	{
		double val;
		pcl::TextureMapping<pcl::PointXYZ>::Camera cam;
		Eigen::Affine3f transform;
		bundleFile >> val; //Read focal length from bundle file
		cam.focal_length = val;
		bundleFile >> val; //Read k1 from bundle file
		bundleFile >> val; //Read k2 from bundle file

		bundleFile >> val; transform(0, 0) = val; // Read rotation (0,0) from bundle file
		bundleFile >> val; transform(0, 1) = val; // Read rotation (0,1) from bundle file
		bundleFile >> val; transform(0, 2) = val; // Read rotation (0,2) from bundle file

		bundleFile >> val; transform(1, 0) = val; // Read rotation (1,0) from bundle file
		bundleFile >> val; transform(1, 1) = val; // Read rotation (1,1) from bundle file
		bundleFile >> val; transform(1, 2) = val; // Read rotation (1,2) from bundle file

		bundleFile >> val; transform(2, 0) = val; // Read rotation (2,0) from bundle file
		bundleFile >> val; transform(2, 1) = val; // Read rotation (2,1) from bundle file
		bundleFile >> val; transform(2, 2) = val; // Read rotation (2,2) from bundle file

		bundleFile >> val; transform(0, 3) = val; // Read translation (0,3) from bundle file
		bundleFile >> val; transform(1, 3) = val; // Read translation (1,3) from bundle file
		bundleFile >> val; transform(2, 3) = val; // Read translation (2,3) from bundle file

		transform(3, 0) = 0.0;
		transform(3, 1) = 0.0;
		transform(3, 2) = 0.0;
		transform(3, 3) = 1.0;

		transform = transform.inverse();

		// Column negation
		transform(0, 2) = -1.0*transform(0, 2);
		transform(1, 2) = -1.0*transform(1, 2);
		transform(2, 2) = -1.0*transform(2, 2);

		transform(0, 1) = -1.0*transform(0, 1);
		transform(1, 1) = -1.0*transform(1, 1);
		transform(2, 1) = -1.0*transform(2, 1);

		// Set values from bundle to current camera
		cam.pose = transform;

		cam.texture_file = "../../resources/livox_hikvision/test.png";
		

		// Read image to get full resolution size
		cv::Mat image = cv::imread(cam.texture_file);

		double imageWidth = static_cast<double>(image.cols);
		double textureWithWidth = static_cast<double>(2000.0);

		// Calculate scale factor to texture with textureWithSize
		double factor = textureWithWidth / imageWidth;
		if (factor > 1.0f)
		{
			factor = 1.0f;
		}

		// Update camera size and focal length
		cam.height = static_cast<int>(std::floor(factor*(static_cast<double>(image.rows))));
		cam.width = static_cast<int>(std::floor(factor*(static_cast<double>(image.cols))));
		cam.focal_length *= static_cast<double>(cam.width) / 1200.0;
		
		// Add camera
		cameras_.push_back(cam);
	}
}

void texturing::color_mesh(pcl::PolygonMesh &mesh, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	// color mesh
	printf("in color mesh");
	pcl::PointCloud<pcl::PointXYZRGB> cloud_color_mesh;
	pcl::fromPCLPointCloud2(mesh.cloud, cloud_color_mesh);
	printf("in color mesh");

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

		// Before PCL 1.9 use float to represent color£¬after 1.9 use int
		cloud_color_mesh.points[i].r = int(red / pointIdxNKNSearch.size() + 0.5);   
		cloud_color_mesh.points[i].g = int(green / pointIdxNKNSearch.size() + 0.5);
		cloud_color_mesh.points[i].b = int(blue / pointIdxNKNSearch.size() + 0.5);
	}
	pcl::toPCLPointCloud2(cloud_color_mesh, mesh.cloud);
	printf("finish color mesh!\n");
}

void texturing::texture_mesh(pcl::PolygonMesh &mesh, pcl::TextureMeshPtr texture_mesh, cv::Mat &transform_matrix, cv::Mat &image)
{
	int row_bound = image.rows;
	int column_bound = image.cols;

	// loadMesh
	texture_mesh->cloud = mesh.cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh.cloud, *meshCloud);
	texture_mesh->header = mesh.header;
	texture_mesh->tex_polygons.push_back(mesh.polygons);
	// Used to replace texture coordinates in mesh_.
	//std::vector<Eigen::Vector2f> textureCoordinatesVector;
	std::vector<std::vector<Eigen::Vector2f> > textureCoordinatesVector;
	std::cout << "face number = " << texture_mesh->tex_polygons[0].size();
	for (size_t faceIndex = 0; faceIndex < texture_mesh->tex_polygons[0].size(); ++faceIndex)
	{
		std::cout << "faceIndex";
		for (int i = 0; i < 3; ++i)
		{

			pcl::PointXYZ point = meshCloud->points[texture_mesh->tex_polygons[0][faceIndex].vertices[i]];
			double a_[4] = { point.x, point.y, point.z, 1.0 };
			cv::Mat pos(4, 1, CV_64F, a_);
			cv::Mat newpos(transform_matrix * pos);
			float x = (float)(newpos.at<double>(0, 0) / newpos.at<double>(2, 0));
			float y = (float)(newpos.at<double>(1, 0) / newpos.at<double>(2, 0));
			std::cout << "i = " << i;
			Eigen::Vector2f  uv;
			uv(0) = x / (float)column_bound;
			uv(1) = 1 - y / (float)row_bound;
			if (uv(0) < 0) {
				uv(0) = 0;
			}
			else if (uv(0) > 1) {
				uv(0) = 1;
			}
			if (uv(1) < 0) {
				uv(1) = 0;
			}
			else if (uv(1) > 1) {
				uv(1) = 1;
			}
			// Add uv coordinates to submesh
			// textureCoordinatesVector[0].push_back(uv);
			texture_mesh->tex_coordinates[0].push_back(uv);
		}

	}
	pcl::TexMaterial tex_material2;
	tex_material2.tex_file = std::string("..\\..\\resources\\livox_hikvision\\test.png");
	tex_material2.tex_name = std::string("test");
	texture_mesh->tex_materials.push_back(tex_material2);

	//pcl::visualization::PCLVisualizer* viewer; // ÏÔÊ¾PolygonMesh´úÂë
	//viewer->addTextureMesh(*texture_mesh_);
	//



	//texture_mesh_->tex_coordinates.push_back(textureCoordinatesVector) ;
	/*texture_mesh_->tex_coordinates.clear();
	texture_mesh_->tex_coordinates = textureCoordinatesVector;
*/

//std::vector< Eigen::Vector2f > texcoord;
//texcoord.push_back(Eigen::Vector2f(0.0, 0.0));
//texcoord.push_back(Eigen::Vector2f(1.0, 0.0));
//texcoord.push_back(Eigen::Vector2f(1.0, 1.0));
//texcoord.push_back(Eigen::Vector2f(0.0, 1.0));
////texture_mesh_->tex_coordinates.push_back(texcoord);
//pcl::TexMaterial tex_material;
//tex_material.tex_file = std::string("F:\\Develop\\examples\\111.jpg");
//tex_material.tex_name = std::string("111.jpg");
//texture_mesh_->tex_materials.push_back(tex_material);
};


