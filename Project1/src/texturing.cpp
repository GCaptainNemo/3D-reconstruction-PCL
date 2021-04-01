#include "../include/texturing.h"

void texturing::color_mesh(pcl::PolygonMesh &mesh, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	// 用cloud给mesh染色
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

		cloud_color_mesh.points[i].r = int(red / pointIdxNKNSearch.size() + 0.5);   // PCL 版本1.9之前用float表示颜色，1.9之后用int
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

	//pcl::visualization::PCLVisualizer* viewer; // 显示PolygonMesh代码
	//viewer->addTextureMesh(*texture_mesh);
	//



	//texture_mesh->tex_coordinates.push_back(textureCoordinatesVector) ;
	/*texture_mesh->tex_coordinates.clear();
	texture_mesh->tex_coordinates = textureCoordinatesVector;
*/

//std::vector< Eigen::Vector2f > texcoord;
//texcoord.push_back(Eigen::Vector2f(0.0, 0.0));
//texcoord.push_back(Eigen::Vector2f(1.0, 0.0));
//texcoord.push_back(Eigen::Vector2f(1.0, 1.0));
//texcoord.push_back(Eigen::Vector2f(0.0, 1.0));
////texture_mesh->tex_coordinates.push_back(texcoord);
//pcl::TexMaterial tex_material;
//tex_material.tex_file = std::string("F:\\Develop\\examples\\111.jpg");
//tex_material.tex_name = std::string("111.jpg");
//texture_mesh->tex_materials.push_back(tex_material);
};


