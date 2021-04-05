#include "../include/texturing.h"

Texturing::Texturing() 
{
	this->texture_mesh_ = pcl::TextureMeshPtr(new pcl::TextureMesh);
	tTIA_ = std::vector<int>(0);
}

Texturing::~Texturing() 
{
}



void Texturing::load_mesh(pcl::PolygonMeshPtr mesh)
{
	this->texture_mesh_->cloud = mesh->cloud;
	std::vector<pcl::Vertices> polygons;
	
	// Push faces from ply-mesh into TextureMesh
	polygons.resize(mesh->polygons.size());
	for (size_t i = 0; i < mesh->polygons.size(); ++i)
	{
		polygons[i] = mesh->polygons[i];
	}
	texture_mesh_->tex_polygons.push_back(polygons);
	std::wcout << "there are " << mesh->polygons.size() << " polygons" << std::endl;
};

void Texturing::load_camera()
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
		std::cout << "focal_length = " << val << std::endl;
		cam.focal_length = val;

		// bundleFile store camera pose parameters.
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

		// transform = transform.inverse();

		// Column negation
		/*transform(0, 2) = -1.0*transform(0, 2);
		transform(1, 2) = -1.0*transform(1, 2);
		transform(2, 2) = -1.0*transform(2, 2);

		transform(0, 1) = -1.0*transform(0, 1);
		transform(1, 1) = -1.0*transform(1, 1);
		transform(2, 1) = -1.0*transform(2, 1);
*/
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
		float bundleResizedTo_ = 1200.0f;
		cam.focal_length *= static_cast<double>(cam.width) / bundleResizedTo_;
		
		// Add camera
		cameras_.push_back(cam);
	}
}

bool Texturing::is_face_projected(const pcl::TextureMapping<pcl::PointXYZ>::Camera &camera, const pcl::PointXYZ &p1, const pcl::PointXYZ &p2, const pcl::PointXYZ &p3, pcl::PointXY &proj1, pcl::PointXY &proj2, pcl::PointXY &proj3)
{
	return (Texturing::get_pixel_coordinates(p1, camera, proj1) && Texturing::get_pixel_coordinates(p2, camera, proj2) && Texturing::get_pixel_coordinates(p3, camera, proj3));
}

bool Texturing::get_pixel_coordinates(const pcl::PointXYZ &pt, const pcl::TextureMapping<pcl::PointXYZ>::Camera &cam, pcl::PointXY &UV_coordinates) 
{
	if (pt.z > 0)
	{
		// compute image center and dimension
		double sizeX = cam.width;
		double sizeY = cam.height;
		double cx, cy;
		if (cam.center_w > 0)
			cx = cam.center_w;
		else
			cx = sizeX / 2.0;
		if (cam.center_h > 0)
			cy = cam.center_h;
		else
			cy = sizeY / 2.0;

		double focal_x, focal_y;
		if (cam.focal_length_w > 0)
			focal_x = cam.focal_length_w;
		else
			focal_x = cam.focal_length;
		if (cam.focal_length_h > 0)
			focal_y = cam.focal_length_h;
		else
			focal_y = cam.focal_length;

		// project point on camera's image plane
		UV_coordinates.x = static_cast<float> ((focal_x * (pt.x / pt.z) + cx)); //horizontal
		UV_coordinates.y = static_cast<float> ((focal_y * (pt.y / pt.z) + cy)); //vertical

		// point is visible!
		if (UV_coordinates.x >= 15.0 && UV_coordinates.x <= (sizeX - 15.0) && UV_coordinates.y >= 15.0 && UV_coordinates.y <= (sizeY - 15.0))
		{
			return (true); // point was visible by the camera
		}
	}
	
	// point is NOT visible by the camera
	UV_coordinates.x = -1.0f;
	UV_coordinates.y = -1.0f;
	return (false); // point was not visible by the camera
};

bool Texturing::check_point_in_triangle(const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3, const pcl::PointXY &pt)
{
	// Compute vectors
	Eigen::Vector2d v0, v1, v2;
	v0(0) = p3.x - p1.x; v0(1) = p3.y - p1.y; // v0= C - A
	v1(0) = p2.x - p1.x; v1(1) = p2.y - p1.y; // v1= B - A
	v2(0) = pt.x - p1.x; v2(1) = pt.y - p1.y; // v2= P - A

	// Compute dot products
	double dot00 = v0.dot(v0); // dot00 = dot(v0, v0)
	double dot01 = v0.dot(v1); // dot01 = dot(v0, v1)
	double dot02 = v0.dot(v2); // dot02 = dot(v0, v2)
	double dot11 = v1.dot(v1); // dot11 = dot(v1, v1)
	double dot12 = v1.dot(v2); // dot12 = dot(v1, v2)

	// Compute barycentric coordinates
	double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
	double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

	// Check if point is in triangle
	return ((u >= 0) && (v >= 0) && (u + v < 1));
}

void Texturing::mesh_image_match()
{
	// Frustrum culling + Occlusion culling

	tTIA_.resize(texture_mesh_->tex_polygons[0].size());

	// Set all values in the triangleToImageAssignment vector to a default value (-1) if there are no optimal camera
	for (size_t i = 0; i < tTIA_.size(); ++i)
	{
		tTIA_[i] = -1;
	}

	// Vector containing information if the face has been given an optimal camera or not
	std::vector<bool> hasOptimalCamera = std::vector<bool>(texture_mesh_->tex_polygons[0].size());

	//Vector containing minimal distances to optimal camera
	std::vector<double> tTIA_distances(texture_mesh_->tex_polygons[0].size(), DBL_MAX);
	//Vector containing minimal angles of face to cameraplane normals
	std::vector<double> tTIA_angles(texture_mesh_->tex_polygons[0].size(), DBL_MAX);

	// Set default value that no face has an optimal camera
	for (size_t faceIndex = 0; faceIndex < hasOptimalCamera.size(); ++faceIndex)
	{
		hasOptimalCamera[faceIndex] = false;
	}

	// Convert vertices to pcl::PointXYZ cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::cout << "there are " << texture_mesh_->cloud.width * texture_mesh_->cloud.height << " texture mesh cloud\n";
	pcl::fromPCLPointCloud2(texture_mesh_->cloud, *meshCloud);
	std::cout << "there are " << meshCloud->size() << " meshcloud\n";

	// Create dummy point and UV-index for vertices not visible in any cameras
	pcl::PointXY nanPoint;
	nanPoint.x = std::numeric_limits<float>::quiet_NaN();
	nanPoint.y = std::numeric_limits<float>::quiet_NaN();
	pcl::texture_mapping::UvIndex uvNull;
	uvNull.idx_cloud = -1;
	uvNull.idx_face = -1;
	std::cout << "there are " << cameras_.size() << " cameras" << std::endl;
	for (size_t cameraIndex = 0; cameraIndex < cameras_.size(); ++cameraIndex)
	{
		// Move vertices in mesh into the camera coordinate system
		pcl::PointCloud<pcl::PointXYZ>::Ptr cameraCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*meshCloud, *cameraCloud, cameras_[cameraIndex].pose.inverse());
		
		// Cloud to contain points projected into current camera
		pcl::PointCloud<pcl::PointXY>::Ptr projections(new pcl::PointCloud<pcl::PointXY>);

		// Vector containing information if the polygon is visible in current camera
		std::vector<bool> visibility;
		visibility.resize(texture_mesh_->tex_polygons[0].size());

		// Vector for remembering the correspondence between uv-coordinates and faces
		std::vector<pcl::texture_mapping::UvIndex> indexUvToPoints;

		// Count the number of vertices inside the camera frustum
		int countInsideFrustum = 0;

		// Frustum culling for all faces
		std::cout << "start Frustrum culling\n";
		for (size_t faceIndex = 0; faceIndex < texture_mesh_->tex_polygons[0].size(); ++faceIndex)
		{
			// Variables for the face vertices as projections in the camera plane
			pcl::PointXY pixelPos0; pcl::PointXY pixelPos1; pcl::PointXY pixelPos2;
			// If the face is inside the camera frustum
			if (Texturing::is_face_projected(cameras_[cameraIndex],
				cameraCloud->points[texture_mesh_->tex_polygons[0][faceIndex].vertices[0]],
				cameraCloud->points[texture_mesh_->tex_polygons[0][faceIndex].vertices[1]],
				cameraCloud->points[texture_mesh_->tex_polygons[0][faceIndex].vertices[2]],
				pixelPos0, pixelPos1, pixelPos2))
			{
				
				// Add pixel positions in camera to projections
				projections->points.push_back((pixelPos0));
				projections->points.push_back((pixelPos1));
				projections->points.push_back((pixelPos2));

				// Remember corresponding face
				pcl::texture_mapping::UvIndex u1, u2, u3;
				u1.idx_cloud = texture_mesh_->tex_polygons[0][faceIndex].vertices[0];
				u2.idx_cloud = texture_mesh_->tex_polygons[0][faceIndex].vertices[1];
				u3.idx_cloud = texture_mesh_->tex_polygons[0][faceIndex].vertices[2];
				u1.idx_face = faceIndex; u2.idx_face = faceIndex; u3.idx_face = faceIndex;
				indexUvToPoints.push_back(u1);
				indexUvToPoints.push_back(u2);
				indexUvToPoints.push_back(u3);

				// Update visibility vector
				visibility[faceIndex] = true;

				// Update count
				++countInsideFrustum;
			}
			else
			{
				// If not visible set nanPoint and uvNull
				projections->points.push_back(nanPoint);
				projections->points.push_back(nanPoint);
				projections->points.push_back(nanPoint);
				indexUvToPoints.push_back(uvNull);
				indexUvToPoints.push_back(uvNull);
				indexUvToPoints.push_back(uvNull);

				// Update visibility vector
				visibility[faceIndex] = false;
			}
		}
		std::cout << "there are " << countInsideFrustum << " faces inside Frustrum\n";

		std::vector<double> local_tTIA_distances(texture_mesh_->tex_polygons[0].size(), DBL_MAX);
		std::vector<double> local_tTIA_angles(texture_mesh_->tex_polygons[0].size(), DBL_MAX);
		



		// If any faces are visible in the current camera, perform occlusion culling
		std::cout << "start occlusion culling\n";
		if (countInsideFrustum > 0)
		{
			// Set up acceleration structure
			pcl::KdTreeFLANN<pcl::PointXY> kdTree;
			
			// Input data is project to image coordinate points(UV). Arrange by facet
			kdTree.setInputCloud(projections);

			// Loop through all faces and perform occlusion culling for faces inside frustum
			for (size_t faceIndex = 0; faceIndex < texture_mesh_->tex_polygons[0].size(); ++faceIndex)
			{
				if (visibility[faceIndex])
				{
					// Vectors to store output from radiusSearch in acceleration structure
					std::vector<int> neighbors;  // The index of neighbours. 
					std::vector<float> neighborsSquaredDistance;  // The distance of neighbours.

					// Variables for the vertices in face as projections in the camera plane
					pcl::PointXY pixelPos0; pcl::PointXY pixelPos1; pcl::PointXY pixelPos2;
					
					if (Texturing::is_face_projected(cameras_[cameraIndex],
						cameraCloud->points[texture_mesh_->tex_polygons[0][faceIndex].vertices[0]],
						cameraCloud->points[texture_mesh_->tex_polygons[0][faceIndex].vertices[1]],
						cameraCloud->points[texture_mesh_->tex_polygons[0][faceIndex].vertices[2]],
						pixelPos0, pixelPos1, pixelPos2))
					{
						// Variables for a radius circumscribing the polygon in the camera plane and the center of the polygon
						double radius; pcl::PointXY center;

						// Get values for radius and center
						Texturing::get_triangle_centroid(pixelPos0, pixelPos1, pixelPos2, center, radius);

						// Perform radius search in the acceleration structure
						int radiusSearch = kdTree.radiusSearch(center, radius, neighbors, neighborsSquaredDistance);

						// Extract distances for all vertices for face to camera(for Camera the forward coordinate is z)
						double d0 = cameraCloud->points[texture_mesh_->tex_polygons[0][faceIndex].vertices[0]].z;
						double d1 = cameraCloud->points[texture_mesh_->tex_polygons[0][faceIndex].vertices[1]].z;
						double d2 = cameraCloud->points[texture_mesh_->tex_polygons[0][faceIndex].vertices[2]].z;

						// Calculate largest distance and store in distance variable
						double distance = std::max(d0, std::max(d1, d2));

						//Get points
						pcl::PointXYZ p0 = cameraCloud->points[texture_mesh_->tex_polygons[0][faceIndex].vertices[0]];
						pcl::PointXYZ p1 = cameraCloud->points[texture_mesh_->tex_polygons[0][faceIndex].vertices[1]];
						pcl::PointXYZ p2 = cameraCloud->points[texture_mesh_->tex_polygons[0][faceIndex].vertices[2]];
						//Calculate face normal

						pcl::PointXYZ diff0;
						pcl::PointXYZ diff1;
						diff0.x = p1.x - p0.x;
						diff0.y = p1.y - p0.y;
						diff0.z = p1.z - p0.z;
						diff1.x = p2.x - p0.x;
						diff1.y = p2.y - p0.y;
						diff1.z = p2.z - p0.z;

						// cross product to calculate the face's normal vector
						pcl::PointXYZ normal;
						normal.x = diff0.y * diff1.z - diff0.z * diff1.y;
						normal.y = -(diff0.x * diff1.z - diff0.z * diff1.x);
						normal.z = diff0.x * diff1.y - diff0.y * diff1.x;
						double norm = sqrt(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
						//Angle of face to camera（Camera coordinate system）
						double cos = normal.z / norm;

						//Save distance of faceIndex to current camera, maximize distances of three vertices
						local_tTIA_distances[faceIndex] = distance;

						//Save angle of faceIndex to current camera
						if (normal.z >= 0)
							local_tTIA_angles[faceIndex] = sqrt(1.0 - cos * cos);
						
						// If other projections are found inside the radius
						if (radiusSearch > 0)
						{

							// Compare distance to all neighbors inside radius
							for (size_t i = 0; i < neighbors.size(); ++i)
							{
								// Distance variable from neighbor to camera
								double neighborDistance = cameraCloud->points[indexUvToPoints[neighbors[i]].idx_cloud].z;

								// If the neighbor has a greater distance to the camera and is inside face polygon set it as not visible
								if (distance < neighborDistance)
								{
									if (Texturing::check_point_in_triangle(pixelPos0, pixelPos1, pixelPos2, projections->points[neighbors[i]]))
									{
										// Update visibility for neighbors
										visibility[indexUvToPoints[neighbors[i]].idx_face] = false;
									}
								}
							}
						}
					}
				}
			}
		}

		// Number of polygons that add current camera as the optimal camera
		int count = 0;

		// Update optimal cameras for faces visible in current camera
		for (size_t faceIndex = 0; faceIndex < visibility.size(); ++faceIndex)
		{
			if (visibility[faceIndex])
			{
				if (local_tTIA_distances[faceIndex] < tTIA_distances[faceIndex] && local_tTIA_angles[faceIndex] < tTIA_angles[faceIndex])
				{
					tTIA_angles[faceIndex] = local_tTIA_angles[faceIndex];
					tTIA_distances[faceIndex] = local_tTIA_distances[faceIndex];
					hasOptimalCamera[faceIndex] = true;
					tTIA_[faceIndex] = cameraIndex;
					++count;
				}
			}
		}
		std::cout << "count = " << count << std::endl;
	}
}

void Texturing::get_triangle_centroid(const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3, pcl::PointXY &circumcenter, double &radius)
{
	// compute centroid's coordinates (translate back to original coordinates)
	circumcenter.x = static_cast<float> (p1.x + p2.x + p3.x) / 3;
	circumcenter.y = static_cast<float> (p1.y + p2.y + p3.y) / 3;
	double r1 = (circumcenter.x - p1.x) * (circumcenter.x - p1.x) + (circumcenter.y - p1.y) * (circumcenter.y - p1.y);
	double r2 = (circumcenter.x - p2.x) * (circumcenter.x - p2.x) + (circumcenter.y - p2.y) * (circumcenter.y - p2.y);
	double r3 = (circumcenter.x - p3.x) * (circumcenter.x - p3.x) + (circumcenter.y - p3.y) * (circumcenter.y - p3.y);

	// radius to avoid accuracy loss
	radius = std::sqrt(std::max(r1, std::max(r2, r3)));
}

void Texturing::color_mesh(pcl::PolygonMesh &mesh, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
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

void Texturing::texture_mesh(pcl::PolygonMesh &mesh, pcl::TextureMeshPtr texture_mesh, cv::Mat &transform_matrix, cv::Mat &image)
{
	int row_bound = image.rows;
	int column_bound = image.cols;

	// load_mesh
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


