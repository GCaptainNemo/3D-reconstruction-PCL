#include "../include/texturing.h"
#include <algorithm>


Texturing::Texturing() 
{
	this->texture_mesh_ = pcl::TextureMeshPtr(new pcl::TextureMesh);
	tTIA_ = std::vector<int>(0);
	patches_ = std::vector<Patch>(0);
	textureResolution_ = 4096.0;
	textureWithSize_ = 2000.0;
	padding_ = 15.0;
	outputFolder_ = "../../linshi/";

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

void Texturing::calculate_patches()
{
	// Convert vertices to pcl::PointXYZ cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(texture_mesh_->cloud, *meshCloud);

	// Reserve size for patches_, tTIA_ is optimal camera index of each face.
	patches_.reserve(tTIA_.size());

	// Vector containing vector with indicies to faces visible in corresponding camera index
	std::vector<std::vector<int> > optFaceCameraVector = std::vector<std::vector<int> >(cameras_.size());

	// Counter variables for visible and occluded faces
	int countVis = 0;
	int countOcc = 0;

	Patch nonVisibleFaces;
	nonVisibleFaces.optimalCameraIndex_ = -1;
	nonVisibleFaces.materialIndex_ = -1;
	nonVisibleFaces.placed_ = true;

	// Setup vector containing vectors with all faces correspondning to camera according to triangleToImageAssignment vector
	for (size_t i = 0; i < tTIA_.size(); ++i)
	{
		if (tTIA_[i] > -1)
		{
			// If face has an optimal camera add to optFaceCameraVector and update counter for visible faces
			countVis++;
			optFaceCameraVector[tTIA_[i]].push_back(i);
		}
		else
		{
			// Add non visible face to patch nonVisibleFaces
			nonVisibleFaces.faces_.push_back(i);

			// Update counter for occluded faces
			countOcc++;
		}
	}

	// Loop through all cameras
	for (size_t cameraIndex = 0; cameraIndex < cameras_.size(); ++cameraIndex)
	{
		// Transform mesh into camera coordinate system
		pcl::PointCloud<pcl::PointXYZ>::Ptr cameraCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*meshCloud, *cameraCloud, cameras_[cameraIndex].pose.inverse());

		// While faces visible in camera remains to be assigned to a patch
		while (0 < optFaceCameraVector[cameraIndex].size())
		{
			// Create current patch
			Patch patch;

			// Vector containing faces to check connectivity with current patch
			std::vector<size_t> addedFaces = std::vector<size_t>(0);

			// Add last face in optFaceCameraVector to faces to check connectivity and add it to the current patch
			addedFaces.push_back(optFaceCameraVector[cameraIndex].back());

			// Add first face to patch
			patch.faces_.push_back(optFaceCameraVector[cameraIndex].back());

			// Remove face from optFaceCameraVector
			optFaceCameraVector[cameraIndex].pop_back();

			// Declare uv-coordinates for face
			pcl::PointXY uvCoord1; pcl::PointXY uvCoord2; pcl::PointXY uvCoord3;

			// Calculate uv-coordinates for face in camera
			if (Texturing::is_face_projected(cameras_[cameraIndex],
				cameraCloud->points[texture_mesh_->tex_polygons[0][addedFaces.back()].vertices[0]],
				cameraCloud->points[texture_mesh_->tex_polygons[0][addedFaces.back()].vertices[1]],
				cameraCloud->points[texture_mesh_->tex_polygons[0][addedFaces.back()].vertices[2]],
				uvCoord1, uvCoord2, uvCoord3))
			{
				// Set minimum and maximum uv-coordinate value for patch
				patch.minu_ = std::min(uvCoord1.x, std::min(uvCoord2.x, uvCoord3.x));
				patch.minv_ = std::min(uvCoord1.y, std::min(uvCoord2.y, uvCoord3.y));
				patch.maxu_ = std::max(uvCoord1.x, std::max(uvCoord2.x, uvCoord3.x));
				patch.maxv_ = std::max(uvCoord1.y, std::max(uvCoord2.y, uvCoord3.y));

				while (0 < addedFaces.size())
				{
					// Set face to check neighbors
					size_t patchFaceIndex = addedFaces.back();

					// Remove patchFaceIndex from addedFaces
					addedFaces.pop_back();

					// Check against all remaining faces with the same optimal camera
					for (size_t i = 0; i < optFaceCameraVector[cameraIndex].size(); ++i)
					{
						size_t modelFaceIndex = optFaceCameraVector[cameraIndex][i];

						// Don't check against self
						if (modelFaceIndex != patchFaceIndex)
						{
							// Store indices for vertices of both faces
							size_t face0v0 = texture_mesh_->tex_polygons[0][modelFaceIndex].vertices[0];
							size_t face0v1 = texture_mesh_->tex_polygons[0][modelFaceIndex].vertices[1];
							size_t face0v2 = texture_mesh_->tex_polygons[0][modelFaceIndex].vertices[2];
							size_t face1v0 = texture_mesh_->tex_polygons[0][patchFaceIndex].vertices[0];
							size_t face1v1 = texture_mesh_->tex_polygons[0][patchFaceIndex].vertices[1];
							size_t face1v2 = texture_mesh_->tex_polygons[0][patchFaceIndex].vertices[2];

							// Count the number of shared vertices
							size_t nShared = 0;
							nShared += (face0v0 == face1v0 ? 1 : 0) + (face0v0 == face1v1 ? 1 : 0) + (face0v0 == face1v2 ? 1 : 0);
							nShared += (face0v1 == face1v0 ? 1 : 0) + (face0v1 == face1v1 ? 1 : 0) + (face0v1 == face1v2 ? 1 : 0);
							nShared += (face0v2 == face1v0 ? 1 : 0) + (face0v2 == face1v1 ? 1 : 0) + (face0v2 == face1v2 ? 1 : 0);

							// If sharing a vertex
							if (nShared > 0)
							{
								// Declare uv-coordinates for face
								pcl::PointXY uv1; pcl::PointXY uv2; pcl::PointXY uv3;

								// Calculate uv-coordinates for face in camera
								Texturing::is_face_projected(cameras_[cameraIndex],
									cameraCloud->points[texture_mesh_->tex_polygons[0][modelFaceIndex].vertices[0]],
									cameraCloud->points[texture_mesh_->tex_polygons[0][modelFaceIndex].vertices[1]],
									cameraCloud->points[texture_mesh_->tex_polygons[0][modelFaceIndex].vertices[2]],
									uv1, uv2, uv3);
								
								// Update minimum and maximum uv-coordinate value for patch
								patch.minu_ = std::min(patch.minu_, std::min(uv1.x, std::min(uv2.x, uv3.x)));
								patch.minv_ = std::min(patch.minv_, std::min(uv1.y, std::min(uv2.y, uv3.y)));
								patch.maxu_ = std::max(patch.maxu_, std::max(uv1.x, std::max(uv2.x, uv3.x)));
								patch.maxv_ = std::max(patch.maxv_, std::max(uv1.y, std::max(uv2.y, uv3.y)));

								// Add modelFaceIndex to patch
								patch.faces_.push_back(modelFaceIndex);

								// Add modelFaceIndex from faces to check for neighbors with same optimal camera
								addedFaces.push_back(modelFaceIndex);

								// Remove modelFaceIndex from optFaceCameraVector to exclude it from comming iterations
								optFaceCameraVector[cameraIndex].erase(optFaceCameraVector[cameraIndex].begin() + i);
							}
						}
					}
				}
			}

			// Set optimal camera for patch
			patch.optimalCameraIndex_ = static_cast<int>(cameraIndex);

			// Add patch to patches_ vector
			patches_.push_back(patch);
		}
	}
	patches_.push_back(nonVisibleFaces);
}

Coords Texturing::recursiveFindCoords(Node &n, float w, float h)
{
	// Coordinates to return and place patch
	Coords c;

	if (NULL != n.lft_)
	{
		c = recursiveFindCoords(*(n.lft_), w, h);
		if (c.success_)
		{
			return c;
		}
		else
		{
			return recursiveFindCoords(*(n.rgt_), w, h);
		}
	}
	else
	{
		// If the patch is to large or occupied return success false for coord
		if (n.used_ || w > n.width_ || h > n.height_)
		{
			c.success_ = false;
			return c;
		}

		// If the patch matches perfectly, store it
		if (w == n.width_ && h == n.height_)
		{
			n.used_ = true;
			c.r_ = n.r_;
			c.c_ = n.c_;
			c.success_ = true;

			return c;
		}

		// Initialize children for node
		n.lft_ = new Node(n);
		n.rgt_ = new Node(n);

		n.rgt_->used_ = false;
		n.lft_->used_ = false;
		n.rgt_->rgt_ = NULL;
		n.rgt_->lft_ = NULL;
		n.lft_->rgt_ = NULL;
		n.lft_->lft_ = NULL;

		// Check how to adjust free space
		if (n.width_ - w > n.height_ - h)
		{
			n.lft_->width_ = w;
			n.rgt_->c_ = n.c_ + w;
			n.rgt_->width_ = n.width_ - w;
		}
		else
		{
			n.lft_->height_ = h;
			n.rgt_->r_ = n.r_ + h;
			n.rgt_->height_ = n.height_ - h;
		}

		return recursiveFindCoords(*(n.lft_), w, h);
	}
}

void Texturing::sortPatches()
{
	// Bool to set true when done
	bool done = false;

	// Material index
	int materialIndex = 0;

	// Number of patches left from last loop
	size_t countLeftLastIteration = 0;

	while (!done)
	{
		// Create container for current material
		Node root;
		root.width_ = textureResolution_;
		root.height_ = textureResolution_;

		// Set done to true
		done = true;

		// Number of patches that did not fit in current material
		size_t countNotPlacedPatches = 0;

		// Number of patches placed in current material
		size_t placed = 0;

		for (size_t patchIndex = 0; patchIndex < patches_.size(); ++patchIndex)
		{
			if (!patches_[patchIndex].placed_)
			{
				// Calculate dimensions of the patch
				float w = patches_[patchIndex].maxu_ - patches_[patchIndex].minu_ + 2 * padding_;
				float h = patches_[patchIndex].maxv_ - patches_[patchIndex].minv_ + 2 * padding_;

				// Try to place patch in root container for this material
				if (w > 0.0 && h > 0.0)
				{
					patches_[patchIndex].c_ = recursiveFindCoords(root, w, h);
				}

				if (!patches_[patchIndex].c_.success_)
				{
					++countNotPlacedPatches;
					done = false;
				}
				else
				{
					// Set patch material as current material
					patches_[patchIndex].materialIndex_ = materialIndex;

					// Set patch as placed
					patches_[patchIndex].placed_ = true;

					// Update number of patches placed in current material
					placed++;

					// Update patch with padding_
					//patches_[patchIndex].c_.c_ += padding_;
					//patches_[patchIndex].c_.r_ += padding_;
					patches_[patchIndex].minu_ -= padding_;
					patches_[patchIndex].minv_ -= padding_;
					patches_[patchIndex].maxu_ = std::min((patches_[patchIndex].maxu_ + padding_), textureResolution_);
					patches_[patchIndex].maxv_ = std::min((patches_[patchIndex].maxv_ + padding_), textureResolution_);
				}
			}
		}

		// Update material index
		++materialIndex;

		if (countLeftLastIteration == countNotPlacedPatches && countNotPlacedPatches != 0)
		{
			done = true;
		}
		countLeftLastIteration = countNotPlacedPatches;
	}

	// Set number of textures
	nrTextures_ = materialIndex;
}

void Texturing::create_textures()
{
	// Convert vertices to pcl::PointXYZ cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(this->texture_mesh_->cloud, *meshCloud);

	// Container for faces according to submesh. Used to replace faces in mesh_.
	std::vector<std::vector<pcl::Vertices> > face_vector = std::vector<std::vector<pcl::Vertices> >(nrTextures_ + 1);

	// Container for texture coordinates according to submesh. Used to replace texture coordinates in mesh_.
	std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> texture_coordinates_vector = 
		std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>>(nrTextures_ + 1);
//	std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> texture_coordinates_vector = std::vector<std::vector<Eigen::Vector2f> >(nrTextures_ + 1);


	// Container for materials according to submesh. Used to replace materials in mesh_.
	std::vector<pcl::TexMaterial> material_vector = std::vector<pcl::TexMaterial>(nrTextures_ + 1);

	// Setup model according to patches placement
	for (int textureIndex = 0; textureIndex < nrTextures_; ++textureIndex)
	{
		for (size_t patchIndex = 0; patchIndex < patches_.size(); ++patchIndex)
		{
			// If patch is placed in current mesh add all containing faces to that submesh
			if (patches_[patchIndex].materialIndex_ == textureIndex)
			{
				// Transform mesh into camera
				pcl::PointCloud<pcl::PointXYZ>::Ptr cameraCloud(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::transformPointCloud(*meshCloud, *cameraCloud, cameras_[patches_[patchIndex].optimalCameraIndex_].pose.inverse());

				// Loop through all faces in patch
				for (size_t faceIndex = 0; faceIndex < patches_[patchIndex].faces_.size(); ++faceIndex)
				{
					// Setup global face index in mesh_
					size_t globalFaceIndex = patches_[patchIndex].faces_[faceIndex];

					// Add current face to current submesh
					face_vector[textureIndex].push_back(texture_mesh_->tex_polygons[0][globalFaceIndex]);

					// Pixel positions
					pcl::PointXY pixelPos0; pcl::PointXY pixelPos1; pcl::PointXY pixelPos2;

					// Get pixel positions in corresponding camera for the vertices of the face
					Texturing::get_pixel_coordinates(cameraCloud->points[texture_mesh_->tex_polygons[0][globalFaceIndex].vertices[0]], cameras_[patches_[patchIndex].optimalCameraIndex_], pixelPos0);
					Texturing::get_pixel_coordinates(cameraCloud->points[texture_mesh_->tex_polygons[0][globalFaceIndex].vertices[1]], cameras_[patches_[patchIndex].optimalCameraIndex_], pixelPos1);
					Texturing::get_pixel_coordinates(cameraCloud->points[texture_mesh_->tex_polygons[0][globalFaceIndex].vertices[2]], cameras_[patches_[patchIndex].optimalCameraIndex_], pixelPos2);

					// Shorthands for patch variables
					float c = patches_[patchIndex].c_.c_ + padding_;
					float r = patches_[patchIndex].c_.r_ + padding_;
					float minu = patches_[patchIndex].minu_ + padding_;
					float minv = patches_[patchIndex].minv_ + padding_;

					// Declare uv coordinates
					Eigen::Vector2f uv1, uv2, uv3;

					// Set uv coordinates according to patch
					uv1(0) = (pixelPos0.x - minu + c) / textureResolution_;
					uv1(1) = 1.0f - (pixelPos0.y - minv + r) / textureResolution_;

					uv2(0) = (pixelPos1.x - minu + c) / textureResolution_;
					uv2(1) = 1.0f - (pixelPos1.y - minv + r) / textureResolution_;

					uv3(0) = (pixelPos2.x - minu + c) / textureResolution_;
					uv3(1) = 1.0f - (pixelPos2.y - minv + r) / textureResolution_;

					// Add uv coordinates to submesh
					texture_coordinates_vector[textureIndex].push_back(uv1);
					texture_coordinates_vector[textureIndex].push_back(uv2);
					texture_coordinates_vector[textureIndex].push_back(uv3);
				}
			}
		}

		// Declare material and setup default values
		pcl::TexMaterial mesh_material;
		mesh_material.tex_Ka.r = 0.0f; mesh_material.tex_Ka.g = 0.0f; mesh_material.tex_Ka.b = 0.0f;
		mesh_material.tex_Kd.r = 0.0f; mesh_material.tex_Kd.g = 0.0f; mesh_material.tex_Kd.b = 0.0f;
		mesh_material.tex_Ks.r = 0.0f; mesh_material.tex_Ks.g = 0.0f; mesh_material.tex_Ks.b = 0.0f;
		mesh_material.tex_d = 1.0f; mesh_material.tex_Ns = 200.0f; mesh_material.tex_illum = 2;
		std::stringstream tex_name;
		tex_name << "texture_" << textureIndex;
		tex_name >> mesh_material.tex_name;
		mesh_material.tex_file = mesh_material.tex_name + ".jpg";
		material_vector[textureIndex] = mesh_material;
	}

	// Add non visible patches to submesh
	for (size_t patchIndex = 0; patchIndex < patches_.size(); ++patchIndex)
	{
		// If the patch does not have an optimal camera
		if (patches_[patchIndex].optimalCameraIndex_ == -1)
		{
			// Add all faces and set uv coordinates
			for (size_t faceIndex = 0; faceIndex < patches_[patchIndex].faces_.size(); ++faceIndex)
			{
				// Setup global face index in mesh_
				size_t globalFaceIndex = patches_[patchIndex].faces_[faceIndex];

				// Add current face to current submesh
				face_vector[nrTextures_].push_back(texture_mesh_->tex_polygons[0][globalFaceIndex]);

				// Declare uv coordinates
				Eigen::Vector2f uv1, uv2, uv3;

				// Set uv coordinates according to patch
				uv1(0) = 0.25f;//(pixelPos0.x - minu + c)/textureResolution_;
				uv1(1) = 0.25f;//1.0f - (pixelPos0.y - minv + r)/textureResolution_;

				uv2(0) = 0.25f;//(pixelPos1.x - minu + c)/textureResolution_;
				uv2(1) = 0.75f;//1.0f - (pixelPos1.y - minv + r)/textureResolution_;

				uv3(0) = 0.75f;//(pixelPos2.x - minu + c)/textureResolution_;
				uv3(1) = 0.75f;//1.0f - (pixelPos2.y - minv + r)/textureResolution_;

				// Add uv coordinates to submesh
				texture_coordinates_vector[nrTextures_].push_back(uv1);
				texture_coordinates_vector[nrTextures_].push_back(uv2);
				texture_coordinates_vector[nrTextures_].push_back(uv3);
			}
		}
	}

	// Declare material and setup default values for nonVisibileFaces submesh
	pcl::TexMaterial mesh_material;
	mesh_material.tex_Ka.r = 0.0f; mesh_material.tex_Ka.g = 0.0f; mesh_material.tex_Ka.b = 0.0f;
	mesh_material.tex_Kd.r = 0.0f; mesh_material.tex_Kd.g = 0.0f; mesh_material.tex_Kd.b = 0.0f;
	mesh_material.tex_Ks.r = 0.0f; mesh_material.tex_Ks.g = 0.0f; mesh_material.tex_Ks.b = 0.0f;
	mesh_material.tex_d = 1.0f; mesh_material.tex_Ns = 200.0f; mesh_material.tex_illum = 2;
	std::stringstream tex_name;
	tex_name << "non_visible_faces_texture";
	tex_name >> mesh_material.tex_name;
	mesh_material.tex_file = mesh_material.tex_name + ".jpg";
	material_vector[nrTextures_] = mesh_material;

	// Replace polygons, texture coordinates and materials in mesh_
	texture_mesh_->tex_polygons = face_vector;
	texture_mesh_->tex_coordinates = texture_coordinates_vector;
	texture_mesh_->tex_materials = material_vector;

	// Containers for image and the resized image used for texturing
	cv::Mat image;
	cv::Mat resizedImage;

	for (int textureIndex = 0; textureIndex < nrTextures_; ++textureIndex)
	{
		// Current texture for corresponding material
		cv::Mat texture = cv::Mat::zeros(textureResolution_, textureResolution_, CV_8UC3);

		for (int cameraIndex = 0; cameraIndex < static_cast<int>(cameras_.size()); ++cameraIndex)
		{
			// Load image for current camera
			image = cv::imread(cameras_[cameraIndex].texture_file, 1);

			// Calculate the resize factor to texturize with textureWithSize_
			double resizeFactor = textureWithSize_ / static_cast<double>(image.cols);

			if (resizeFactor > 1.0f)
			{
				resizeFactor = 1.0f;
			}

			// Resize image to the resolution used to texture with
			// CV_INTER_AREA used to shrink image. 
			cv::resize(image, resizedImage, cv::Size(), resizeFactor, resizeFactor, CV_INTER_AREA);

			// Loop through all patches
			for (size_t patchIndex = 0; patchIndex < patches_.size(); ++patchIndex)
			{
				// If the patch has the current camera as optimal camera
				if (patches_[patchIndex].materialIndex_ == textureIndex && patches_[patchIndex].optimalCameraIndex_ == cameraIndex)
				{
					// Pixel coordinates to extract image information from
					int extractX = static_cast<int>(floor(patches_[patchIndex].minu_));// + padding_);
					int extractY = static_cast<int>(floor(patches_[patchIndex].minv_));// + padding_);

					// Pixel coordinates to insert the image information to
					int insertX = static_cast<int>(floor(patches_[patchIndex].c_.c_));
					int insertY = static_cast<int>(floor(patches_[patchIndex].c_.r_));

					// The size of the image information to use
					int width = static_cast<int>(floor(patches_[patchIndex].maxu_)) - extractX - 1;
					int height = static_cast<int>(floor(patches_[patchIndex].maxv_)) - extractY - 1;

					// Get image information and add to texture
					cv::Mat src = resizedImage(cv::Rect(extractX, extractY, width, height));
					cv::Mat dst = texture(cv::Rect(insertX, insertY, width, height));
					src.copyTo(dst);
				}
			}
		}
		cv::imwrite(outputFolder_ + texture_mesh_->tex_materials[textureIndex].tex_file, texture);
	}

	// Create nonVisibleFaces texture and save to file
	cv::Mat nonVisibleFacesTexture = cv::Mat::zeros(50, 50, CV_8UC3) + cv::Scalar(255, 255, 255);
	cv::imwrite(outputFolder_ + texture_mesh_->tex_materials[nrTextures_].tex_file, nonVisibleFacesTexture);
}

void Texturing::write_obj_file()
{
	if (Texturing::saveOBJFile(outputFolder_ + "odm_textured_model.obj", *texture_mesh_.get(), 7) == 0)
	{
		std::cout << "odm_textured_model.obj successfully saved.\n";
	}
	else
	{
		std::cout << "Failed to save model.\n";
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

int Texturing::saveOBJFile(const std::string &file_name, const pcl::TextureMesh &tex_mesh, unsigned precision)
{
	if (tex_mesh.cloud.data.empty())
	{
		PCL_ERROR("[pcl::io::saveOBJFile] Input point cloud has no data!\n");
		return (-1);
	}

	// Open file
	std::ofstream fs;
	fs.precision(precision);
	fs.open(file_name.c_str());

	// Define material file
	std::string mtl_file_name = file_name.substr(0, file_name.find_last_of(".")) + ".mtl";
	// Strip path for "mtllib" command
	std::string mtl_file_name_nopath = mtl_file_name;
	//std::cout << mtl_file_name_nopath << std::endl;
	mtl_file_name_nopath.erase(0, mtl_file_name.find_last_of('/') + 1);

	/* Write 3D information */
	// number of points
	int nr_points = tex_mesh.cloud.width * tex_mesh.cloud.height;
	int point_size = tex_mesh.cloud.data.size() / nr_points;

	// mesh size
	int nr_meshes = tex_mesh.tex_polygons.size();
	// number of faces for header
	int nr_faces = 0;
	for (int m = 0; m < nr_meshes; ++m)
		nr_faces += tex_mesh.tex_polygons[m].size();

	// Write the header information
	fs << "####" << std::endl;
	fs << "# OBJ dataFile simple version. File name: " << file_name << std::endl;
	fs << "# Vertices: " << nr_points << std::endl;
	fs << "# Faces: " << nr_faces << std::endl;
	fs << "# Material information:" << std::endl;
	fs << "mtllib " << mtl_file_name_nopath << std::endl;
	fs << "####" << std::endl;

	// Write vertex coordinates
	fs << "# Vertices" << std::endl;
	for (int i = 0; i < nr_points; ++i)
	{
		int xyz = 0;
		// "v" just be written one
		bool v_written = false;
		for (size_t d = 0; d < tex_mesh.cloud.fields.size(); ++d)
		{
			int count = tex_mesh.cloud.fields[d].count;
			if (count == 0)
				count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
			int c = 0;
			// adding vertex
			if ((tex_mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) /*sensor_msgs::PointField::FLOAT32)*/ && (
				tex_mesh.cloud.fields[d].name == "x" ||
				tex_mesh.cloud.fields[d].name == "y" ||
				tex_mesh.cloud.fields[d].name == "z"))
			{
				if (!v_written)
				{
					// write vertices beginning with v
					fs << "v ";
					v_written = true;
				}
				float value;
				memcpy(&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof(float)], sizeof(float));
				fs << value;
				if (++xyz == 3)
					break;
				fs << " ";
			}
		}
		if (xyz != 3)
		{
			PCL_ERROR("[pcl::io::saveOBJFile] Input point cloud has no XYZ data!\n");
			return (-2);
		}
		fs << std::endl;
	}
	fs << "# " << nr_points << " vertices" << std::endl;

	//  // Write vertex normals
	//  for (int i = 0; i < nr_points; ++i)
	//  {
	//    int xyz = 0;
	//    // "vn" just be written one
	//    bool v_written = false;
	//    for (size_t d = 0; d < tex_mesh.cloud.fields.size (); ++d)
	//    {
	//      int count = tex_mesh.cloud.fields[d].count;
	//      if (count == 0)
	//      count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
	//      int c = 0;
	//      // adding vertex
	//      if ((tex_mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
	//      tex_mesh.cloud.fields[d].name == "normal_x" ||
	//      tex_mesh.cloud.fields[d].name == "normal_y" ||
	//      tex_mesh.cloud.fields[d].name == "normal_z"))
	//      {
	//        if (!v_written)
	//        {
	//          // write vertices beginning with vn
	//          fs << "vn ";
	//          v_written = true;
	//        }
	//        float value;
	//        memcpy (&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
	//        fs << value;
	//        if (++xyz == 3)
	//          break;
	//        fs << " ";
	//      }
	//    }
	//    if (xyz != 3)
	//    {
	//    //PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no normals!\n");
	//    //return (-2);
	//    }
	//    fs << std::endl;
	//  }
	  // Write vertex texture with "vt" (adding latter)

	for (int m = 0; m < nr_meshes; ++m)
	{
		if (tex_mesh.tex_coordinates.size() == 0)
			continue;

		//PCL_INFO ("%d vertex textures in submesh %d\n", tex_mesh.tex_coordinates[m].size (), m);
		fs << "# " << tex_mesh.tex_coordinates[m].size() << " vertex textures in submesh " << m << std::endl;
		for (size_t i = 0; i < tex_mesh.tex_coordinates[m].size(); ++i)
		{
			fs << "vt ";
			fs << tex_mesh.tex_coordinates[m][i][0] << " " << tex_mesh.tex_coordinates[m][i][1] << std::endl;
		}
	}

	int f_idx = 0;

	// int idx_vt =0;
	//PCL_INFO ("Writting faces...\n");
	for (int m = 0; m < nr_meshes; ++m)
	{
		if (m > 0)
			f_idx += tex_mesh.tex_polygons[m - 1].size();

		if (tex_mesh.tex_materials.size() != 0)
		{
			fs << "# The material will be used for mesh " << m << std::endl;
			//TODO pbl here with multi texture and unseen faces
			fs << "usemtl " << tex_mesh.tex_materials[m].tex_name << std::endl;
			fs << "# Faces" << std::endl;
		}
		for (size_t i = 0; i < tex_mesh.tex_polygons[m].size(); ++i)
		{
			// Write faces with "f"
			fs << "f";
			size_t j = 0;
			// There's one UV per vertex per face, i.e., the same vertex can have
			// different UV depending on the face.
			for (j = 0; j < tex_mesh.tex_polygons[m][i].vertices.size(); ++j)
			{
				unsigned int idx = tex_mesh.tex_polygons[m][i].vertices[j] + 1;
				fs << " " << idx
					<< "/" << 3 * (i + f_idx) + j + 1;
				//<< "/" << idx; // vertex index in obj file format starting with 1
			}
			fs << std::endl;
		}
		//PCL_INFO ("%d faces in mesh %d \n", tex_mesh.tex_polygons[m].size () , m);
		fs << "# " << tex_mesh.tex_polygons[m].size() << " faces in mesh " << m << std::endl;
	}
	fs << "# End of File";

	// Close obj file
	//PCL_INFO ("Closing obj file\n");
	fs.close();

	/* Write material defination for OBJ file*/
	// Open file
	//PCL_INFO ("Writing material files\n");
	//dont do it if no material to write
	if (tex_mesh.tex_materials.size() == 0)
		return (0);

	std::ofstream m_fs;
	m_fs.precision(precision);
	m_fs.open(mtl_file_name.c_str());
	//std::cout << "MTL file is located at_ " << mtl_file_name << std::endl;
	// default
	m_fs << "#" << std::endl;
	m_fs << "# Wavefront material file" << std::endl;
	m_fs << "#" << std::endl;
	for (int m = 0; m < nr_meshes; ++m)
	{
		m_fs << "newmtl " << tex_mesh.tex_materials[m].tex_name << std::endl;
		m_fs << "Ka " << tex_mesh.tex_materials[m].tex_Ka.r << " " << tex_mesh.tex_materials[m].tex_Ka.g << " " << tex_mesh.tex_materials[m].tex_Ka.b << std::endl; // defines the ambient color of the material to be (r,g,b).
		m_fs << "Kd " << tex_mesh.tex_materials[m].tex_Kd.r << " " << tex_mesh.tex_materials[m].tex_Kd.g << " " << tex_mesh.tex_materials[m].tex_Kd.b << std::endl; // defines the diffuse color of the material to be (r,g,b).
		m_fs << "Ks " << tex_mesh.tex_materials[m].tex_Ks.r << " " << tex_mesh.tex_materials[m].tex_Ks.g << " " << tex_mesh.tex_materials[m].tex_Ks.b << std::endl; // defines the specular color of the material to be (r,g,b). This color shows up in highlights.
		m_fs << "d " << tex_mesh.tex_materials[m].tex_d << std::endl; // defines the transparency of the material to be alpha.
		m_fs << "Ns " << tex_mesh.tex_materials[m].tex_Ns << std::endl; // defines the shininess of the material to be s.
		m_fs << "illum " << tex_mesh.tex_materials[m].tex_illum << std::endl; // denotes the illumination model used by the material.
		// illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
		// illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
		m_fs << "map_Kd " << tex_mesh.tex_materials[m].tex_file << std::endl;
		m_fs << "###" << std::endl;
	}
	m_fs.close();
	return (0);
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
	//std::vector<Eigen::Vector2f> texture_coordinates_vector;
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
			// texture_coordinates_vector[0].push_back(uv);
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



	//texture_mesh_->tex_coordinates.push_back(texture_coordinates_vector) ;
	/*texture_mesh_->tex_coordinates.clear();
	texture_mesh_->tex_coordinates = texture_coordinates_vector;
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


