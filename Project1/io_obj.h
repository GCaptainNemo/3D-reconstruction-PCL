#define _CRT_SECURE_NO_WARNINGS

class ioOBJ 
{
public:
	ioOBJ();
	~ioOBJ();
public:
	void bin2pcd(const char *filenameInput, const char *filenameOutput);
	void read_calib(const char *filename);
	void read_pointcloud(const char *filename);
	void read_image(const char *filename);

};








