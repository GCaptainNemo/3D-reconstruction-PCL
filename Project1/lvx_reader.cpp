/*
* @Author: meng-zha
* @Date:   2019-12-07 13:02:35
* @Last Modified by:   meng-zha
* @Last Modified time: 2020-06-23 16:41:49
*/

#include "lvx_reader.h"

LvxReader::LvxReader(std::string fileName) : curFrameIndex(0), curOffset(0), fileName(fileName)
{
    lvxFile.open(fileName.c_str(), std::ios::in | std::ios::binary);
    this->InitLvxFileHeader();
}

LvxReader::~LvxReader()
{
    lvxFile.close();
    delete[] deviceInfo;
}

void LvxReader::InitLvxFileHeader()
{
    // Public Header Block
    lvxFile.read((char *)publicHeader.signature, sizeof(publicHeader.signature));
    lvxFile.read((char *)publicHeader.version, sizeof(publicHeader.version));
    lvxFile.read((char *)&publicHeader.magicCode, sizeof(publicHeader.magicCode));

    cout<<(int)publicHeader.version[1]<<endl;

    // Device Info Block
    if (publicHeader.version[1]=='\001')    // Change of version 1.1.0.0
    {
        lvxFile.read((char *)&privateHeader.frameDuration,sizeof(privateHeader.frameDuration));
        lvxFile.read((char *)&privateHeader.deviceCount,sizeof(privateHeader.deviceCount));
        deviceCount = privateHeader.deviceCount;
    }
    else
    {
        lvxFile.read((char *)&deviceCount, sizeof(deviceCount));
    }
    
    deviceInfo = new LvxDeviceInfo[(int)deviceCount];
    if (publicHeader.version[1]=='\001')    // Change of version 1.1.0.0
    {
        lvxFile.read((char *)deviceInfo, ((int)deviceCount) * sizeof(LvxDeviceInfo));
    }
    else
    {
        // This is an ugly code, which makes the device info chaos.
        lvxFile.read((char *)deviceInfo, ((int)deviceCount) * (sizeof(LvxDeviceInfo)-1));
    }
    
}


// read next frame of points cloud
pcl::PointCloud<pcl::PointXYZI>::Ptr LvxReader::ReadNextFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    if (lvxFile.peek() == EOF)
    {
        printf("file end\n");
        return nullptr;
    }

    FrameHeader header;
    lvxFile.read((char *)&header, sizeof(header));

    int count=0;
    if (this->publicHeader.version[1] == '\001')
    {
        count = (header.nextOffset-header.currentOffset-sizeof(header))/sizeof(LvxBasePackDetail);
    }
    else
    {
        lvxFile.read((char *)&count,8);
    }
    count = (int)count;

    // record the offset of ith frame
    offsetRecord.push_back(header.currentOffset);

    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    cloud->width = 100 * count;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    cloud->is_dense = false;

    for (int i = 0; i < count; i++)
    {
        LvxBasePackDetail packData;
        lvxFile.read((char *)&packData, sizeof(LvxBasePackDetail));
        if (packData.dataType != '\000')
        {
            // the count of points may lead to errors
            cout<<'The data type needs to be Cartesian Coordinate System!'<<endl;
            abort();
        }
        if (this->publicHeader.version[1] == '\001')
        {
            for (int j = 0; j < 100; j++)
            {
                cloud->points[100 * i + j].x = (float)(reinterpret_cast<int&>(packData.point[j].x))/100;
                cloud->points[100 * i + j].y = (float)(reinterpret_cast<int&>(packData.point[j].y))/100;
                cloud->points[100 * i + j].z = (float)(reinterpret_cast<int&>(packData.point[j].z))/100;
                cloud->points[100 * i + j].intensity = packData.point[j].reflectivity / 255.;
            }
        }
        else{
            for (int j = 0; j < 100; j++)
            {
                cloud->points[100 * i + j].x = packData.point[j].x;
                cloud->points[100 * i + j].y = packData.point[j].y;
                cloud->points[100 * i + j].z = packData.point[j].z;
                cloud->points[100 * i + j].intensity = packData.point[j].reflectivity / 255.;
            }
        }
    }

    // while(!viewer->wasStopped())
    // {
    //     viewer->removeAllPointClouds();
    //     viewer->addPointCloud<pcl::PointXYZI>((pcl::PointCloud<pcl::PointXYZI>::ConstPtr)cloud,"test");
    //     viewer->updatePointCloud<pcl::PointXYZI>((pcl::PointCloud<pcl::PointXYZI>::ConstPtr)cloud,"test");
    //     viewer->spinOnce(50);
    //     break;
    // }

    curFrameIndex++;
    curOffset = header.nextOffset;

    return cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LvxReader::SumXSeconds(int time)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr sumCloud(new pcl::PointCloud<pcl::PointXYZI>);
    sumCloud->height = 1;
    sumCloud->width = 0;
    sumCloud->is_dense = false;
    for (int i = 0; i < time/0.05; i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        cloud = this->ReadNextFrame(cloud);
        if (cloud == nullptr)
        {
            break;
        }
        sumCloud->width += cloud->width;
        sumCloud->points.insert(sumCloud->points.end(), cloud->points.begin(), cloud->points.end());
        sumCloud->points.resize(sumCloud->width * sumCloud->height);
    }
    return sumCloud;
}

int main(int argc, char const *argv[])
{
    printf("hello world\n");
    LvxReader lvx("/media/meng-zha/58b26bdb-c733-4c63-b7d9-4d845394a721/FuXiao_20200111/mid100_raw/mid100_1.lvx");
    // LvxReader lvx("/home/meng-zha/Livox Viewer For Linux Ubuntu16.04_x64 0.5.0/data/record files/2020-02-10 19-46-55.lvx");
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    // cloud=lvx.SumXSeconds(5);
    // char pcdName[100];
    // sprintf(pcdName, "/media/meng-zha/58b26bdb-c733-4c63-b7d9-4d845394a721/FuXiao_20200111/mid100_pcd/FuXiao_%ds_4.pcd", 10);
    // pcl::io::savePCDFileASCII(pcdName, *cloud);
    int index = 0;
    while (1)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr sumCloud(new pcl::PointCloud<pcl::PointXYZI>);
        sumCloud->height = 1;
        sumCloud->width = 0;
        sumCloud->is_dense = false;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        cloud = lvx.ReadNextFrame(cloud);
        sumCloud->width += cloud->width;
        sumCloud->points.insert(sumCloud->points.end(), cloud->points.begin(), cloud->points.end());
        sumCloud->points.resize(sumCloud->width * sumCloud->height);
        if (cloud == nullptr)
        {
            break;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_n(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_n = lvx.ReadNextFrame(cloud_n);
        sumCloud->width += cloud_n->width;
        sumCloud->points.insert(sumCloud->points.end(), cloud_n->points.begin(), cloud_n->points.end());
        sumCloud->points.resize(sumCloud->width * sumCloud->height);
        if (cloud_n == nullptr)
        {
            break;
        }
        char pcdName[100];
        sprintf(pcdName, "/media/meng-zha/58b26bdb-c733-4c63-b7d9-4d845394a721/FuXiao_20200111/mid100_pcd/mid100_seq_10hz/FuXiao_1_%d.pcd", index);
        // sprintf(pcdName,"/home/meng-zha/beginners/lvxReader/data/mid100_2/BeiCao_%d.pcd",index);
        printf("%d,%d\n",index,sumCloud->width);
        pcl::io::savePCDFileASCII(pcdName, *sumCloud);
        index++;
        // printf("%s\n", pcdName);
    }

    // while (!viewer->wasStopped())
    // {
    //     viewer->removeAllPointClouds();
    //     viewer->addPointCloud<pcl::PointXYZI>((pcl::PointCloud<pcl::PointXYZI>::ConstPtr)sumCloud, "test");
    //     viewer->spinOnce(500);
    // }
    // pcl::io::savePCDFileASCII("../data/北操积分10s.pcd", *sumCloud);

    printf("goodbye world\n");
    return 0;
}
