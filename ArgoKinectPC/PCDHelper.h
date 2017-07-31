#pragma once
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include "string"

class PCDHelper
{
public:
	PCDHelper();
	~PCDHelper();
	void readPCD(std::string modelStepFileName, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result);
	void savePCD(std::string filename, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input);
	void saveVFHinPCD(std::string filename, pcl::PointCloud<pcl::VFHSignature308>::Ptr input);
	void readVFHinPCD(std::string filename, pcl::PointCloud<pcl::VFHSignature308>::Ptr cloud);
};