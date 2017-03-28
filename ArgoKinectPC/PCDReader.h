#pragma once
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "string"

class PCDReader
{
public:
	PCDReader();
	~PCDReader();
	pcl::PointCloud<pcl::PointXYZ>::Ptr readPCD(std::string modelStepFileName);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr getPCD();
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};