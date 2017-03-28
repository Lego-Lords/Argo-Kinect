#pragma once
#ifndef ICPCOMPARE_H
#define ICPCOMPARE_H
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

extern pcl::PointCloud<pcl::PointXYZ>::Ptr icp;

class ICPCompare
{
	//add method skeleton
public:
	ICPCompare();
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> comparePCD(pcl::PointCloud<pcl::PointXYZ>::Ptr pcIn, pcl::PointCloud<pcl::PointXYZ>::Ptr pcOut);
	
};

#endif

