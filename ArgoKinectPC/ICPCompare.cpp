#include "stdafx.h"
#include "ICPCompare.h"
#include "ICPCompare.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
using namespace std;

ICPCompare::ICPCompare()
{
}

//add what to do
pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> ICPCompare::comparePCD(pcl::PointCloud<pcl::PointXYZ>::Ptr pcIn, pcl::PointCloud<pcl::PointXYZ>::Ptr pcOut)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the CloudIn data
	cloud_in = pcIn;
	cloud_out = pcOut;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_in); // originally setInputCloud
	icp.setInputTarget(cloud_out);

	icp.setMaximumIterations(500);
	icp.setTransformationEpsilon(1e-9);
	icp.setMaxCorrespondenceDistance(0.05);
	icp.setEuclideanFitnessEpsilon(1);
	icp.setRANSACOutlierRejectionThreshold(500);

	pcl::PointCloud<pcl::PointXYZ> Final;
	//icp.setMaximumIterations(1);
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	return icp;
}

