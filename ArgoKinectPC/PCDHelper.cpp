#include "stdafx.h"
#include "PCDHelper.h"
#include "string"


PCDHelper::PCDHelper()
{
}


PCDHelper::~PCDHelper()
{
}

void PCDHelper::readPCD(std::string modelStepFileName, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result) {
	std::cout << "Going to read: " << modelStepFileName << std::endl;
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(modelStepFileName, *result) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		//return (-1);
	}
	/*std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from test_pcd.pcd with the following fields: "
		<< std::endl;
	for (size_t i = 0; i < cloud->points.size(); ++i)
		std::cout << "    " << cloud->points[i].x
		<< " " << cloud->points[i].y
		<< " " << cloud->points[i].z << std::endl;*/
	/*
	//VIEW THE SHIT
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	// Create PCLVisualizer
	viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("Point Cloud Viewer");

	// Initialize camera position
	viewer->setCameraPosition(0.0, 0.0, -100, 0.0, 0.0, 0.0);

	// Add Coordinate System
	viewer->addCoordinateSystem(0.1);

	if (!viewer->updatePointCloud(cloud, "cloud")) {
		viewer->addPointCloud(cloud, "cloud");
	}

	// Update Viwer
	viewer->spinOnce();
	//END VIEW THE SHIT
	*/

	//return cloud;
}

void PCDHelper::savePCD(std::string filename, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input) {
	if (pcl::io::savePCDFile(filename, *input, true) == 0) {
		cout << "Saved " << filename << "." << endl;
	}
	
	else PCL_ERROR("Problem saving %s.\n", filename.c_str());
}


//pcl::PointCloud<pcl::PointXYZ>PCDReader getPCD() {
//	return cloud;
//}