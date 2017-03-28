#include "stdafx.h"
#include "PCDReader.h"
#include "string"


PCDReader::PCDReader()
{
}


PCDReader::~PCDReader()
{
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCDReader::readPCD(std::string modelStepFileName) {

	// SAVE TO CLOUD VARIABLE
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(modelStepFileName, *cloud) == -1) //* load the file
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

	return cloud;
}

//pcl::PointCloud<pcl::PointXYZ>PCDReader getPCD() {
//	return cloud;
//}