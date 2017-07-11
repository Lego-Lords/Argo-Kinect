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
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr results(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(modelStepFileName, *result) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		//return (-1);
	}
	else
	{
		/*pcl::PointXYZRGBA point;
		//pcl::fromPCLPointCloud2(*cloud, *result);
		for (size_t i = 0; i < result->points.size(); ++i)
		{
			//uint32_t rgb = *reinterpret_cast<int*>(&(results->points[i].rgb));

			std::cout << " R:   " << result->points[i].r << std::endl;

			//uint8_t r = (rgb >> 16) & 0x0000ff;
			//uint8_t g = (rgb >> 8) & 0x0000ff;
			//uint8_t b = (rgb) & 0x0000ff;
			//point.r = r;
			//std::cout << " R:   " << point.r << std::endl;
			//point.g = g;
			//point.b = b;
			//point.a = 1.0;
			//point.x = results->points[i].x;
			//point.y = results->points[i].y;
			//point.z = results->points[i].z;

			//result->push_back(point);
		}
		//result->points.*/
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