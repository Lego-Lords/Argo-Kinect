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
	//std::cout << "Going to read: " << modelStepFileName << std::endl;
	pcl::PCLPointCloud2 cloud;
	if (pcl::io::loadPCDFile(modelStepFileName, cloud) == -1) //* load the file
	{
		//PCL_ERR OR("Couldn't read file test_pcd.pcd \n");
		//return (-1);
	}
	else
	{
		//std::cout << "Fields: " << pcl::getFieldsList(cloud).c_str() << std::endl;
		pcl::fromPCLPointCloud2(cloud, *result);
		for (size_t i = 0; i < result->size(); i++) {
			result->points[i].a = 255;
		}
		/*pcl::PointXYZRGBA point;

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
	filename += ".pcd";
	if (pcl::io::savePCDFile(filename, *input) == 0) {
		cout << "Saved " << filename << "." << endl;
	}
	
	else PCL_ERROR("Problem saving %s.\n", filename.c_str());
	cin.get();
}

void PCDHelper::saveVFHinPCD(std::string filename, pcl::PointCloud<pcl::VFHSignature308>::Ptr input) {
	if (pcl::io::savePCDFile(filename, *input) == 0) {
		cout << "Saved " << filename << ". Size: " << input->size() << endl;
	}

	
	else PCL_ERROR("Problem saving %s.\n", filename.c_str());
}

void PCDHelper::readVFHinPCD(std::string filename, pcl::PointCloud<pcl::VFHSignature308>::Ptr cloud) {
	if (pcl::io::loadPCDFile(filename, *cloud) == 0) {
		cout << "Going to read " << filename << ". Size: " << cloud->size() << endl;
	}


	else PCL_ERROR("Problem reading %s.\n", filename.c_str());
}