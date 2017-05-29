#include "stdafx.h"
#include "Segmenter.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

Segmenter::Segmenter() {
}


Segmenter::~Segmenter() {
}


void Segmenter::segmentCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output) {
	//initialize data so it can be used in other functions, point class ptrs to actual ptrs in main
	this->input = input; //input is what functions use
	this->output = output; //output is result of the function

	/*************** Segmentation Pipeline ***************/
	//downsample, lower the resolution of the kinect so it would be faster to process

	//crop the area that the kinect can see
	lowerVisibleArea();

	//downsample point cloud
	downsampleCloud();

	//remove largest plane
	segmentPlane();

}

void Segmenter::lowerVisibleArea() {
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  smolCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setInputCloud(input);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.3, 0.9);
	pass.filter(*output);
	*input = *output;
}

void Segmenter::downsampleCloud() {
	
}

void Segmenter::segmentPlane() {
	//Remove the plane (floor/table)
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.005);

	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	int i = 0, nr_points = (int)input->points.size();

	// Form the largest planar component from the cloud
	seg.setInputCloud(input);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() == 0) {
		std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	}

	// Extract yung mga di kasama sa plane aka outliers
	extract.setInputCloud(input);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*output);
	*input = *output;
}

pcl::PointCloud<pcl::PointXYZRGB> Segmenter::voxelize(pcl::PointCloud<pcl::PointXYZRGB> cloud) {
	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	pcl::toPCLPointCloud2(cloud, *cloud_blob);
	// Create the filtering object
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud_blob);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*cloud_filtered_blob);

	// Convert from blob to legit cloud
	pcl::fromPCLPointCloud2(*cloud_blob, *cloud_filtered);
	return *cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB> Segmenter::removePlane(pcl::PointCloud<pcl::PointXYZRGB> cloud) {

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.01);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(&cloud);
	seg.setInputCloud(cloudPtr);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0) {
		std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
		//break;
	}

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;

	// Extract yung mga di kasama sa plane
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_extracted = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	extract.setInputCloud(cloudPtr);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloud_extracted);
	//std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
	return *cloud_extracted;
}

pcl::PointCloud<pcl::PointXYZRGB> Segmenter::segment(pcl::PointCloud<pcl::PointXYZRGB> cloud) {

	return voxelize(cloud);

}
