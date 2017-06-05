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


void Segmenter::segmentCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output) {
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
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr  smolCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PassThrough<pcl::PointXYZRGB> pass;
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
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.02);

	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
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

void Segmenter::removeSkinPixels() {
	
	
}

