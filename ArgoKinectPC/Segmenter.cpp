#include "stdafx.h"
#include "Segmenter.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

Segmenter::Segmenter()
{
}


Segmenter::~Segmenter()
{
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr Segmenter::voxelize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	pcl::PCLPointCloud2::Ptr cloud_blob, cloud_filtered_blob;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
	pcl::toPCLPointCloud2(*cloud, *cloud_blob);
	// Create the filtering object
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud_blob);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*cloud_filtered_blob);

	// Convert from blob to legit cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	pcl::fromPCLPointCloud2(*cloud_blob, *cloud_filtered);
	return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Segmenter::removePlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{

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


	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
		std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
		//break;
	}

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;

	// Extract yung mga di kasama sa plane
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_extracted;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloud_extracted);
	//std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
	return cloud_extracted;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Segmenter::segment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	
	return removePlane(voxelize(cloud));

}
