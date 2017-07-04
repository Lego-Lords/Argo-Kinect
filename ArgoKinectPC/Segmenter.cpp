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
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>



Segmenter::Segmenter() {
}


Segmenter::~Segmenter() {
}


void Segmenter::segmentCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer) {
	//initialize data so it can be used in other functions, point class ptrs to actual ptrs in main
	this->input = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();; 
	*(this->input) = *input;//input is what functions use
	this->output = output; //output is result of the function


	this->viewer = viewer; //for visualization

	*output = *input;

	/*************** Segmentation Pipeline ***************/
	//downsample, lower the resolution of the kinect so it would be faster to process
	//downsampleCloud();

	//crop the area that the kinect can see
	lowerVisibleArea();

	//remove largest plane
	//segmentPlane();

	//remove lighting
	normalizeColor();

	//remove skin/iwan lang bricks
	isolateBricks();

	//remove outliers, mga fake colors
	removeOutliers();
}

void Segmenter::lowerVisibleArea() {
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  smolCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setInputCloud(input);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.4, 0.9);
	pass.filter(*output);
	*input = *output;
}

void Segmenter::downsampleCloud() {
	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*input, *cloud_blob);
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud_blob);
	sor.setLeafSize(0.01f, 0.01f, 0.01f); //0.01 = 1 cm, 0.001 = 1 mm
	sor.filter(*cloud_filtered_blob);
	pcl::fromPCLPointCloud2(*cloud_filtered_blob, *output);
	*input = *output;
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
	seg.setDistanceThreshold(0.0085);

	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	int i = 0, nr_points = (int)input->points.size();

	// Form the largest planar component from the cloud
	seg.setInputCloud(input);
	seg.segment(*inliers, *coefficients);
	//if (inliers->indices.size() == 0) {
	//	std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	//}

	// Extract yung mga di kasama sa plane aka outliers
	extract.setInputCloud(input);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*output);
	*input = *output;
}



void Segmenter::isolateBricks() {
	//init
	input_noA = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	pcl::copyPointCloud(*input, *input_noA);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  redBricks(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  greenBricks(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  blueBricks(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  yellowBricks(new pcl::PointCloud<pcl::PointXYZRGBA>);

	//filter per color
	filterColorBricks(redBricks, 255, 70, 69, 0, 69, 0);
	filterColorBricks(greenBricks, 150, 0, 255, 150, 100, 0);
	filterColorBricks(blueBricks, 69, 0, 69, 0, 255, 70);
	filterColorBricks(yellowBricks, 255, 150, 255, 150, 100, 0);

	output->clear();
	*output += *redBricks;
	*output += *greenBricks;
	*output += *blueBricks;
	*output += *yellowBricks;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> segmented_cloud_color_handler(output, 255, 255, 165);
	//if (!viewer->updatePointCloud(output, segmented_cloud_color_handler, "segmented_cloud")) {
		//viewer->addPointCloud(output, segmented_cloud_color_handler, "segmented_cloud");
	//}
	//std::cout << "cloud size: " << output->size() << std::endl;
	*input = *output;
}

void Segmenter::filterColorBricks(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered, int rMax, int rMin, int gMax, int gMin, int bMax, int bMin) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr dummyNoAFiltered = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	
	pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::LT, rMax)));
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::GT, rMin)));
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::LT, gMax)));
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::GT, gMin)));
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::LT, bMax)));
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::GT, bMin)));

	// build the filter 
	pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem(color_cond);
	condrem.setInputCloud(input_noA);

	// apply filter 
	condrem.filter(*dummyNoAFiltered);

	pcl::copyPointCloud(*dummyNoAFiltered, *filtered);
}

void Segmenter::normalizeColor() {
	for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = input->begin(); it != input->end(); it++) {
		float total = it->r + it->g + it->b;
		it->r = it->r / total * 255;
		it->g = it->g / total * 255;
		it->b = it->b / total * 255;
	}
	*output = *input;
}

void Segmenter::removeOutliers() {
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
	sor.setInputCloud(input);
	sor.setMeanK(50);
	sor.setStddevMulThresh(0.5);
	sor.filter(*output);
	*input = *output;
}

