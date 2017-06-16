#include "stdafx.h"
#include "Recognizer.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>


Recognizer::Recognizer() {
	selectedModel = 0;
	maxSteps = 0;
	currStep = 1;
	sceneFound = 0;
	hasUpdate = true;
	cloudAgainst = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
	model_normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
	scene_normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
}


Recognizer::~Recognizer() {
}

void Recognizer::recognizeState(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer) {
	
	//if no model is selected, wait for selection
	if (selectedModel == 0) {
		//get selection from database
		selectedModel = sqlCon.getCurrentStep(connection);
	}
	else {
		if (hasUpdate) {
			getCloudToCompare();
			hasUpdate = false;
		}
		if (sceneFound > 0) {
			// compute normals
			computeNormals(input, scene_normals, 10);
			computeNormals(cloudAgainst, model_normals, 10);

			//downsample clouds to get keypoints
			obtainKeypoints(input, scene_keypoints, 0.01f);
			obtainKeypoints(cloudAgainst, model_keypoints, 0.03f);

			//compute descriptor for keypoints
			computeDescriptor(input, scene_keypoints, scene_normals, scene_descriptors, 0.02f);
			computeDescriptor(cloudAgainst, model_keypoints, model_normals, model_descriptors, 0.02f);

			//find correspondences
			findCorrespondences();

		}
	}
	
}

void Recognizer::getCloudToCompare() {
	std::string stepfile = "";
	switch (selectedModel) {
		case 1: stepfile = snowcat; break;
		case 2: stepfile = pyramid; break;
	}
	pread.readPCD(stepfile + std::to_string(currStep + 1) + ".pcd", cloudAgainst);
}

void Recognizer::computeNormals(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::Normal>::Ptr normals, int numNeighbors) {
	pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> norm_est;
	norm_est.setKSearch(numNeighbors);
	norm_est.setInputCloud(input);
	norm_est.compute(*normals);
}

void Recognizer::obtainKeypoints(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints, float radius) {
	pcl::UniformSampling<pcl::PointXYZRGBA> sampler;
	sampler.setInputCloud(input);
	sampler.setRadiusSearch(radius);
	sampler.filter(*keypoints);
}

void Recognizer::computeDescriptor(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::SHOT352>::Ptr descriptors, float radius) {
	pcl::SHOTEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT352> descEstimator;
	descEstimator.setRadiusSearch(radius);
	descEstimator.setInputCloud(keypoints);
	descEstimator.setInputNormals(normals);
	descEstimator.setSearchSurface(inputCloud);
	descEstimator.compute(*descriptors);
}

void Recognizer::findCorrespondences() {
	
}
