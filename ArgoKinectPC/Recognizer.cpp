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
	selectedModel = 1;
	maxSteps = 0;
	currStep = 0;
	hasUpdate = true;

	cloudAgainst = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();

	model_normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
	scene_normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();

	model_keypoints = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
	scene_keypoints = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();

	model_descriptors = boost::make_shared<pcl::PointCloud<pcl::SHOT352>>();
	scene_descriptors = boost::make_shared<pcl::PointCloud<pcl::SHOT352>>();

	corrs = boost::make_shared<pcl::Correspondences>();

	model_rf = boost::make_shared<pcl::PointCloud<pcl::ReferenceFrame>>();
	scene_rf = boost::make_shared<pcl::PointCloud<pcl::ReferenceFrame>>();
}


Recognizer::~Recognizer() {
}

void Recognizer::recognizeState(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer) {
	this->input = input;
	this->viewer = viewer;
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
		if (input->size() > 0 && cloudAgainst->size() > 0) {
			// compute normals
			computeNormals(cloudAgainst, model_normals, 10);
			computeNormals(input, scene_normals, 10);
		
			//downsample clouds to get keypoints
			obtainKeypoints(cloudAgainst, model_keypoints, 0.03f);
			obtainKeypoints(input, scene_keypoints, 0.01f);
			
			//compute descriptor for keypoints
			computeDescriptor(cloudAgainst, model_keypoints, model_normals, model_descriptors, 0.02f);
			computeDescriptor(input, scene_keypoints, scene_normals, scene_descriptors, 0.02f);
			
			//find correspondences
			findCorrespondences();

			//compute keypoints then cluster correspondences found
			computeReferenceFrames(model_keypoints, model_normals, cloudAgainst, model_rf, 0.015f);
			computeReferenceFrames(scene_keypoints, scene_normals, input, model_rf, 0.015f);
			
			//cluster correspondences to find object
			clusterCorrespondences(0.015f, 5.0f);

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
	
	pcl::KdTreeFLANN<pcl::SHOT352> kdsearch;
	kdsearch.setInputCloud(model_descriptors);

	for (size_t i = 0; i < scene_descriptors->size(); ++i) {
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		if (!pcl_isfinite(scene_descriptors->at(i).descriptor[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = kdsearch.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
		if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
		{
			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			corrs->push_back(corr);
		}
	}
}

void Recognizer::computeReferenceFrames(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::ReferenceFrame>::Ptr rf, float radius) {
	pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::ReferenceFrame> rf_est;
	rf_est.setFindHoles(true);
	rf_est.setRadiusSearch(radius);
	rf_est.setInputCloud(keypoints);
	rf_est.setInputNormals(normals);
	rf_est.setSearchSurface(inputCloud);
	rf_est.compute(*rf);
}

void Recognizer::clusterCorrespondences(float binSize, float thresh) {
	pcl::Hough3DGrouping<pcl::PointXYZRGBA, pcl::PointXYZRGBA, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
	clusterer.setHoughBinSize(binSize);
	clusterer.setHoughThreshold(thresh);
	clusterer.setUseInterpolation(true);
	clusterer.setUseDistanceWeight(false);

	clusterer.setInputCloud(model_keypoints);
	clusterer.setInputRf(model_rf);
	clusterer.setSceneCloud(scene_keypoints);
	clusterer.setSceneRf(scene_rf);
	clusterer.setModelSceneCorrespondences(corrs);

	clusterer.recognize(rototranslations, clustCorrs);

	std::cout << "Correspondences found: " << corrs->size() << std::endl;
	std::cout << "Model instances found: " << rototranslations.size () << std::endl;

}
