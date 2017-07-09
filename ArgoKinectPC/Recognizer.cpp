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
#include <pcl/common/centroid.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/time.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>


Recognizer::Recognizer() {
	selectedModel = 3;
	maxSteps = 0;
	currStep = 5;
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

	// Visualization
	this->viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("ICP Viewer");

}


Recognizer::~Recognizer() {
}

void Recognizer::recognizeState(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer) {
	this->input = input;
	
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
			estimatePose();
			//performICP();
			// compute normals
			/*computeNormals(cloudAgainst, model_normals, 10);
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
			clusterCorrespondences(0.015f, 5.0f);*/


		}
	}
	
}

void Recognizer::getCloudToCompare() {
	std::string stepfile = "";
	switch (selectedModel) {
		case 1: stepfile = snowcat; break;
		case 2: stepfile = pyramid; break;
		case 3: stepfile = quacktro; break;
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


void Recognizer::estimatePose()
{
	//compute centroids of both clouds
	Eigen::Vector4f sceneCentroid, modelCentroid, poseTranslate;
	pcl::compute3DCentroid(*input, sceneCentroid);
	pcl::compute3DCentroid(*cloudAgainst, modelCentroid);

	//get the difference between the 2 centroids
	poseTranslate(0) = sceneCentroid(0) - modelCentroid(0);
	poseTranslate(1) = sceneCentroid(1) - modelCentroid(1);
	poseTranslate(2) = sceneCentroid(2) - modelCentroid(2);
	poseTranslate(3) = sceneCentroid(3) - modelCentroid(3);

	//std::cout << "estimate translation: " << poseTranslate << std::endl;

	//compute normals at every point
	//computeNormals(cloudAgainst, model_normals, 10);
	
	computeNormals(input, scene_normals, 10);


	//viewer->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(input, scene_normals, 10, 0.05, "normals");

	// Estimate features
	/*pcl::console::print_highlight("Estimating features...\n");
	::FPFHEstimationOMP<pcl::Normal, pcl::Normal, pcl::FPFHSignature33> fest;
	fest.setRadiusSearch(0.025);
	fest.setInputCloud(cloudAgainst);
	fest.setInputNormals(model_normals);
	fest.compute(*object_features);
	fest.setInputCloud(scene);
	fest.setInputNormals(scene);
	fest.compute(*scene_features);

	// Perform alignment
	pcl::console::print_highlight("Starting alignment...\n");
	pcl::FPFHSignature33SampleConsensusPrerejective<>
	pcl::FPFHSignature33SampleConsensusPrerejective<pcl::Normal, pcl::PointNormal, pcl::FPFHSignature33> align;
	align.setInputSource(object);
	align.setSourceFeatures(object_features);
	align.setInputTarget(scene);
	align.setTargetFeatures(scene_features);
	align.setMaximumIterations(50000); // Number of RANSAC iterations
	align.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
	align.setCorrespondenceRandomness(5); // Number of nearest features to use
	align.setSimilarityThreshold(0.9f); // Polygonal edge length similarity threshold
	align.setMaxCorrespondenceDistance(2.5f * leaf); // Inlier threshold
	align.setInlierFraction(0.25f); // Required inlier fraction for accepting a pose hypothesis
	{
		pcl::ScopeTime t("Alignment");
		align.align(*object_aligned);
	}*/


}

void
Recognizer::print4x4Matrix(const Eigen::Matrix4d & matrix) {
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void Recognizer::performICP()
{
	pread.readPCD("compare_snowcat_1.pcd", input);
	pread.readPCD("steps/snowcat_step_32.pcd", cloudAgainst);
	int age;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> input_color(input, 255, 255, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> goal_color (cloudAgainst, 20, 180, 20);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> icp_color(cloud_icp, 180, 20, 20);
	//viewer->addPointCloud(input, input_color, "input");
	viewer->addPointCloud(cloudAgainst, goal_color, "target");
	viewer->addPointCloud(input, icp_color, "aligned");
	

	int iterations = 1;
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
	icp.setMaximumIterations(100);
	icp.setInputSource(input);
	icp.setInputTarget(cloudAgainst);
	bool next_iteration = true;

	while (next_iteration)
	{
		icp.align(*input);
		
		cin >> age;
		if (icp.hasConverged()) {
			printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
			std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloudAgainst" << std::endl;
			transformation_matrix = icp.getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
			print4x4Matrix(transformation_matrix);
			//viewer->updatePointCloud(input, icp_color, "aligned");
		}

		else {
			PCL_ERROR("\nICP has not converged.\n");
		}
	}

	

	
}
