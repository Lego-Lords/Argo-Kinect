#include "stdafx.h" 
#include "Recognizer.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/time.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/ransac_based/obj_rec_ransac.h>


Recognizer::Recognizer() {
	selectedModel = 3;
	maxSteps = 6;
	currStep = 5;
	hasUpdate = true;
	trackingActive = false;
	useEstimate = false;


	input = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
	cloudAgainst = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();


	visual = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
	nextStepModel = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
	currentStepModel = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();

	aligned = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
	modelPointNormal = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
	scenePointNormal = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();

	model_normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
	scene_normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();

	model_keypoints = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
	scene_keypoints = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();

	model_descriptors = boost::make_shared<pcl::PointCloud<pcl::SHOT352>>();
	scene_descriptors = boost::make_shared<pcl::PointCloud<pcl::SHOT352>>();

	model_features = boost::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
	scene_features = boost::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();


	model_rf = boost::make_shared<pcl::PointCloud<pcl::ReferenceFrame>>();
	scene_rf = boost::make_shared<pcl::PointCloud<pcl::ReferenceFrame>>();

	initialTransform = Eigen::Matrix4f::Identity();
	// Visualization
	this->viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("ICP Viewer");
	this->viewer->setCameraPosition(0.0, 0.0, -1.0, 0.0, 0.0, 0.0);
	this->viewer->addCoordinateSystem(0.1);

	//this->viewer->addPointCloud(visual, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>(visual, 0.0, 0.0, 255.0), "virtual");
	getCloudToCompare(nextStepModel);
	//centerCloud(nextStepModel);
	this->viewer->addPointCloud(nextStepModel, "try");
	//this->viewer->spinOnce();
}


Recognizer::~Recognizer() {
}

void Recognizer::init()
{
	corrs = boost::make_shared<pcl::Correspondences>();

}


void Recognizer::centerCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {

	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud, centroid);

	Eigen::Affine3f tMatrix;
	pcl::getTransformation(0, 0, 0, 0, 0, 0, tMatrix);

	pcl::transformPointCloud(*cloud, *cloud, tMatrix);
}


void Recognizer::recognizeState(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene) {

	//if no model is selected, wait for selection
	if (selectedModel == 0) {
		//get selection from database
		selectedModel = sqlCon.getCurrentStep(connection);
	}
	else {
		if (currStep != maxSteps) {
			init();
			if (hasUpdate) {
				if (currStep != 0)
					*currentStepModel = *nextStepModel;
				getCloudToCompare(nextStepModel);
				hasUpdate = false;
			}
			if (scene->size() > 0 && !trackingActive) {
				*input = *scene;
				viewer->spinOnce();
				/*std::cout << "Input cloud: " << input->size() << std::endl;

				std::cout << "Computing normals... " << std::endl;
				computeNormals(input, scene_normals, 10);
				computeNormals(nextStepModel, model_normals, 10);

				//recognizeRANSAC();
				
				std::cout << "Obtaining keypoints... " << scene_keypoints->size() << std::endl;
				std::cout << "Obtaining keypoints... " << model_keypoints->size() << std::endl;
				obtainKeypoints(input, scene_keypoints, leafsize);
				obtainKeypoints(nextStepModel, model_keypoints, leafsize);

				std::cout << "Computing scene descriptors.. " << std::endl;
				//compute descriptor for keypoints
				computeDescriptor(input, scene_keypoints, scene_normals, scene_descriptors, 0.06f);
				std::cout << "Computing model descriptors.. " << std::endl;
				computeDescriptor(nextStepModel, model_keypoints, model_normals, model_descriptors, 0.06f);
				std::cout << "Obtaining descriptors... " << scene_descriptors->size() << std::endl;
				std::cout << "Obtaining descriptors... " << model_descriptors->size() << std::endl;

				std::cout << "Finding correspondences... " << std::endl;
				//find correspondences
				findCorrespondences();

				std::cout << "Computing ref frames... " << std::endl;
				//compute keypoints then cluster correspondences found
				computeReferenceFrames(model_keypoints, model_normals, nextStepModel, model_rf, 0.015f);
				computeReferenceFrames(scene_keypoints, scene_normals, input, scene_rf, 0.015f);
				std::cout << "Obtaining ref... " << scene_rf->size() << std::endl;
				std::cout << "Obtaining ref... " << model_rf->size() << std::endl;


				std::cout << "Clustering correspondences... " << std::endl;
				//cluster correspondences to find object
				clusterCorrespondences(0.015f, 10.0f); */
				//trackingActive = true;
				
				//performICP();
				
				 /*
				std::cout << "Downsampling... " << std::endl;
				downsample(input, input, leafsize);
				*visual = *cloudAgainst;
				downsample(nextStepModel, nextStepModel, leafsize);
				copyPointCloud(*cloudAgainst, *modelPointNormal);
				copyPointCloud(*input, *scenePointNormal);
				
				computePointNormals(modelPointNormal, 0.01);
				computePointNormals(scenePointNormal, 0.01);
				if (!trackingActive) {
					estimatePose();
				}
				performICP();*/
				

			}
		}
	}
}

void Recognizer::recognizeRANSAC()
{
	//Object for recognition output
	std::list<pcl::recognition::ObjRecRANSAC::Output> recog_output;
	pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*nextStepModel, *object);
	pcl::copyPointCloud(*input, *scene);
	const float pair_width = 100.0f;
	const float voxel_size = leafsize;
	std::cout << "Recognizing using RANSAC" << std::endl;
	pcl::recognition::ObjRecRANSAC recognition(pair_width, voxel_size);//recognition object
	recognition.addModel(*object, *model_normals, "next step");

	recognition.recognize(*scene, *scene_normals, recog_output, 0.99f);
}

void Recognizer::getCloudToCompare(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr saved) {
	std::string stepfile = "";
	switch (selectedModel) {
		case 1: stepfile = snowcat; break;
		case 2: stepfile = pyramid; break;
		case 3: stepfile = quacktro; break;
		case 4: stepfile = jay; break;
		case 5: stepfile = heart; break;
	}
	pread.readPCD(stepfile + std::to_string(currStep + 1) + ".pcd", saved);
	std::cout << "Obtained cloud " << saved->size() << std::endl;
}

void Recognizer::computeNormals(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::Normal>::Ptr normals, float val) {
	pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> norm_est;
	norm_est.setRadiusSearch(val);
	norm_est.setInputCloud(inputCloud);
	norm_est.compute(*normals);
}

void Recognizer::computePointNormals(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, float val) {
	pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal> norm_est;
	norm_est.setRadiusSearch(val);
	norm_est.setInputCloud(inputCloud);
	norm_est.compute(*inputCloud);
}

void Recognizer::obtainKeypoints(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints, float radius) {
	pcl::UniformSampling<pcl::PointXYZRGBA> sampler;
	sampler.setInputCloud(inputCloud);
	sampler.setRadiusSearch(radius);
	sampler.filter(*keypoints);
}

void Recognizer::downsample(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outputCloud, float leafsize) {
	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*inputCloud, *cloud_blob);
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud_blob);
	sor.setLeafSize(leafsize, leafsize, leafsize);
	sor.filter(*cloud_filtered_blob);
	pcl::fromPCLPointCloud2(*cloud_filtered_blob, *outputCloud);
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
	std::cout << "Correspondences found: " << corrs->size() << std::endl;
}
/**void Recognizer::clusterCorrespondences(float binSize, float thresh) {
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector < pcl::Correspondences > clustered_corrs;
	pcl::GeometricConsistencyGrouping<pcl::PointXYZRGBA, pcl::PointXYZRGBA> gc_clusterer;
	gc_clusterer.setGCSize(binSize);
	gc_clusterer.setGCThreshold(thresh);

	gc_clusterer.setInputCloud(model_keypoints);
	gc_clusterer.setSceneCloud(scene_keypoints);
	gc_clusterer.setModelSceneCorrespondences(corrs);

	//gc_clusterer.cluster (clustered_corrs);
	std::cout << "Recognize clusters " << std::endl;
	gc_clusterer.recognize(rototranslations, clustered_corrs);

	
	//std::cout << "Model instances found: " << rototranslations.size() << std::endl;

}
/**/
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
	std::cout << "hallo found: " << corrs->size() << std::endl;
	clusterer.recognize(rototranslations, clustCorrs);

	
	std::cout << "Model instances found: " << rototranslations.size () << std::endl;

}


void Recognizer::estimatePose()
{
	// Estimate features
	std::cout << "Estimating features... " << std::endl;
	pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fest;
	fest.setRadiusSearch(0.025);
	fest.setInputCloud(modelPointNormal);
	fest.setInputNormals(modelPointNormal);
	fest.compute(*model_features);

	fest.setInputCloud(scenePointNormal);
	fest.setInputNormals(scenePointNormal);
	fest.compute(*scene_features);

	// Perform alignment
	std::cout << "Starting alignment... " << std::endl;
	pcl::SampleConsensusInitialAlignment<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> align;
	align.setInputSource(modelPointNormal);
	align.setSourceFeatures(model_features);
	align.setInputTarget(scenePointNormal);
	align.setTargetFeatures(scene_features);
	align.setMinSampleDistance(0.05f);

	align.setMaximumIterations(5000); // Number of RANSAC iterations
	//align.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
	//align.setCorrespondenceRandomness(2); // Number of nearest features to use
	//align.setSimilarityThreshold(0.6f); // Polygonal edge length similarity threshold
	align.setMaxCorrespondenceDistance(2.5f * leafsize); // Inlier threshold
	//align.setInlierFraction(0.25f); // Required inlier fraction for accepting a pose hypothesis
	{
		pcl::ScopeTime t("Alignment");
		align.align(*aligned);
	}

	if (align.hasConverged()) {
		trackingActive = true;
		// Print results
		std::cout << "Alignment converged... " << std::endl;

		//printf("Inliers: %i/%i\n", align.getInliers().size(), cloudAgainst->size());

		initialTransform = align.getFinalTransformation();

		// Show alignment
		//viewer->addPointCloud(scenePointNormal, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(scenePointNormal, 0.0, 255.0, 0.0), "scene");
		viewer->updatePointCloud(visual, "virtual");
		//viewer->addPointCloud(aligned, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(aligned, 0.0, 0.0, 255.0), "object_aligned");		
	}
	else {
		std::cout << "Alignment failed... " << std::endl;
	}
}

void Recognizer::performICP()
{
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
	icp.setMaximumIterations(1000);
	icp.setMaxCorrespondenceDistance(0.05);
	icp.setRANSACOutlierRejectionThreshold(1.5);
	icp.setEuclideanFitnessEpsilon(1);
	icp.setTransformationEpsilon(1e-9);

	icp.setInputSource(modelPointNormal);
	icp.setInputTarget(scenePointNormal);
	
	icp.align(*aligned, initialTransform);
	if (icp.hasConverged()) {
		transformation_matrix = icp.getFinalTransformation();
		initialTransform = transformation_matrix;
		//std::cout << "Outliers: " << icp. << std::endl;
		std::cout << "ICP converged, score: " << icp.getFitnessScore() << std::endl;
		print4x4Matrix(transformation_matrix);
		pcl::transformPointCloud(*visual, *visual, transformation_matrix);
		viewer->updatePointCloud(visual, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>(visual, 0.0, 0.0, 255.0), "virtual");
		
		viewer->spinOnce();
	}

	else {
		std::cout << "ICP failed" << std::endl;
	}
	
}

void Recognizer::convertRGBAtoPointNormal(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input, pcl::PointCloud<pcl::PointNormal>::Ptr output)
{
	output->resize(input->points.size());
	for (size_t i = 0; i < input->points.size(); ++i) {
		output->points[i].x = input->points[i].x;
		output->points[i].y = input->points[i].y;
		output->points[i].z = input->points[i].z;
	}
}


void Recognizer::print4x4Matrix(const Eigen::Matrix4f & matrix) {
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

