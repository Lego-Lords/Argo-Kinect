#ifndef RECOGNIZE_H
#define RECOGNIZE_H
#include "stdafx.h"
#include "PCDHelper.h"
#include "SQLConnect.h"

class Recognizer {
private:
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output;
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	pcl::PointCloud<pcl::Normal>::Ptr model_normals;
	pcl::PointCloud<pcl::Normal>::Ptr scene_normals;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model_keypoints;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_keypoints;


	pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors;
	pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_features;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features;

	pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf;
	pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudAgainst;

	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector < pcl::Correspondences > clustCorrs;

	pcl::PointCloud<pcl::PointNormal>::Ptr scenePointNormal;
	pcl::PointCloud<pcl::PointNormal>::Ptr modelPointNormal;
	pcl::PointCloud<pcl::PointNormal>::Ptr aligned;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr visual;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr currentStepModel;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr nextStepModel;

	pcl::CorrespondencesPtr corrs;
	

	PCDHelper pread;
	SQLConnect sqlCon;
	MYSQL* connection;

	int filesSaved;
	int selectedModel;
	int maxSteps;
	int currStep;
	int sceneFound;

	bool hasUpdate;
	bool trackingActive;
	bool useEstimate;

	Eigen::Matrix4f initialTransform;

	const std::string snowcat = "steps/snowcat_step_";
	const std::string pyramid = "steps/pyramid_step_";
	const std::string quacktro = "steps/duck_step_";
	const std::string jay = "steps/letterJ_step_";
	const std::string heart = "steps/heart_step_";
	const float leafsize = 0.005f;

public:
	Recognizer();
	~Recognizer();
	void recognizeState(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene);
	void getCloudToCompare(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr saved);
	void centerCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
	void computeNormals(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::Normal>::Ptr normals, float val);
	void computePointNormals(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, float val);
	void downsample(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outputCloud, float leafsize);
	void obtainKeypoints(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints, float radius);
	void computeDescriptor(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::SHOT352>::Ptr descriptors, float radius);
	void findCorrespondences();
	void clusterCorrespondences(float binSize, float thresh);
	void computeReferenceFrames(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::ReferenceFrame>::Ptr rf, float radius);
	void print4x4Matrix(const Eigen::Matrix4f & matrix);
	void estimatePose();
	void performICP();
	void convertRGBAtoPointNormal(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input, pcl::PointCloud<pcl::PointNormal>::Ptr output);
	void init();
	void lowerVisibleArea(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  cloud, std::string axis, float min, float max);
};

#endif