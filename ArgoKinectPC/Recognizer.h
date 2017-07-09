#pragma once
#include "PCDReader.h"
#include "SQLConnect.h"

class Recognizer {
private:
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_icp;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	pcl::PointCloud<pcl::Normal>::Ptr model_normals;
	pcl::PointCloud<pcl::Normal>::Ptr scene_normals;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model_keypoints;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_keypoints;

	pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors;
	pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors;

	pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf;
	pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudAgainst;

	pcl::CorrespondencesPtr corrs;

	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustCorrs;


	PCDReader pread;
	SQLConnect sqlCon;
	MYSQL* connection;

	int filesSaved;
	int selectedModel;
	int maxSteps;
	int currStep;
	int sceneFound;

	bool hasUpdate;

	const std::string snowcat = "steps/snowcat_step_";
	const std::string pyramid = "steps/pyramid_step_";
	const std::string quacktro = "steps/duck_step_";

public:
	Recognizer();
	~Recognizer();
	void recognizeState(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
	void getCloudToCompare();
	void computeNormals(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::Normal>::Ptr normals, int numNeighbors);
	void obtainKeypoints(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints, float radius);
	void computeDescriptor(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::SHOT352>::Ptr descriptors, float radius);
	void findCorrespondences();
	void computeReferenceFrames(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::ReferenceFrame>::Ptr rf, float radius);
	void clusterCorrespondences(float binSize, float thresh);
	void print4x4Matrix(const Eigen::Matrix4d & matrix);
	void estimatePose();
	void performICP();
};