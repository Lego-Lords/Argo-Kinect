#ifndef RECOGNIZE_H
#define RECOGNIZE_H
#include "stdafx.h"
#include "PCDHelper.h"
#include "SQLConnect.h"
#include <flann/flann.h>
//#include <flann/io/hdf5.h>

typedef std::pair<std::string, std::vector<float> > vfh_model;

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
	

	PCDHelper helper;
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
	bool hasError = false;

	Eigen::Matrix4f initialTransform;

	const std::string base_dir = "features/";

	const std::string snowcat = "snowcat";
	const std::string pyramid = "pyramid";
	const std::string quacktro = "duck";
	const std::string jay = "letterj";
	const std::string heart = "heart";
	const float leafsize = 0.005f;

	const std::string models_hd5_filename = "features/models.hd5";
	const std::string models_list_filename = "features/models.list";
	const std::string kdtree_filename = base_dir + "kdtree.idx";

	std::vector<vfh_model> compModels;
	std::vector<vfh_model> nextStepModels;
	std::vector<vfh_model> currStepModels;

	flann::Matrix<float> data;
	vfh_model lowest;
	bool moveToNextStep = false;

	int numOfCandidates; //ilan yung kukunin niyang (model with angle)
	int acceptedScore;
	int numOfIter;
	int currIter;
	int currAcceptedCandidateIter;
	int currAcceptedCandidateIndex;
	string acceptedCandidates[10][100];
	int acceptedCandidatesDistance[10][100];
	bool hasAddedCandidate = false;
	map<string, unsigned int> acceptedCandidatesFreq;
	map<string, unsigned int> acceptedCandidatesDistanceMap;
	map<string, float> acceptedCandidatesAverageMap;
	string lowestCandidate;

	//used by Kingston
	float acceptanceThreshold;
	float threshForNormal;
	int numOfIteration;
	int currentIteration;
	std::map<std::string, std::pair<float, int> > myMultiValueMap;
	float leastAverage;
	string finalAnswer;
	float distanceThreshold;
	float minOccurPercent;

	int lowerThreshForSmall;
	float threshForSmall;
	float minOccurPercentForSmall;
	int sizeSmallClouds;
	float cloudSizePercent;

	float cloudSizeRejection;
	

	int isStepByStep;
	string typeOfTest;

	int useLogs;
	ofstream resultsFile;
	string resultsfilename;

	bool initResultsFile;

	int cyclesTaken;
	std::clock_t start;
	double duration;
	double updateduration;
	double totalduration;

	int showIterVals;
	int showSceneSize;
	int useServer;
	float aveSizeOfNext;
	bool initAssembly;
	string selectmodelname;
	int bin_size;

	string db_add;
	string db_user;
	string db_pass;
	string db_name;

	std::pair<string, float> bestCandidate;
	std::pair<string, float> secondChoiceKaLang;

	std::pair<string, string> savedBestCandidate;
	std::pair<string, string> saved2ndCandidate;


public:
	Recognizer();
	~Recognizer();
	void recognizeState(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene);
	void computeVFHFeatures(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::VFHSignature308>::Ptr outputHist);
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
	void loadHistogramsFromFiles();
	void loadVFHs(const boost::filesystem::path &file_dir, std::string stepfile, std::vector<vfh_model> &models);
	void nearestKSearch(flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model,
		int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances);
	vector<string> split(string str, char delimiter);
	void initValuesFromFile();
	float round4f(float f);
	double round4d(double d);
	float computeAveOfStep(int step);

	int isolateBricks(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
	int filterColorBricks(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &filtered_bricks, int rMax, int rMin, int gMax, int gMin, int bMax, int bMin);
	int clusterBricks(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &cloudclusters);
	void createColorHistogram(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector<float> &hist);
};

#endif