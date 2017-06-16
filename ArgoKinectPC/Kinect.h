#ifndef KINECT_H
#define KINECT_H
#include "stdafx.h"
#include "Segmenter.h"
#include "Recognizer.h"
#include "Kinect.h"
#include "SQLConnect.h"
#include "PCDReader.h"
#include <wrl/client.h>
using namespace Microsoft::WRL;
class Kinect
{
private:
	// Sensor
	ComPtr<IKinectSensor> kinect;

	// Coordinate Mapper
	ComPtr<ICoordinateMapper> coordinateMapper;

	// Reader
	ComPtr<IColorFrameReader> colorFrameReader;
	ComPtr<IDepthFrameReader> depthFrameReader;

	// Color Buffer
	std::vector<BYTE> colorBuffer;
	int colorWidth;
	int colorHeight;
	unsigned int colorBytesPerPixel;

	// Depth Buffer
	std::vector<UINT16> depthBuffer;
	int depthWidth;
	int depthHeight;
	unsigned int depthBytesPerPixel;

	// PCL
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pOutput;

	//for argo
	Segmenter segmenter;
	Recognizer recognizer;

	int filesSaved;
	int selectedModel;
	int maxSteps;
	int currStep;
	bool updated, noSelect, saveCloud;
	std::string stepfile;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudAgainst;
	SQLConnect con;
	PCDReader pread;
	MYSQL * connection;
	bool datagathering;
	bool matching;
	int sceneFound;

public:
	Kinect();
	~Kinect();
	void run();
	void initialize();
	void initializeSensor();
	void initializeColor();
	void initializeDepth();
	void initializePointCloud();
	void initializeArgo();
	void finalize();
	void update();
	void updateColor();
	void updateDepth();
	void updatePointCloud();
	void draw();
	void drawPointCloud();
	void show();
	void showPointCloud();
	void Kinect::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void * viewer_void);
	void match();
	void checkSave();
};

#endif

