#ifndef KINECT_H
#define KINECT_H
#include "stdafx.h"
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
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pOutput;

	//for argo
	int selectedModel;
	int maxSteps;
	int currStep;
	bool updated;


public:
	Kinect();
	~Kinect();
	void run();
	void initialize();
	void initializeSensor();
	void initializeColor();
	void initializeDepth();
	void initializePointCloud();
	void finalize();
	void update();
	void updateColor();
	void updateDepth();
	void updatePointCloud();
	void draw();
	void drawPointCloud();
	void show();
	void showPointCloud();
	void segment();
};

#endif

