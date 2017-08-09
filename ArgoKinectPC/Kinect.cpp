#include "stdafx.h"
#include "Kinect.h"
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/registration/icp.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h> 
#include <pcl/console/parse.h>


typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;


struct CloudStyle {
	double r;
	double g;
	double b;
	double size;

	CloudStyle(double r,
		double g,
		double b,
		double size) :
		r(r),
		g(g),
		b(b),
		size(size) {
	}
};

CloudStyle style_white(255.0, 255.0, 255.0, 4.0);
CloudStyle style_red(255.0, 0.0, 0.0, 3.0);
CloudStyle style_green(0.0, 255.0, 0.0, 5.0);
CloudStyle style_cyan(93.0, 200.0, 217.0, 4.0);
CloudStyle style_violet(255.0, 0.0, 255.0, 8.0);


// Error Check
#define ERROR_CHECK( ret )                                        \
	if( FAILED( ret ) ){                                          \
		std::stringstream ss;                                     \
		ss << "failed " #ret " " << std::hex << ret << std::endl; \
		throw std::runtime_error( ss.str().c_str() );             \
	}

	// Constructor
Kinect::Kinect() {
	// Initialize
	initialize();
}

// Destructor
Kinect::~Kinect() {
	// Finalize
	finalize();
}

// Processing
void Kinect::run() {
	while (!viewer->wasStopped()) {
		string filename;
		bool save = false;
		if (pOutput->size() > 0 && training) {
			
			cout << "input filename: ";
			cin >> filename;
			save = true;
		}

		// Update Data
		update();
		//int age;
		//cin >> age;

		//Segment Data oh yeah wubalubadubdub
		if (cloud->size() > 0)
		{ 
			segmenter.segmentCloud(cloud, pOutput, viewer);

			if (pOutput->size() > 0 && training && save) {
				helper.savePCD(filename, pOutput);
				pictureViewer->updatePointCloud(pOutput, "picture");
				save = false;
			}
			//helper.readPCD("features/duck_step_4_0.pcd", pOutput);
			if (pOutput->size() > 0 && !training && matching)
				recognizer.recognizeState(pOutput);
		}


		// Draw Data
		draw();

		// Show Data
		show();


	}
}

// Initialize
void Kinect::initialize() {
	// Initialize Sensor
	initializeSensor();

	// Initialize Color
	initializeColor();

	// Initialize Depth
	initializeDepth();

	// Initialize Point Cloud
	initializePointCloud();

	initializeArgo();


}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
	if (event.getKeySym() == "r" && event.keyDown())
		std::cout << "'r' was pressed" << std::endl;
	if (event.getKeySym() == "h" && event.keyDown())
		std::cout << "'h' was pressed" << std::endl;
}


void Kinect::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	if (event.getKeySym() == "space" && event.keyDown())
	{
		std::cout << "Space is pressed" << std::endl;
		saveCloud = true;
	}
}

// Initialize Sensor
inline void Kinect::initializeSensor() {
	// Open Sensor
	ERROR_CHECK(GetDefaultKinectSensor(&kinect));

	ERROR_CHECK(kinect->Open());

	// Check Open
	BOOLEAN isOpen = FALSE;
	ERROR_CHECK(kinect->get_IsOpen(&isOpen));
	if (!isOpen) {
		throw std::runtime_error("failed IKinectSensor::get_IsOpen( &isOpen )");
	}

	// Retrieve Coordinate Mapper
	ERROR_CHECK(kinect->get_CoordinateMapper(&coordinateMapper));
}

// Initialize Color
inline void Kinect::initializeColor() {
	// Open Color Reader
	ComPtr<IColorFrameSource> colorFrameSource;
	ERROR_CHECK(kinect->get_ColorFrameSource(&colorFrameSource));
	ERROR_CHECK(colorFrameSource->OpenReader(&colorFrameReader));

	// Retrieve Color Description
	ComPtr<IFrameDescription> colorFrameDescription;
	ERROR_CHECK(colorFrameSource->CreateFrameDescription(ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription));
	ERROR_CHECK(colorFrameDescription->get_Width(&colorWidth)); // 1920
	ERROR_CHECK(colorFrameDescription->get_Height(&colorHeight)); // 1080
	ERROR_CHECK(colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel)); // 4

																				// Allocation Color Buffer
	colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);
}

// Initialize Depth
inline void Kinect::initializeDepth() {
	// Open Depth Reader
	ComPtr<IDepthFrameSource> depthFrameSource;
	ERROR_CHECK(kinect->get_DepthFrameSource(&depthFrameSource));
	ERROR_CHECK(depthFrameSource->OpenReader(&depthFrameReader));

	// Retrieve Depth Description
	ComPtr<IFrameDescription> depthFrameDescription;
	ERROR_CHECK(depthFrameSource->get_FrameDescription(&depthFrameDescription));
	ERROR_CHECK(depthFrameDescription->get_Width(&depthWidth)); // 512
	ERROR_CHECK(depthFrameDescription->get_Height(&depthHeight)); // 424
	ERROR_CHECK(depthFrameDescription->get_BytesPerPixel(&depthBytesPerPixel)); // 2

																				// Allocation Depth Buffer
	depthBuffer.resize(depthWidth * depthHeight);
}

// Initialize Point Cloud
inline void Kinect::initializePointCloud() {
	// Create Point Cloud
	cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
	pOutput = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
	cloud->width = static_cast<uint32_t>(depthWidth);
	cloud->height = static_cast<uint32_t>(depthHeight);
	cloud->points.resize(cloud->height * cloud->width);
	cloud->is_dense = false;

	// Create PCLVisualizer
	viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("Point Cloud Viewer");

	// Initialize camera position
	viewer->setCameraPosition(0.0, 0.0, -1.0, 0.0, 0.0, 0.0);

	// Add Coordinate System
	//viewer->addCoordinateSystem(0.1);
	viewer->addPointCloud(cloud, "cloud");
}

// Finalize
void Kinect::finalize() {
	// Close Sensor
	if (kinect != nullptr) {
		kinect->Close();
	}
}

// Update Data
void Kinect::update() {
	// Update Color
	updateColor();

	// Update Depth
	updateDepth();

	// Update Point Cloud
	updatePointCloud();
}

// Update Color
inline void Kinect::updateColor() {
	// Retrieve Color Frame
	ComPtr<IColorFrame> colorFrame;
	const HRESULT ret = colorFrameReader->AcquireLatestFrame(&colorFrame);
	if (FAILED(ret)) {
		return;
	}

	// Convert Format ( YUY2 -> BGRA )
	ERROR_CHECK(colorFrame->CopyConvertedFrameDataToArray(static_cast<UINT>(colorBuffer.size()), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra));
}

// Update Depth
inline void Kinect::updateDepth() {
	// Retrieve Depth Frame
	ComPtr<IDepthFrame> depthFrame;
	const HRESULT ret = depthFrameReader->AcquireLatestFrame(&depthFrame);
	if (FAILED(ret)) {
		return;
	}

	// Retrieve Depth Data
	ERROR_CHECK(depthFrame->CopyFrameDataToArray(static_cast<UINT>(depthBuffer.size()), &depthBuffer[0]));
}

// Update Point Cloud
inline void Kinect::updatePointCloud() {
	// Reset Point Cloud
	cloud->clear();

	// Convert to Point Cloud
	for (int depthY = 0; depthY < depthHeight; depthY++) {
		for (int depthX = 0; depthX < depthWidth; depthX++) {
			pcl::PointXYZRGBA point;

			// Retrieve Mapped Coordinates
			DepthSpacePoint depthSpacePoint = { static_cast<float>(depthX), static_cast<float>(depthY) };
			UINT16 depth = depthBuffer[depthY * depthWidth + depthX];
			ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };
			ERROR_CHECK(coordinateMapper->MapDepthPointToColorSpace(depthSpacePoint, depth, &colorSpacePoint));

			// Set Color to Point
			int colorX = static_cast<int>(colorSpacePoint.X + 0.5f);
			int colorY = static_cast<int>(colorSpacePoint.Y + 0.5f);
			if ((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)) {
				unsigned int colorIndex = (colorY * colorWidth + colorX) * colorBytesPerPixel;
				point.b = colorBuffer[colorIndex + 0];
				point.g = colorBuffer[colorIndex + 1];
				point.r = colorBuffer[colorIndex + 2];
				point.a = colorBuffer[colorIndex + 3];
			}

			// Retrieve Mapped Coordinates
			CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
			ERROR_CHECK(coordinateMapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint));

			// Set Depth to Point
			if ((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)) {
				point.x = cameraSpacePoint.X;
				point.y = cameraSpacePoint.Y;
				point.z = cameraSpacePoint.Z;
			}

			// Set Point to Point Cloud
			cloud->push_back(point);
		}
	}
}

// Draw Data
void Kinect::draw() {
	// Draw Point Cloud
	drawPointCloud();
}

// Draw Point Cloud
inline void Kinect::drawPointCloud() {

	if (!viewer->updatePointCloud(pOutput, "cloud")/*!viewer->updatePointCloud(pOutput, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>(pOutput, 255.0, 255.0, 255.0), "cloud")*/) {
		viewer->addPointCloud(pOutput, "cloud");
	}
}

// Show Data
void Kinect::show() {
	// Show Point Cloud
	showPointCloud();
}

// Show Point Cloud
inline void Kinect::showPointCloud() {
	// Update Viwer
	viewer->spinOnce();
}

void Kinect::initializeArgo() {
	selectedModel = 0;
	maxSteps = 0;
	currStep = 5;
	updated = true;
	noSelect = true;
	saveCloud = true;
	
	filesSaved = 1;
	training = false;
	featureExtraction = false;
	matching = true;
	sceneFound = 0;

	if (training) {
		pictureViewer = boost::make_shared<pcl::visualization::PCLVisualizer>("Picture Taking Viewer");
		pictureViewer->setCameraPosition(0.0, 0.0, -1.0, 0.0, 0.0, 0.0);
		pictureViewer->addCoordinateSystem(0.1);
		pictureViewer->addPointCloud(cloud, "picture");
	}
	
	if (featureExtraction) {
		cout << "welcome to feature extraction ";
		trainer.convertToHists();
	}
}



