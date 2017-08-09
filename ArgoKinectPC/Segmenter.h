#ifndef SEGMENT_H
#define SEGMENT_H
#include "stdafx.h"
#include <wrl/client.h>
using namespace Microsoft::WRL;

class Segmenter {
	private:
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output;
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_noA;
		
		pcl::PointCloud<pcl::PointXYZHSV>::Ptr input_hsv;
		const int DEPTH_WIDTH = 512;
		const int DEPTH_HEIGHT = 424;

		int numBricks;

	public:
		Segmenter();
		~Segmenter();
		void segmentCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
		void lowerVisibleArea(std::string axis, float min, float max);
		void downsampleCloud();
		void segmentPlane();
		void alignPlaneToAxis();
		void centerCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
		void isolateBricks();
		int filterColorBricks(std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &filtered_bricks, int rMax, int rMin, int gMax, int gMin, int bMax, int bMin);
		void normalizeColor();
		void removeOutliers();
		int mod(int a, int b);
		int clusterBricks(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &cloudclusters);
		void addCloudsToBigCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &cloudclusters);
		void regionSegment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &cloudclusters);
		void isolateBricksCloud();
		void filterColorBricksCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered, int rMax, int rMin, int gMax, int gMin, int bMax, int bMin);
};

#endif

