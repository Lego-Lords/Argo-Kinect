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

	public:
		Segmenter();
		~Segmenter();
		void segmentCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
		void lowerVisibleArea(std::string axis, float min, float max);
		void downsampleCloud();
		void segmentPlane();
		void isolateBricks();
		void filterColorBricks(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered, int rMax, int rMin, int gMax, int gMin, int bMax, int bMin);
		void normalizeColor();
		void removeOutliers();
		void extractBricks();
		void filterHue(pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud, int hue, int hue_threshold, pcl::PointCloud<pcl::PointXYZHSV>::Ptr output);
		void PointCloudXYZHSVtoXYZRGBA(pcl::PointCloud<pcl::PointXYZHSV>& in, pcl::PointCloud<pcl::PointXYZRGBA>& output);
		void filterBrickColor(pcl::PointCloud<pcl::PointXYZHSV>::Ptr filtered, int h, int h_thresh, int sMax, int sMin, int vMax, int vMin);
		void createConcaveHull(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output);
		int mod(int a, int b);
};

#endif

