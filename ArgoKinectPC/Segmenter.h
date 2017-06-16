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
		const int DEPTH_WIDTH = 512;
		const int DEPTH_HEIGHT = 424;

	public:
		Segmenter();
		~Segmenter();
		void segmentCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
		void lowerVisibleArea();
		void downsampleCloud();
		void segmentPlane();
		void isolateBricks();
		void filterColorBricks(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered, int rMax, int rMin, int gMax, int gMin, int bMax, int bMin);
		void normalizeColor();
		void removeOutliers();
};

#endif

