#ifndef SEGMENT_H
#define SEGMENT_H
#include "stdafx.h"
#include <wrl/client.h>
using namespace Microsoft::WRL;

class Segmenter {
	private:
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output;

	public:
		Segmenter();
		~Segmenter();
		void segmentCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output);
		void lowerVisibleArea();
		void downsampleCloud();
		void segmentPlane();
		pcl::PointCloud<pcl::PointXYZRGB> voxelize(pcl::PointCloud<pcl::PointXYZRGB> cloud);
		pcl::PointCloud<pcl::PointXYZRGB> removePlane(pcl::PointCloud<pcl::PointXYZRGB> cloud);
		pcl::PointCloud<pcl::PointXYZRGB> segment(pcl::PointCloud<pcl::PointXYZRGB> cloud);
};

#endif

