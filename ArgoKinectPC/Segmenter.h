#ifndef SEGMENT_H
#define SEGMENT_H
#include "stdafx.h"
#include <wrl/client.h>
using namespace Microsoft::WRL;

class Segmenter {
	private:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr input;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr output;

	public:
		Segmenter();
		~Segmenter();
		void segmentCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);
		void lowerVisibleArea();
		void downsampleCloud();
		void segmentPlane();
		void removeSkinPixels();
};

#endif

