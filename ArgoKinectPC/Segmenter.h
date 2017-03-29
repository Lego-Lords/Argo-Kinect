#ifndef SEGMENT_H
#define SEGMENT_H
#include "stdafx.h"
#include <wrl/client.h>
using namespace Microsoft::WRL;
class Segmenter
{

public:
	Segmenter();
	~Segmenter();
	pcl::PointCloud<pcl::PointXYZRGB> voxelize(pcl::PointCloud<pcl::PointXYZRGB> cloud);
	pcl::PointCloud<pcl::PointXYZRGB> removePlane(pcl::PointCloud<pcl::PointXYZRGB> cloud);
	pcl::PointCloud<pcl::PointXYZRGB> segment(pcl::PointCloud<pcl::PointXYZRGB> cloud);
};

#endif

                               