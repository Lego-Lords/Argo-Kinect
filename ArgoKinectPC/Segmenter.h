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
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr removePlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
};

#endif

                               