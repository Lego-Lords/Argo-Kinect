#ifndef TRAIN_H
#define TRAIN_H
#include "stdafx.h"
#include <wrl/client.h>
#include "PCDHelper.h"
using namespace Microsoft::WRL;

class Trainer {
	private:
		std::string extension;
		std::string save_dir;
		std::string file_dir;
		std::string model_name;
		PCDHelper helper;

	public:
		Trainer();
		~Trainer();
		void convertToHists();
		void loadClouds(const boost::filesystem::path &file_dir, std::string extension);
		void computeNormals(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::Normal>::Ptr normals, float val);
		void computeVFHFeatures(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs);
		void downsampleCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud);
};

#endif

