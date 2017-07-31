#include "stdafx.h"
#include "Trainer.h"
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>


Trainer::Trainer() {
	extension = ".pcd";
	save_dir = "features/";
	file_dir = "templates/";
	model_name = "duck/";
}


Trainer::~Trainer() {
}

void Trainer::convertToHists() {
	std::string read_dir = file_dir + model_name;
	loadClouds(read_dir, extension);

}

void Trainer::loadClouds(const boost::filesystem::path &file_dir, std::string extension) {
	if (!boost::filesystem::exists(file_dir) && !boost::filesystem::is_directory(file_dir))
		return;

	for (boost::filesystem::directory_iterator it(file_dir); it != boost::filesystem::directory_iterator(); ++it) {
		if (boost::filesystem::is_directory(it->status())) {
			std::stringstream ss;
			ss << it->path();
			//pcl::console::print_highlight("Loading %s (%lu models loaded so far).\n", ss.str().c_str(), (unsigned long)models.size());
			loadClouds(it->path(), extension);
		}
		if (boost::filesystem::is_regular_file(it->status()) && boost::filesystem::extension(it->path()) == extension) {
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
			pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>());

			helper.readPCD(it->path().string(), cloud);
			computeVFHFeatures(cloud, vfhs);
			helper.saveVFHinPCD(save_dir + model_name + it->path().filename().string(), vfhs);
			//vfh_model m;
			//if (loadHist(base_dir / it->path().filename(), m))
				//models.push_back(m);
		}
	}
}

void Trainer::computeVFHFeatures(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs) {

	//compute normals of cloud
	pcl::PointCloud<pcl::Normal>::Ptr normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
	computeNormals(inputCloud, normals, 10);


	pcl::VFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud(inputCloud);
	vfh.setInputNormals(normals);
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
	vfh.setSearchMethod(tree);

	// Compute the features
	vfh.compute(*vfhs);
}

void Trainer::computeNormals(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::Normal>::Ptr normals, float val) {
	pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> norm_est;
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(val);
	norm_est.setInputCloud(inputCloud);
	norm_est.compute(*normals);
}

void Trainer::downsampleCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input) {
	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*input, *cloud_blob);
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud_blob);
	sor.setLeafSize(0.01f, 0.01f, 0.01f); //0.01 = 1 cm, 0.001 = 1 mm
	sor.filter(*cloud_filtered_blob);
	pcl::fromPCLPointCloud2(*cloud_filtered_blob, *input);
}
