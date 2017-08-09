#include "stdafx.h"
#include "Trainer.h"
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>


Trainer::Trainer() {
	extension = ".pcd";
	save_dir = "vfhrgb/";
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
	
	//cout << "resize ok ";
	ofstream newFeatures;
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

			cout << "reading " << it->path().string() << std::endl;
			std::vector<float> hist;
			hist.resize(100);
			computeVFHFeatures(cloud, vfhs);
			
			createColorHistogram(cloud, hist);
			
			std::string dirfile = save_dir + model_name + it->path().filename().string();
			size_t lastindex = dirfile.find_last_of(".");
			std::string rawname = dirfile.substr(0, lastindex);
			newFeatures.open(rawname + ".txt", std::ios_base::app);
			for (int i = 0; i < 308; i++) {
				newFeatures << vfhs->points[0].histogram[i] << std::endl;
			}
			for (int i = 0; i < 100; i++) {
				newFeatures << hist[i] << std::endl;
			}
			newFeatures.close();
			//helper.saveVFHinPCD(save_dir + model_name + it->path().filename().string(), vfhs);
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

void Trainer::createColorHistogram(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector<float> &hist) {
	int nbins = 100;
	double bin_interval = 3 * 256.0 / nbins;
	double bin_interval_h = 3 * 360.0 / nbins;
	double bin_interval_s = 3 * 1.0 / nbins;
	double bin_interval_v = 3 * 1.0 / nbins;

	//Initialize a temp histogram with zeros
	std::vector<double> color;
	color.resize(nbins);
	for (int i = 0; i< nbins; i++)
		color[i] = 0.0;

	//Keep binning the histogram based on the color of the points in cloud
	double R = 0, G = 0, B = 0;
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		R = cloud->points[i].r;
		G = cloud->points[i].g;
		B = cloud->points[i].b;
		int ind;
		double r, g, b, h, s, v, d;
		double min_, max_;
		r = (double)R / 255.0;
		g = (double)G / 255.0;
		b = (double)B / 255.0;
		min_ = std::min(r, std::min(g, b));
		max_ = std::max(r, std::max(g, b));
		if (max_ == min_)
		{
			v = max_;
			h = 0;
			s = 0;
		}
		else
		{
			d = (r == min_) ? g - b : ((b == min_) ? r - g : b - r);
			h = (r == min_) ? 3 : ((b == min_) ? 1 : 5);
			h = 60 * (h - d / (max_ - min_));
			s = (max_ - min_) / max_;
			v = max_;
		}
		ind = h / bin_interval_h;
		color[ind]++;
		ind = s / bin_interval_s;
		color[ind + nbins / 3]++;
		ind = v / bin_interval_v;
		color[ind + 2 * nbins / 3]++;
	}

	for (int i = 0; i < nbins; i++)
	{

		hist[i] = color[i] / cloud->points.size();
	}
}
