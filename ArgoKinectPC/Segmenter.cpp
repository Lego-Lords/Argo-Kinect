#include "stdafx.h"
#include "Segmenter.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <string>
#include <pcl/point_types_conversion.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>


Segmenter::Segmenter() {
}


Segmenter::~Segmenter() {
}


void Segmenter::segmentCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer) {
	//initialize data so it can be used in other functions, point class ptrs to actual ptrs in main
	this->input = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();; 
	

	/*************** Segmentation Pipeline ***************/
	//downsample, lower the resolution of the kinect so it would be faster to process
	//downsampleCloud();
	if (input->size() > 0) {
		*(this->input) = *input;//input is what functions use
		this->output = output; //output is result of the function


		this->viewer = viewer; //for visualization

		*output = *input;

		//downsampleCloud();

		//crop the area that the kinect can see
		lowerVisibleArea("z", 0.2, 0.8);
		lowerVisibleArea("x", -0.2, 0.2);

		alignPlaneToAxis();

		//remove largest plane
		//segmentPlane();

		//remove lighting
		normalizeColor();

		//remove skin/iwan lang bricks
		isolateBricksCloud();
		//extractBricks();
		
		//remove outliers, mga fake colors
		removeOutliers();

		centerCloud(this->input);


		/*std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>  trial;
		if (this->input->size() > 0) {
			input_noA = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
			pcl::copyPointCloud(*this->input, *input_noA);
			regionSegment(input_noA, trial);
		}*/


		*output = *(this->input);
	}

	//std::cout << "final size: " << output->size() << std::endl;
}

void Segmenter::lowerVisibleArea(std::string axis, float min, float max) {
	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setInputCloud(input);
	pass.setFilterFieldName(axis);
	pass.setFilterLimits(min, max);
	pass.filter(*output);
	*input = *output;
}

void Segmenter::downsampleCloud() {
	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*input, *cloud_blob);
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud_blob);
	sor.setLeafSize(0.01f, 0.01f, 0.01f); //0.01 = 1 cm, 0.001 = 1 mm
	sor.filter(*cloud_filtered_blob);
	pcl::fromPCLPointCloud2(*cloud_filtered_blob, *output);
	*input = *output;
}

void Segmenter::alignPlaneToAxis() {
	if (input->size() > 0) {
		centerCloud(input);

		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::SACSegmentation<pcl::PointXYZRGBA> seg;

		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(100);
		seg.setDistanceThreshold(0.006);

		seg.setInputCloud(input);
		seg.segment(*inliers, *coefficients);

		Eigen::Matrix<float, 1, 3> floor_plane_normal_vector, xz_plane_normal_vector, rotation_vector, another_one;

		floor_plane_normal_vector[0] = coefficients->values[0];
		floor_plane_normal_vector[1] = coefficients->values[1];
		floor_plane_normal_vector[2] = coefficients->values[2];


		xz_plane_normal_vector[0] = 0.0;
		xz_plane_normal_vector[1] = 1.0;
		xz_plane_normal_vector[2] = 0.0;

		rotation_vector = xz_plane_normal_vector.cross(floor_plane_normal_vector);
		//float theta
		float theta = -atan2(rotation_vector.norm(), xz_plane_normal_vector.dot(floor_plane_normal_vector));

		//float theta = acos(floor_plane_normal_vector.dot(xz_plane_normal_vector) / sqrt(pow(coefficients->values[0], 2) + pow(coefficients->values[1], 2) + pow(coefficients->values[2], 2)));

		Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
		transform_2.translation() << 0, 0, 0;
		transform_2.rotate(Eigen::AngleAxisf(theta, rotation_vector.normalized()));
		//transform_2.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()));
		//std::cout << "Transformation matrix: " << std::endl << transform_2.matrix() << std::endl;

		//TODO: rotate again to xz plane 

		pcl::transformPointCloud(*input, *output, transform_2);
		*input = *output;

		transform_2 = Eigen::Affine3f::Identity();
		transform_2.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));
		pcl::transformPointCloud(*input, *output, transform_2);
		*input = *output;
	}
}

int Segmenter::mod(int a, int b) {
	int ret = a % b;
	if (ret < 0)
		ret += b;
	return ret;
}



void Segmenter::isolateBricks() {
	//init
	input_noA = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	pcl::copyPointCloud(*input, *input_noA);
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>  redBricks;
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>  greenBricks;
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>  blueBricks;
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>  yellowBricks;

	//filter per color
	numBricks = 0;
	numBricks += filterColorBricks(redBricks, 255, 70, 69, 0, 69, 0);
	numBricks += filterColorBricks(greenBricks, 100, 0, 255, 101, 100, 0);
	numBricks += filterColorBricks(blueBricks, 79, 0, 100, 0, 255, 101);
	numBricks += filterColorBricks(yellowBricks, 255, 101, 255, 101, 100, 0);

	

	//std::cout << "Num RGBY Final Bricks: " << numBricks << std::endl;
	//if (!viewer->updatePointCloud(output, "normalized")) {
		//viewer->addPointCloud(output, "normalized");
	//}

	/*output->clear();
	ddCloudsToBigCloud(output, redBricks);
	addCloudsToBigCloud(output, greenBricks);
	addCloudsToBigCloud(output, blueBricks);
	addCloudsToBigCloud(output, yellowBricks);

	//std::cout << "cloud size: " << output->size() << std::endl;
	*input = *output;
	*/

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> segmented_cloud_color_handler(output, 255, 0, 0);
	//if (!viewer->updatePointCloud(output, segmented_cloud_color_handler, "segmented_cloud")) {
		//viewer->addPointCloud(output, segmented_cloud_color_handler, "segmented_cloud");
	//}
}

int Segmenter::filterColorBricks(std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &filtered_bricks, int rMax, int rMin, int gMax, int gMin, int bMax, int bMin) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr dummyNoAFiltered = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr singlecloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
	pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::LT, rMax)));
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::GT, rMin)));
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::LT, gMax)));
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::GT, gMin)));
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::LT, bMax)));
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::GT, bMin)));

	// build the filter 
	pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
	condrem.setCondition(color_cond);
	condrem.setInputCloud(input_noA);

	// apply filter 
	condrem.filter(*dummyNoAFiltered);

	pcl::copyPointCloud(*dummyNoAFiltered, *singlecloud);
	if (singlecloud->size() > 0) {
		return clusterBricks(singlecloud, filtered_bricks);
	}
		
	return 0;
}

void Segmenter::normalizeColor() {
	for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = input->begin(); it != input->end(); it++) {
		float total = it->r + it->g + it->b;
		it->r = it->r / total * 255;
		it->g = it->g / total * 255;
		it->b = it->b / total * 255;
	}
	*output = *input;
}

void Segmenter::removeOutliers() {
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
	if (input->size() > 0) {
		//std::cout << "final size: " << output->size() << std::endl;
		sor.setInputCloud(input);
		sor.setMeanK(50);
		sor.setStddevMulThresh(1.0);
		sor.filter(*output);
		*input = *output;
	}
}

void Segmenter::centerCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {

	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud, centroid);
	//std::cout << "Centroid: " << centroid << std::endl;

	Eigen::Affine3f tMatrix = Eigen::Affine3f::Identity();
	tMatrix.translate(centroid.head<3>());

	pcl::transformPointCloud(*cloud, *cloud, tMatrix.inverse());
}

int Segmenter::clusterBricks(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &cloudclusters) {
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
	ec.setClusterTolerance(0.01); // 2cm
	ec.setMinClusterSize(30);
	ec.setMaxClusterSize(800);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(cloud->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		cloudclusters.push_back(cloud_cluster);
		j++;
	}
	//std::cout << "Num Bricks: " << j << std::endl;
	return j;

}

void Segmenter::addCloudsToBigCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &cloudclusters) {
	for (std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>::const_iterator it = cloudclusters.begin(); it != cloudclusters.end(); ++it)
	{
		*cloud += **it;
	}
}


void Segmenter::segmentPlane() {
	//Remove the plane (floor/table)
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.006);

	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	int i = 0, nr_points = (int)input->points.size();

	// Form the largest planar component from the cloud
	seg.setInputCloud(input);
	seg.segment(*inliers, *coefficients);

	extract.setInputCloud(input);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*output);
	*input = *output;
}

void Segmenter::regionSegment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &cloudclusters) {
	pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> >(new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
	reg.setInputCloud(cloud);
	reg.setSearchMethod(tree);
	reg.setDistanceThreshold(6);
	reg.setPointColorThreshold(5);
	reg.setRegionColorThreshold(5);
	reg.setMinClusterSize(30);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_clusterA(new pcl::PointCloud<pcl::PointXYZRGBA>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(cloud->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		pcl::copyPointCloud(*cloud_cluster, *cloud_clusterA);
		cloudclusters.push_back(cloud_clusterA);
		j++;
	}
	std::cout << "Num Region Bricks: " << j << std::endl;

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	pcl::visualization::CloudViewer viewer("Cluster viewer");
	viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped())
	{
		boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}
}


void Segmenter::isolateBricksCloud() {
	//init
	input_noA = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	pcl::copyPointCloud(*input, *input_noA);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  redBricks(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  greenBricks(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  blueBricks(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  yellowBricks(new pcl::PointCloud<pcl::PointXYZRGBA>);

	//filter per color
	filterColorBricksCloud(redBricks, 255, 70, 69, 0, 69, 0);
	filterColorBricksCloud(greenBricks, 100, 0, 255, 101, 100, 0);
	filterColorBricksCloud(blueBricks, 79, 0, 100, 0, 255, 101);
	filterColorBricksCloud(yellowBricks, 255, 101, 255, 101, 100, 0);



	//if (!viewer->updatePointCloud(output, "normalized")) {
	//viewer->addPointCloud(output, "normalized");
	//}

	output->clear();
	*output += *redBricks;
	*output += *greenBricks;
	*output += *blueBricks;
	*output += *yellowBricks;

	*input = *output;

	
	//std::cout << "cloud size: " << output->size() << std::endl;
	

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> segmented_cloud_color_handler(output, 255, 0, 0);
	//if (!viewer->updatePointCloud(output, segmented_cloud_color_handler, "segmented_cloud")) {
	//viewer->addPointCloud(output, segmented_cloud_color_handler, "segmented_cloud");
	//}
}

void Segmenter::filterColorBricksCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered, int rMax, int rMin, int gMax, int gMin, int bMax, int bMin) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr dummyNoAFiltered = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

	pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::LT, rMax)));
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::GT, rMin)));
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::LT, gMax)));
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::GT, gMin)));
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::LT, bMax)));
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::GT, bMin)));

	// build the filter 
	pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
	condrem.setCondition(color_cond);
	condrem.setInputCloud(input_noA);

	// apply filter 
	condrem.filter(*dummyNoAFiltered);

	pcl::copyPointCloud(*dummyNoAFiltered, *filtered);
}
