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
		isolateBricks();
		//extractBricks();
		
		//remove outliers, mga fake colors
		removeOutliers();

		centerCloud(this->input);
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

void Segmenter::filterHue(pcl::PointCloud<pcl::PointXYZHSV>::Ptr input, int hue, int hue_threshold, pcl::PointCloud<pcl::PointXYZHSV>::Ptr output)
{
	float hue_min = mod(hue - hue_threshold, 360);
	float hue_max = mod(hue + hue_threshold, 360);

	//if (hue_max > )

	for (size_t i = 0; i < input->points.size(); i++) {
		// check to see if we are in range
		if (input->points[i].h < hue_max || input->points[i].h > hue_min)
		{
			output->points.push_back(input->points[i]);
			//std::cout << "hue: " << input->points[i].h << std::endl;
		}
			
	}
}

void Segmenter::extractBricks() {

	//if (!viewer->updatePointCloud(input, "normalized")) {
		//viewer->addPointCloud(input, "normalized");
	//}
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr filtered_bricks(new pcl::PointCloud<pcl::PointXYZHSV>());
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr  redBricks(new pcl::PointCloud<pcl::PointXYZHSV>);
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr  greenBricks(new pcl::PointCloud<pcl::PointXYZHSV>);
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr  blueBricks(new pcl::PointCloud<pcl::PointXYZHSV>);
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr  yellowBricks(new pcl::PointCloud<pcl::PointXYZHSV>);
	input_hsv = boost::make_shared<pcl::PointCloud<pcl::PointXYZHSV>>();
	input_hsv->clear();

	pcl::PointCloudXYZRGBAtoXYZHSV(*input, *input_hsv);
	//filterBrickColor(redBricks, 10, 40, 0.8, 0.0, 1.0, 0.4);
	//filterBrickColor(redBricks, 0, 7, 1.0, 0.8, 1.0, 0.9);
	//std::cout << "red size: " << redBricks->size() << std::endl;
	//filterBrickColor(greenBricks, 20, 5, 0.7, 0.0, 1.0, 0.1);
	filterBrickColor(blueBricks, 240, 10, 1.0, 0.8, 1.0, 0.7);
	//filterBrickColor(yellowBricks, 60, 10, 1.0, 0.9, 1.0, 0.1);
	
	*filtered_bricks += *redBricks;
	*filtered_bricks += *greenBricks;
	*filtered_bricks += *blueBricks;
	*filtered_bricks += *yellowBricks;

	Segmenter::PointCloudXYZHSVtoXYZRGBA(*filtered_bricks, *output);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> segmented_cloud_color_handler(output, 0, 255, 0);
	//if (!viewer->updatePointCloud(output, segmented_cloud_color_handler, "segmented_cloud")) {
		//viewer->addPointCloud(output, segmented_cloud_color_handler, "segmented_cloud");
	//}
	*input = *output;
}

void Segmenter::filterBrickColor(pcl::PointCloud<pcl::PointXYZHSV>::Ptr filtered, int h, int h_thresh, int sMax, int sMin, int vMax, int vMin)
{
	filtered->clear();

	filterHue(input_hsv, h, h_thresh, filtered);
	
	pcl::PassThrough<pcl::PointXYZHSV> pass;
	/*pass.setInputCloud(filtered);
	pass.setFilterFieldName("h");
	pass.setFilterLimits(h_thresh, h);
	pass.setFilterLimitsNegative(false);
	pass.filter(*filtered);*/

	pass.setInputCloud(filtered);
	pass.setFilterFieldName("s");
	pass.setFilterLimits(sMin, sMax);
	pass.setFilterLimitsNegative(false);
	pass.filter(*filtered);

	pass.setInputCloud(filtered);
	pass.setFilterFieldName("v");
	pass.setFilterLimits(vMin, vMax);
	pass.setFilterLimitsNegative(false);
	pass.filter(*filtered);

	std::cout << "filtered size: " << filtered->size() << std::endl;

	
}

void Segmenter::PointCloudXYZHSVtoXYZRGBA(pcl::PointCloud<pcl::PointXYZHSV>& in, pcl::PointCloud<pcl::PointXYZRGBA>& output) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out (new pcl::PointCloud<pcl::PointXYZRGB>());
	out->width = in.width;
	out->height = in.height;
	out->header = in.header;
	for (size_t i = 0; i < in.points.size(); i++) {
		pcl::PointXYZRGB p;
		pcl::PointXYZHSVtoXYZRGB(in.points[i], p);
		p.x = in.points[i].x;
		p.y = in.points[i].y;
		p.z = in.points[i].z;
		out->points.push_back(p);
	}
	pcl::copyPointCloud(*out, output);
	for (size_t i = 0; i < output.size(); i++) {
		output.points[i].a = 255;
	}
}


void Segmenter::isolateBricks() {
	//init
	input_noA = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	pcl::copyPointCloud(*input, *input_noA);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  redBricks(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  greenBricks(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  blueBricks(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  yellowBricks(new pcl::PointCloud<pcl::PointXYZRGBA>);

	//filter per color
	filterColorBricks(redBricks, 255, 70, 69, 0, 69, 0);
	filterColorBricks(greenBricks, 100, 0, 255, 101, 100, 0);
	filterColorBricks(blueBricks, 79, 0, 100, 0, 255, 101);
	filterColorBricks(yellowBricks, 255, 101, 255, 101, 100, 0);



	//if (!viewer->updatePointCloud(output, "normalized")) {
		//viewer->addPointCloud(output, "normalized");
	//}

	output->clear();
	*output += *redBricks;
	*output += *greenBricks;
	*output += *blueBricks;
	*output += *yellowBricks;
	
	//std::cout << "cloud size: " << output->size() << std::endl;
	*input = *output;

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> segmented_cloud_color_handler(output, 255, 0, 0);
	//if (!viewer->updatePointCloud(output, segmented_cloud_color_handler, "segmented_cloud")) {
		//viewer->addPointCloud(output, segmented_cloud_color_handler, "segmented_cloud");
	//}
}

void Segmenter::filterColorBricks(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered, int rMax, int rMin, int gMax, int gMin, int bMax, int bMin) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr dummyNoAFiltered = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	
	pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::LT, rMax)));
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::GT, rMin)));
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::LT, gMax)));
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::GT, gMin)));
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::LT, bMax)));
	color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::GT, bMin)));

	// build the filter 
	pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem(color_cond);
	condrem.setInputCloud(input_noA);

	// apply filter 
	condrem.filter(*dummyNoAFiltered);

	pcl::copyPointCloud(*dummyNoAFiltered, *filtered);
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