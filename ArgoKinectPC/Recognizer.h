#pragma once
class Recognizer {
private:
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output;

public:
	Recognizer();
	~Recognizer();
};

