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
	numBricks += filterColorBricks(yellowBricks, 255, 101, 255, 101, 150, 0);



	std::cout << "Num RGBY Final Bricks: " << numBricks << std::endl;
	//if (!viewer->updatePointCloud(output, "normalized")) {
	//viewer->addPointCloud(output, "normalized");
	//}

	output->clear();
	addCloudsToBigCloud(output, redBricks);
	addCloudsToBigCloud(output, greenBricks);
	addCloudsToBigCloud(output, blueBricks);
	addCloudsToBigCloud(output, yellowBricks);
	/**output += *greenBricks;
	*output += *blueBricks;
	*output += *yellowBricks;
	*/
	//std::cout << "cloud size: " << output->size() << std::endl;
	*input = *output;

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
	ec.setClusterTolerance(0.02); // 2cm
	ec.setMinClusterSize(25);
	ec.setMaxClusterSize(700);
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
	std::cout << "Num Bricks: " << j << std::endl;
	return j;

}

void Segmenter::addCloudsToBigCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &cloudclusters) {
	for (std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>::const_iterator it = cloudclusters.begin(); it != cloudclusters.end(); ++it)
	{
		*cloud += **it;
	}
}