void Kinect::segmentUpdate() {


	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  smolCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  convexHull(new pcl::PointCloud<pcl::PointXYZRGBA>);
	/*pcl::toPCLPointCloud2(*cloud, *cloud_blob);
	// Create the filtering object
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud_blob);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*cloud_filtered_blob);
	pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);*/


	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.3, 0.9);
	pass.filter(*smolCloud);

	/*pass.setInputCloud(smolCloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(0.3, -0.3);
	pass.filter(*cloud_filtered);*/

	cloud_filtered.swap(smolCloud);

	//Remove the plane (floor/table) ///////////////////
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.005);

	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	int i = 0, nr_points = (int)cloud_filtered->points.size();

	//while (cloud_filtered->points.size() > 0.3 * nr_points) {
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0) {
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			//break;
		}

		// Extract yung mga di kasama sa plane
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_extracted = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*smolCloud);
		//std::cout << "May natanggal na plane gg\n" << std::endl;
		cloud_filtered.swap(smolCloud);
		i++;
	//}
	//std::cout << i << std::endl;

	//checkSave();
	/*
	int i = 0, nr_points = (int)cloud_filtered->points.size();


	seg.setInputCloud(cloud_filtered);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() == 0)
	{
		std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	}
	else
	{
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		//extract.setNegative(false);
		extract.filter(*smolCloud);

		// Retrieve the convex hull.
		pcl::ConvexHull<pcl::PointXYZRGBA> hull;
		hull.setInputCloud(smolCloud);
		// Make sure that the resulting hull is bidimensional.
		hull.setDimension(2);
		hull.reconstruct(*convexHull);

		if (hull.getDimension() == 2)
		{
			// Prism object.
			pcl::ExtractPolygonalPrismData<pcl::PointXYZRGBA> prism;
			prism.setInputCloud(cloud_filtered);
			prism.setInputPlanarHull(convexHull);
			// First parameter: minimum Z value. Set to 0, segments objects lying on the plane (can be negative).
			// Second parameter: maximum Z value, set to 10cm. Tune it according to the height of the objects you expect.
			prism.setHeightLimits(0.0f, 0.1f);
			pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

			prism.segment(*objectIndices);

			// Get and show all points retrieved by the hull.
			extract.setIndices(objectIndices);
			extract.filter(*cloud_filtered);
			//std::cout << "May natanggal na plane gg\n" << std::endl;
		}
	}

	// Extract yung mga di kasama sa plane
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_extracted = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

	pOutput.swap(cloud_filtered);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colorClouds = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
	for (size_t i = 0; i < cloud_filtered->points.size(); i++)
	{
		if ((int(cloud_filtered->points[i].r) > 2 * int(cloud_filtered->points[i].g)) && (cloud_filtered->points[i].r > cloud_filtered->points[i].b))
			colorClouds->points.push_back(cloud_filtered->points[i]);
		if ((cloud_filtered->points[i].g > cloud_filtered->points[i].r) && (cloud_filtered->points[i].g > cloud_filtered->points[i].b))
			colorClouds->points.push_back(cloud_filtered->points[i]);
		if ((cloud_filtered->points[i].b > cloud_filtered->points[i].r) && (cloud_filtered->points[i].b > cloud_filtered->points[i].g))
			colorClouds->points.push_back(cloud_filtered->points[i]);
		if ((cloud_filtered->points[i].r > cloud_filtered->points[i].g) && (int(cloud_filtered->points[i].g) - int(cloud_filtered->points[i].b) > 30))
			colorClouds->points.push_back(cloud_filtered->points[i]);
	}
	pOutput.swap(colorClouds);*/
	// Creating the KdTree object for the search method of the extraction
	if (datagathering) {
		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
		tree->setInputCloud(cloud_filtered);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
		ec.setClusterTolerance(0.02); // 2cm
		ec.setMinClusterSize(100);
		ec.setMaxClusterSize(25000);
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud_filtered);
		ec.extract(cluster_indices);

		int j = 0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
				cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*
			cloud_cluster->width = cloud_cluster->points.size();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;
			pcl::PCDWriter writer;
			std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
			std::stringstream ss;
			ss << "cloud_cluster_" << j << ".pcd";
			writer.write<pcl::PointXYZRGBA>(ss.str(), *cloud_cluster, false);
			j++;
		}
	}

	pOutput.swap(cloud_filtered);
	pOutput.swap(cloud);
	sceneFound = 0;
	sceneFound = pOutput->size();
}