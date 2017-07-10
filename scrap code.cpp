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

void Kinect::match() {
	std::string snowcat = "snowcat_step_";
	std::string pyramid = "pyramid_step_";


	if (noSelect) {
		std::cout << "Check db" << std::endl;
		selectedModel = con.getSelectedModel(connection);
		//selectedModel = 2;
		//std::cout << selectedModel << std::endl;
		//cin.get();
		if (selectedModel != 0) {
			noSelect = false;
			currStep = con.getCurrentStep(connection);
			//currStep = 9;
			maxSteps = 13;
			switch (selectedModel) {
			case 1: stepfile = snowcat; break;
			case 2: stepfile = pyramid; break;
			}
		}
	}

	else {
		if (updated) {
			//currStep++;
			cloudAgainst = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
			pread.readPCD(stepfile + std::to_string(currStep + 1) + ".pcd", cloudAgainst);
			//pread.readPCD("cloud_cluster_1.pcd", cloudAgainst);
			updated = false;

		}
		if (sceneFound > 0) {
			std::cout << stepfile + std::to_string(currStep + 1) + ".pcd" << endl;
			//pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
			pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>());
			//pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
			pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());
			pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
			pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());
			pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType>());
			pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(new pcl::PointCloud<DescriptorType>());

			float model_ss_(0.04f);
			float scene_ss_(0.04f);
			float rf_rad_(0.2f);
			float descr_rad_(5.0f);
			float cg_size_(0.4f);
			//float cg_thresh_(-0.5f);
			float cg_thresh_(-30.0f);




			//check matching
			/**
			* Compute Normals
			*/
			pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
			norm_est.setKSearch(10);
			norm_est.setInputCloud(cloudAgainst);
			norm_est.compute(*model_normals);

			norm_est.setInputCloud(pOutput);
			norm_est.compute(*scene_normals);

			/**
			*  Downsample Clouds to Extract keypoints
			*/
			pcl::UniformSampling<PointType> uniform_sampling;
			uniform_sampling.setInputCloud(cloudAgainst);
			uniform_sampling.setRadiusSearch(model_ss_);
			uniform_sampling.filter(*model_keypoints);
			std::cout << "Model total points: " << cloudAgainst->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;

			uniform_sampling.setInputCloud(pOutput);
			uniform_sampling.setRadiusSearch(scene_ss_);
			uniform_sampling.filter(*scene_keypoints);
			std::cout << "Scene total points: " << pOutput->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;

			/**
			*  Compute Descriptor for keypoints
			*/
			pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
			descr_est.setRadiusSearch(descr_rad_);

			descr_est.setInputCloud(model_keypoints);
			descr_est.setInputNormals(model_normals);
			descr_est.setSearchSurface(cloudAgainst);
			descr_est.compute(*model_descriptors);

			descr_est.setInputCloud(scene_keypoints);
			descr_est.setInputNormals(scene_normals);
			descr_est.setSearchSurface(pOutput);
			descr_est.compute(*scene_descriptors);

			/**
			*  Find Model-Scene Correspondences with KdTree
			*/
			pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());
			pcl::KdTreeFLANN<DescriptorType> match_search;
			match_search.setInputCloud(model_descriptors);
			std::vector<int> model_good_keypoints_indices;
			std::vector<int> scene_good_keypoints_indices;

			for (size_t i = 0; i < scene_descriptors->size(); ++i) {
				std::vector<int> neigh_indices(1);
				std::vector<float> neigh_sqr_dists(1);
				if (!pcl_isfinite(scene_descriptors->at(i).descriptor[0]))  //skipping NaNs
				{
					continue;
				}
				int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
				if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) {
					pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
					model_scene_corrs->push_back(corr);
					model_good_keypoints_indices.push_back(corr.index_query);
					scene_good_keypoints_indices.push_back(corr.index_match);
				}
			}
			pcl::PointCloud<PointType>::Ptr model_good_kp(new pcl::PointCloud<PointType>());
			pcl::PointCloud<PointType>::Ptr scene_good_kp(new pcl::PointCloud<PointType>());
			pcl::copyPointCloud(*model_keypoints, model_good_keypoints_indices, *model_good_kp);
			pcl::copyPointCloud(*scene_keypoints, scene_good_keypoints_indices, *scene_good_kp);

			std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;

			/**
			*  Clustering
			*/

			std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
			std::vector < pcl::Correspondences > clustered_corrs;

			if (true) {
				pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>());
				pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>());

				pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
				rf_est.setFindHoles(true);
				rf_est.setRadiusSearch(rf_rad_);

				rf_est.setInputCloud(model_keypoints);
				rf_est.setInputNormals(model_normals);
				rf_est.setSearchSurface(cloudAgainst);
				rf_est.compute(*model_rf);

				rf_est.setInputCloud(scene_keypoints);
				rf_est.setInputNormals(scene_normals);
				rf_est.setSearchSurface(pOutput);
				rf_est.compute(*scene_rf);

				//  Clustering
				pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
				clusterer.setHoughBinSize(cg_size_);
				clusterer.setHoughThreshold(cg_thresh_);
				clusterer.setUseInterpolation(true);
				clusterer.setUseDistanceWeight(false);

				clusterer.setInputCloud(model_keypoints);
				clusterer.setInputRf(model_rf);
				clusterer.setSceneCloud(scene_keypoints);
				clusterer.setSceneRf(scene_rf);
				clusterer.setModelSceneCorrespondences(model_scene_corrs);

				clusterer.recognize(rototranslations, clustered_corrs);
			}
			else {
				pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
				gc_clusterer.setGCSize(cg_size_);
				gc_clusterer.setGCThreshold(cg_thresh_);

				gc_clusterer.setInputCloud(model_keypoints);
				gc_clusterer.setSceneCloud(scene_keypoints);
				gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

				gc_clusterer.recognize(rototranslations, clustered_corrs);
			}

			if (rototranslations.size() <= 0) {
				cout << "*** No instances found! ***" << endl;
			}
			else {
				cout << "Recognized Instances: " << rototranslations.size() << endl << endl;
				currStep++;
				con.updateNextStep(connection, currStep);
				updated = true;
			}

			/*
			//Generates clouds for each instances found

			std::vector<pcl::PointCloud<PointType>::ConstPtr> instances;

			for (size_t i = 0; i < rototranslations.size(); ++i)
			{
			pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
			pcl::transformPointCloud(*cloudAgainst, *rotated_model, rototranslations[i]);
			instances.push_back(rotated_model);
			}


			//ICP

			float icp_corr_distance_(0.005f);
			float hv_clutter_reg_(5.0f);
			float hv_inlier_th_(-0.05f);
			float hv_occlusion_th_(0.01f);
			float hv_rad_clutter_(0.03f);
			float hv_regularizer_(3.0f);
			float hv_rad_normals_(0.05);
			bool hv_detect_clutter_(true);
			std::vector<pcl::PointCloud<PointType>::ConstPtr> registered_instances;
			if (true)
			{
			cout << "--- ICP ---------" << endl;

			for (size_t i = 0; i < rototranslations.size(); ++i)
			{
			pcl::IterativeClosestPoint<PointType, PointType> icp;
			icp.setMaximumIterations(5);
			icp.setMaxCorrespondenceDistance(icp_corr_distance_);
			icp.setInputTarget(pOutput);
			icp.setInputSource(instances[i]);
			pcl::PointCloud<PointType>::Ptr registered(new pcl::PointCloud<PointType>);
			icp.align(*registered);
			registered_instances.push_back(registered);
			cout << "Instance " << i << " ";
			if (icp.hasConverged())
			{
			cout << "Aligned!" << endl;
			}
			else
			{
			cout << "Not Aligned!" << endl;
			}
			}

			cout << "-----------------" << endl << endl;
			}


			// Hypothesis Verification

			cout << "--- Hypotheses Verification ---" << endl;
			std::vector<bool> hypotheses_mask;  // Mask Vector to identify positive hypotheses

			pcl::GlobalHypothesesVerification<PointType, PointType> GoHv;

			GoHv.setSceneCloud(pOutput);  // Scene Cloud
			GoHv.addModels(registered_instances, true);  //Models to verify

			GoHv.setInlierThreshold(hv_inlier_th_);
			GoHv.setOcclusionThreshold(hv_occlusion_th_);
			GoHv.setRegularizer(hv_regularizer_);
			GoHv.setRadiusClutter(hv_rad_clutter_);
			GoHv.setClutterRegularizer(hv_clutter_reg_);
			GoHv.setDetectClutter(hv_detect_clutter_);
			GoHv.setRadiusNormals(hv_rad_normals_);

			GoHv.verify();
			GoHv.getMask(hypotheses_mask);  // i-element TRUE if hvModels[i] verifies hypotheses

			for (int i = 0; i < hypotheses_mask.size(); i++)
			{
			if (hypotheses_mask[i])
			{
			cout << "Instance " << i << " is GOOD! <---" << endl;

			}
			else
			{
			cout << "Instance " << i << " is bad!" << endl;
			}
			}
			cout << "-------------------------------" << endl;*/

			/**
			*  Visualization
			*/
			//pcl::visualization::PCLVisualizer viewer("Hypotheses Verification");
			//viewer.addPointCloud(pOutput, "scene_cloud");
			/*
			pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
			pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());

			pcl::PointCloud<PointType>::Ptr off_model_good_kp(new pcl::PointCloud<PointType>());
			pcl::transformPointCloud(*cloudAgainst, *off_scene_model, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
			pcl::transformPointCloud(*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
			pcl::transformPointCloud(*model_good_kp, *off_model_good_kp, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));*/
		}
	}



}