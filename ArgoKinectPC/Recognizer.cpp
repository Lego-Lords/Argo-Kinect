#include "stdafx.h" 
#include "Recognizer.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/time.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/vfh.h>
#include <flann/flann.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>
//#include <flann/io/hdf5.h>

#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <math.h>



Recognizer::Recognizer() {
	//connection = sqlCon.setUpConnection("localhost", "root", "", "argo_db");
	cyclesTaken = 0;
	updateduration = 0;
	totalduration = 0;

	

	selectedModel = 0;
	maxSteps = 6;
	currStep = 0;
	hasUpdate = true;
	trackingActive = false;
	useEstimate = false;

	numOfCandidates = 10; //ilan yung kukunin niyang (model with angle)
	acceptedScore = 69;
	numOfIter = 10;
	currIter = 0;
	currAcceptedCandidateIter = 0;
	currAcceptedCandidateIndex = 0;

	//used by Kingston
	acceptanceThreshold = 55;
	numOfIteration = 30;
	currentIteration = 0;
	std::map<std::string, std::pair<float, int> > myMultiValueMap;

	finalAnswer = "unknown";
	distanceThreshold = 5;
	minOccurPercent = 0.75;
	sizeSmallClouds = 500;
	minOccurPercentForSmall = 0.5;

	bestCandidate.first = "unknown";
	bestCandidate.second = 1000;

	secondChoiceKaLang.first = "unknown2";
	secondChoiceKaLang.second = 1001;

	//LOAD FROM CONFIG.TXT
	initValuesFromFile();

	if (useServer) {
		connection = sqlCon.setUpConnection(&db_add[0u], &db_user[0u], &db_pass[0u], &db_name[0u]);
		selectedModel = 0;
	}
		

	initResultsFile = false;
	initAssembly = false;

	input = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
	cloudAgainst = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();


	visual = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
	nextStepModel = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
	currentStepModel = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();

	aligned = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
	modelPointNormal = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
	scenePointNormal = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();

	model_normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
	scene_normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();

	model_keypoints = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
	scene_keypoints = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();

	model_descriptors = boost::make_shared<pcl::PointCloud<pcl::SHOT352>>();
	scene_descriptors = boost::make_shared<pcl::PointCloud<pcl::SHOT352>>();

	model_features = boost::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
	scene_features = boost::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();


	model_rf = boost::make_shared<pcl::PointCloud<pcl::ReferenceFrame>>();
	scene_rf = boost::make_shared<pcl::PointCloud<pcl::ReferenceFrame>>();

	initialTransform = Eigen::Matrix4f::Identity();
	// Visualization
	this->viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("ICP Viewer");
	this->viewer->setCameraPosition(0.0, 0.0, -1.0, 0.0, 0.0, 0.0);
	//this->viewer->addCoordinateSystem(0.1);

	//t
	//getCloudToCompare(nextStepModel);
	//centerCloud(nextStepModel);
	//this->viewer->addPointCloud(nextStepModel, "try");
	//this->viewer->addPointCloud(nextStepModel, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>(nextStepModel, 255.0, 255.0, 255.0), "try");

	this->viewer->addPointCloud(visual, "virtual");
	//this->viewer->spinOnce();
}


Recognizer::~Recognizer() {
	if (useLogs) {
		resultsFile.close();
	}
}

void Recognizer::init()
{
	corrs = boost::make_shared<pcl::Correspondences>();

}


void Recognizer::centerCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {

	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud, centroid);
	std::cout << "Centroid: " << centroid << std::endl;

	Eigen::Affine3f tMatrix = Eigen::Affine3f::Identity();
	tMatrix.translate(centroid.head<3>());
	//pcl::getTransformation(0, 0, 0, 0, 0, 0, tMatrix);
	
	pcl::transformPointCloud(*cloud, *cloud, tMatrix.inverse());
}

vector<string> Recognizer::split(string str, char delimiter) {
	vector<string> internal;
	stringstream ss(str); // Turn the string into a stream.
	string tok;

	while (getline(ss, tok, delimiter)) {
		internal.push_back(tok);
	}

	return internal;
}


void Recognizer::recognizeState(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene) {

	//if no model is selected, wait for selection
	if (selectedModel == 0) {
		//get selection from database
		if (useServer) {
			std::cout << "Waiting for model to be selected......" << std::endl;
			selectedModel = sqlCon.getSelectedModel(connection);
			maxSteps = sqlCon.getMaxStep(connection);
			currStep = sqlCon.getCurrentStep(connection);
		}
		

		//viewer->addPointCloud(scenePointNormal, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(scenePointNormal, 0.0, 255.0, 0.0), "scene");
		
		//viewer->updatePointCloud(scene, "virtual");
		//viewer->spinOnce();
		//viewer->addPointCloud(aligned, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(aligned, 0.0, 0.0, 255.0), "object_aligned");		
	}
	else {
		if (!initAssembly) {
			switch (selectedModel) {
				case 1: selectmodelname = snowcat; break;
				case 2: selectmodelname = pyramid; break;
				case 3: selectmodelname = quacktro; break;
				case 4: selectmodelname = jay; break;
				case 5: selectmodelname = heart; break;
			}
		}
		if (!initResultsFile && useLogs) {
			std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
			std::time_t start_time = std::chrono::system_clock::to_time_t(start);
			
			resultsfilename = "results/" + typeOfTest + "_" + selectmodelname + "_AT:" + std::to_string(acceptanceThreshold) + "_MO:" + std::to_string(minOccurPercent) + "_NI:" + std::to_string(numOfIteration) + "_NC:" + std::to_string(numOfCandidates) + ".csv";
			resultsfilename.erase(std::remove(resultsfilename.begin(), resultsfilename.end(), '\n'), resultsfilename.end());
			std::replace(resultsfilename.begin(), resultsfilename.end(), ':', '_');
			std::replace(resultsfilename.begin(), resultsfilename.end(), ' ', '_');
			resultsFile.open(resultsfilename, std::ios_base::app);
			//add header
			//resultsFile << "Step,Time,Cycles,Best Candidate, Ave. Distance, 2nd Candidate, Ave. Distance, Points" << std::endl;;
			std::cout << resultsfilename << std::endl;
			resultsFile.flush();
			initResultsFile = true;
		}
		if (currStep < maxSteps) {
/*			if (!hasUpdate && useServer) {
				int dbstep = sqlCon.getCurrentStep(connection);
				if (dbstep != currStep) {
					currStep = dbstep;
					hasUpdate = true;
				}
			}*/
			

			if (hasUpdate) {
				if (isStepByStep) {
					string inputthis;
					std::cout << "Input any string to continue. " << std::endl;
					cin >> inputthis;
				}
				
				//std::cout << "Loading needed files for model......" << maxSteps << std::endl;
				aveSizeOfNext = computeAveOfStep(currStep + 1);
				
				//if (nextStepModel->size() > 0)
					//*visual = *nextStepModel;
				//getCloudToCompare(nextStepModel);
				std::cout << "Average size of next: " << aveSizeOfNext << std::endl;
				loadHistogramsFromFiles();
				hasUpdate = false;
				hasError = false;
			}
			else if (scene->size() > 1) {
				int numBricksScene = isolateBricks(input);
				std::cout << "Num Bricks Scene: " << numBricksScene << std::endl;

				//lower thresholds for smaller clouds
				if (lowerThreshForSmall && aveSizeOfNext < sizeSmallClouds) {
					minOccurPercent = minOccurPercentForSmall;
					
					acceptanceThreshold = threshForSmall;
					//numOfIteration = 30;
				}
				else {
					acceptanceThreshold = threshForNormal;
				}
				//else {
					//minOccurPercent = 0.75;
				//}

				*input = *scene;
				//viewer->spinOnce();
				pcl::PointCloud<pcl::VFHSignature308>::Ptr sceneVFH(new pcl::PointCloud<pcl::VFHSignature308>());

				if(showSceneSize)
					std::cout << "Input cloud: " << input->size() << std::endl;
				//std::cout << "Computing scene vfh... " << std::endl;
				computeVFHFeatures(input, sceneVFH);

				vfh_model sceneHist;
				sceneHist.second.resize(308);
				std::vector <pcl::PCLPointField> fields;
				getFieldIndex(*sceneVFH, "vfh", fields);
				for (size_t i = 0; i < fields[0].count; ++i) {
					sceneHist.second[i] = sceneVFH->points[0].histogram[i];
				}
				/*std::vector<float> hist;
				hist.resize(100);
				createColorHistogram(input, hist);
				for (size_t i = 0; i < 100; i++) {
					sceneHist.second[308 + i] = hist[i];
				}*/
				sceneHist.first = "scene_histogram";

				flann::Matrix<int> k_indices;
				flann::Matrix<float> k_distances;
				int k;
				//k = numOfCandidates;
				if (currStep == 0) {
					k = 18;
				}
				else
					k = 36;
				
				flann::Index<flann::ChiSquareDistance<float> > index(data, flann::SavedIndexParams(kdtree_filename));
				index.buildIndex();
				nearestKSearch(index, sceneHist, k, k_indices, k_distances);

				if (showIterVals) {
					pcl::console::print_highlight("The closest %d neighbors for the physical world are:\n", k);
					for (int i = 0; i < k; ++i) {
						pcl::console::print_info("    %d - %s (%d) with a distance of: %f\n",
							i, compModels.at(k_indices[0][i]).first.c_str(), k_indices[0][i], k_distances[0][i]);
					}
				}
				

				//string
				//compModels.at(k_indices[0][i]).first.c_str()

				//distance
				//k_distances[0][i]

				//index
				//k_indices[0][i]


				//myMultiValueMap.clear();

				if (currentIteration == 0)
					start = std::clock();

				if (currentIteration < numOfIteration)
				{
					
					for (int i = 0; i < k; i++) 
					{
						//if (k_distances[0][i] < acceptanceThreshold)
						//{

							/*candidateMatches[candidateMatchesIndex] = k_indices[0][i];
							candidateMatchesIndex++;*/
						if (currStep < 7) {
							if (numBricksScene == actualValues[compModels.at(k_indices[0][i]).first.c_str()][0])
								k_distances[0][i] -= 5;
							else
								k_distances[0][i] += 5;

							if (numBricksScene >= actualValues[compModels.at(k_indices[0][i]).first.c_str()][3])
								k_distances[0][i] -= 15;
							else
								k_distances[0][i] += 10;

							float cloudsize = actualValues[compModels.at(k_indices[0][i]).first.c_str()][2];
							if (cloudsize > scene->size() - cloudsize*cloudSizePercent && cloudsize < scene->size() + cloudsize*cloudSizePercent)
								k_distances[0][i] -= 10;
							else
								k_distances[0][i] += 10;
					}
						else {
							if(numBricksScene == actualValues[compModels.at(k_indices[0][i]).first.c_str()][0])
								k_distances[0][i] -= 10;
							else
								k_distances[0][i] += 5;

							float cloudsize = actualValues[compModels.at(k_indices[0][i]).first.c_str()][2];
							if (cloudsize > scene->size() - cloudsize*cloudSizePercent && cloudsize < scene->size() + cloudsize*cloudSizePercent)
								k_distances[0][i] -= 15;
							else
								k_distances[0][i] += 10;
						}
							

							if (myMultiValueMap.find(compModels.at(k_indices[0][i]).first.c_str()) == myMultiValueMap.end())
							{
								// not found
								//INSERT NEW KEY
								myMultiValueMap[compModels.at(k_indices[0][i]).first.c_str()] = make_pair(k_distances[0][i], 1);
							}
							else {
								// found
								//UPDATE NEW KEY
								myMultiValueMap[compModels.at(k_indices[0][i]).first.c_str()].first += k_distances[0][i];
								myMultiValueMap[compModels.at(k_indices[0][i]).first.c_str()].second++;
							}
						
						//}
					}
					if (currentIteration < numOfIteration)
						currentIteration++;

					vector<string> sepp;
					int angle;
					string fullline = compModels.at(k_indices[0][0]).first.c_str();
					sepp = split(fullline, '_');
					angle = stoi(sepp[3]);

					/*if (useServer) {
						sqlCon.setRotValue(connection, angle);
					}*/

					if(currentIteration >= numOfIteration)
					{

						for (std::map<std::string, std::pair<float, int>>::iterator iter = myMultiValueMap.begin(); iter != myMultiValueMap.end(); ++iter)
						{
							//std::cout << "KEY " + iter->first << std::endl;
							//std::cout << myMultiValueMap[iter->first].first << std::endl;
							//Must be at least 50%
							//std::cout << "curr " + currentIteration << std::endl;
							//if (myMultiValueMap[iter->first].second >= minOccurPercent*numOfIteration)
							//{
								//std::cout << "\hallogoodoccur" << std::endl;
							
							float currentAverage = myMultiValueMap[iter->first].first / myMultiValueMap[iter->first].second;
							std::cout << "Step: " << iter->first << " Ave: " << currentAverage << std::endl;
							if (currentAverage < bestCandidate.second)
							{
								secondChoiceKaLang.second = bestCandidate.second;
								secondChoiceKaLang.first = bestCandidate.first;

								bestCandidate.second = currentAverage;
								bestCandidate.first = iter->first;
							}
							//}
						}

						//did not find second best
						if (secondChoiceKaLang.first == "unknown") {
							//look for second best
							for (std::map<std::string, std::pair<float, int>>::iterator iter = myMultiValueMap.begin(); iter != myMultiValueMap.end(); ++iter)
							{
								//if (myMultiValueMap[iter->first].second >= minOccurPercent*numOfIteration)
								//{
								float currentAverage = myMultiValueMap[iter->first].first / myMultiValueMap[iter->first].second;
								if (currentAverage < secondChoiceKaLang.second && currentAverage != bestCandidate.second)
								{
									secondChoiceKaLang.second = currentAverage;
									secondChoiceKaLang.first = iter->first;
								}

								//}
							}
						}
						cyclesTaken++;
						duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
						updateduration += duration;

						
						std::cout << "Cycle: " << cyclesTaken << std::endl;
						std::cout << "Duration of Cycle: " << duration << std::endl;
						std::cout << "\nBest: " + bestCandidate.first << std::endl;
						printf(" computed: %.4f \n", bestCandidate.second);

						std::cout << "\n2nd choice lang: " + secondChoiceKaLang.first << std::endl;
						printf(" computed: %.4f \n\n", secondChoiceKaLang.second);

						if (useLogs) {
							if (bestCandidate.first == "unknown") {
								savedBestCandidate.first = "-";
								savedBestCandidate.second = "-";
							}
							else {
								savedBestCandidate.first = bestCandidate.first;
								savedBestCandidate.second = std::to_string(round4f(bestCandidate.second));
							}
							
							if (secondChoiceKaLang.first == "unknown" || secondChoiceKaLang.first == "unknown2") {
								saved2ndCandidate.first = "-";
								saved2ndCandidate.second = "-";
							}
							else {
								saved2ndCandidate.first = secondChoiceKaLang.first;
								saved2ndCandidate.second = std::to_string(round4f(secondChoiceKaLang.second));
							}
							
						}
						
						//string delimiter = "_";

						//string best_step = bestCandidate.first.substr(2, bestCandidate.first.find(delimiter));
						//string second_best_step = secondChoiceKaLang.first.substr(2, secondChoiceKaLang.first.find(delimiter));
						vector<string> sep1;
						string best_step;
						vector<string> sep2;
						string second_best_step;

						if (bestCandidate.first != "unknown") {
							sep1 = split(bestCandidate.first, '_');
							best_step = sep1[2];
						}

						if (secondChoiceKaLang.first != "unknown" && secondChoiceKaLang.first != "unknown2") {
							sep2 = split(secondChoiceKaLang.first, '_');
							second_best_step = sep2[2];
						}
						if (bestCandidate.second > 100) {
							hasError = true;
							finalAnswer = "unknown";
						}
						//case 1: if unkown best hasError = true
						else if (bestCandidate.first == "unknown" && secondChoiceKaLang.first == "unknown2")
						{
							hasError = true;
							finalAnswer = "unknown";
						}
						//case 2: if unkown second take best
						else if (secondChoiceKaLang.first == "unknown2")
						{
							hasError = false;
							finalAnswer = bestCandidate.first;

						}
						//case 3: if best and second is same take best
						else if (best_step == second_best_step)
						{
							hasError = false;
							finalAnswer = bestCandidate.first;
						}
						//case 4: if best and second is diff and distance < thresh hasError
						else if(best_step != second_best_step && secondChoiceKaLang.second - bestCandidate.second < distanceThreshold)
						{
							hasError = true;
							finalAnswer = "unknown";
						}
						//case 5: if best and second is diff and distance > thresh take best
						else if (best_step != second_best_step && secondChoiceKaLang.second - bestCandidate.second >= distanceThreshold)
						{
							hasError = false;
							finalAnswer = bestCandidate.first;
						}
						
						if (!hasError) {
							pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
							//helper.readPCD("templates/" + selectmodelname + "/" + finalAnswer , cloud);

							vector<string> sep3 = split(finalAnswer, '_');
							std::cout << "Recognized Step: " + sep3[2] << std::endl;
							int detectedStep = stoi(sep3[2]);
							int angle = stoi(sep3[3]);

							if (useServer) {
								sqlCon.setRotValue(connection, angle);
							}

							/*if (!trackingActive) {
								Eigen::Affine3f tMatrix = Eigen::Affine3f::Identity();
								tMatrix.rotate(Eigen::AngleAxisf(angle * M_PI / 180.0, Eigen::Vector3f::UnitY()));
								initialTransform = tMatrix.matrix();
								trackingActive = true;
							}*/
							
							//rotate around y axis by that angle


							if (detectedStep == currStep + 1 && bestCandidate.second < acceptanceThreshold) {
								//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
								//helper.readPCD("templates/" + selectmodelname + "/" + finalAnswer, cloud);
								
								//int numBricksTemplate = isolateBricks(cloud);
								
								//std::cout << "Num Bricks Template: " << numBricksTemplate << std::endl;
								//if (cloud->size() > scene->size() - cloud->size()*cloudSizePercent && cloud->size() < scene->size() + cloud->size()*cloudSizePercent)
								//if (numBricksScene == numBricksTemplate)
								moveToNextStep = true;
								//else {
									//moveToNextStep = false;
									//std::cout << "Rejected!!!Cloud Wanted: " << numBricksTemplate << " but your scene has: " << numBricksScene  << std::endl;
								//}
									
							}		
							else
								moveToNextStep = false;
						}
					
						currentIteration = 0;
						myMultiValueMap.clear();
						bestCandidate.first = "unknown";
						bestCandidate.second = 1000;

						secondChoiceKaLang.first = "unknown2";
						secondChoiceKaLang.second = 1001;
					}
				}

				//if (trackingActive)
					//performICP();
				//visualize top cloud
				/*std::string viewmodel;
				switch (selectedModel) {
					case 1: viewmodel = snowcat; break;
					case 2: viewmodel = pyramid; break;

					case 3: viewmodel = quacktro; break;
					case 4: viewmodel = jay; break;
					case 5: viewmodel = heart; break;
				}

				helper.readPCD("templates/" + viewmodel + "/" + bestCandidate.first, visual);
				viewer->updatePointCloud(visual, "virtual");*/
			}

			if (hasError && useServer) {
				//std::cout << "ERROR OH SHI" << std::endl;
				sqlCon.updateHasError(connection, 1);
			}
			else if (useServer) {
				//std::cout << "NO ERROR" << std::endl;
				sqlCon.updateHasError(connection, 0);
			}

			//accept step 
			if (moveToNextStep) {
				currStep++;
				pcl::console::print_highlight("Cycles Taken to Update: %d \n", cyclesTaken);
				std::cout << "Number of points: " << scene->size() << std::endl;
				pcl::console::print_warn("Time taken: %.4f \n", updateduration);

				if (useLogs) {
					resultsFile << currStep << "," << round4d(updateduration) << "," << cyclesTaken << "," << savedBestCandidate.first << "," << savedBestCandidate.second << "," << saved2ndCandidate.first << "," << saved2ndCandidate.second << "," << scene->size() << std::endl;;
					resultsFile.flush();
					//stepnum
					//timetorecog
					//best angle
					//best distance
					//second angle
					//second best distance
					//num points
					//num cycles
				}

				updateduration = 0;
				cyclesTaken = 0;
				
				//std::cout << "UPDATE IS HERE" << std::endl;
				if (useServer)
					sqlCon.updateNextStep(connection, currStep);
				moveToNextStep = false;
				hasUpdate = true;
			}

		}
		else {
			//assembly is finished
			pcl::console::print_value("Assembly has completed! Resetting values....");
			if (useLogs) {
				resultsFile.close();
				initResultsFile = false;
			}
			initAssembly = false;
			Sleep(3000);
			currStepModels.clear();
			nextStepModels.clear();
			currStep = 0;
			selectedModel = 0;
			if (useServer) {
				sqlCon.updateNextStep(connection, 0);
				sqlCon.updateModelSelected(connection, 0);
				sqlCon.setRotValue(connection, 0);
			}
		}
		
	}
}

void Recognizer::computeVFHFeatures(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::VFHSignature308>::Ptr outputHist) {

	//compute normals of cloud
	pcl::PointCloud<pcl::Normal>::Ptr normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
	computeNormals(inputCloud, normals, 10);


	pcl::VFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud(inputCloud);
	vfh.setInputNormals(normals);
	// alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the FPFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
	vfh.setSearchMethod(tree);

	// Compute the features
	vfh.compute(*outputHist);
}


void Recognizer::computeNormals(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::Normal>::Ptr normals, float val) {
	pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> norm_est;
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(val);
	norm_est.setInputCloud(inputCloud);
	norm_est.compute(*normals);
}


void Recognizer::loadHistogramsFromFiles() {
	std::string model_dir = base_dir;
	std::string stepfile;
	switch (selectedModel) {
		case 1: model_dir += snowcat; break;
		case 2: model_dir += pyramid; break;
		case 3: model_dir += quacktro; break;
		case 4: model_dir += jay; break;
		case 5: model_dir += heart; break;
	}

	//load VFHs of curr step
	//if (nextStepModels.size() != 0) {
		//std::cout << "no need load new" << std::endl;
		//currStepModels.swap(nextStepModels);
	//}
		
	if (currStep != 0) {
		currStepModels.clear();
		stepfile = "step_" + std::to_string(currStep) + "_";
		loadVFHs(model_dir, stepfile, currStepModels);
	}

	//loadVFHs of next step
	nextStepModels.clear();
	stepfile = "step_" + std::to_string(currStep + 1) + "_";
	loadVFHs(model_dir, stepfile, nextStepModels);

	//combine models of next step and current step
	compModels.clear();
	compModels.insert(compModels.end(), currStepModels.begin(), currStepModels.end());
	compModels.insert(compModels.end(), nextStepModels.begin(), nextStepModels.end());

	actualValues.clear();
	for (int i = 0; i < compModels.size(); i++) {
		std::vector<float> intvect;
		intvect.resize(4);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
		helper.readPCD("templates/" + selectmodelname + "/" + compModels[i].first, cloud);
		vector<string> sepp;
		sepp = split(compModels[i].first, '_');
		intvect[0] = stof(sepp[2]);
		intvect[1] = stof(sepp[3]);
		intvect[2] = cloud->size();
		intvect[3] = isolateBricks(cloud);
		actualValues[compModels[i].first] = intvect;
	}

	flann::Matrix<float> trainingdata(new float[compModels.size() * compModels[0].second.size()], compModels.size(), compModels[0].second.size());

	for (size_t i = 0; i < trainingdata.rows; ++i)
		for (size_t j = 0; j < trainingdata.cols; ++j)
			trainingdata[i][j] = compModels[i].second[j];

	data = trainingdata;

	//flann::save_to_file(data, models_hd5_filename, "training_data");
	/*std::ofstream fs;
	fs.open(models_list_filename.c_str());
	for (size_t i = 0; i < compModels.size(); ++i)
		fs << compModels[i].first << "\n";
	fs.close();*/

	flann::Index<flann::ChiSquareDistance<float> > index(data, flann::LinearIndexParams());
	//flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
	index.buildIndex();
	index.save(kdtree_filename);
	
	//delete[] data.ptr();
}

inline void Recognizer::nearestKSearch(flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model,
	int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances) {
	// Query point
	flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size()], 1, model.second.size());
	memcpy(&p.ptr()[0], &model.second[0], p.cols * p.rows * sizeof(float));

	indices = flann::Matrix<int>(new int[k], 1, k);
	distances = flann::Matrix<float>(new float[k], 1, k);
	index.knnSearch(p, indices, distances, k, flann::SearchParams(512));
	delete[] p.ptr();
}

float Recognizer::computeAveOfStep(int step) {
	std::string model_dir = "templates/";
	std::string stepfile;
	float sum = 0.0;
	int count = 0;
	switch (selectedModel) {
		case 1: model_dir += snowcat; break;
		case 2: model_dir += pyramid; break;
		case 3: model_dir += quacktro; break;
		case 4: model_dir += jay; break;
		case 5: model_dir += heart; break;
	}

	stepfile = "step_" + std::to_string(currStep+1) + "_";
	for (boost::filesystem::directory_iterator it(model_dir); it != boost::filesystem::directory_iterator(); ++it) {
		if (boost::filesystem::is_regular_file(it->status()) && it->path().filename().string().find(stepfile) != std::string::npos) {
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
			helper.readPCD(it->path().string(), cloud);
			sum += cloud->size();
			count++;
		}
	}
	return sum / count;
}

void Recognizer::loadVFHs(const boost::filesystem::path &file_dir, std::string stepfile, std::vector<vfh_model> &models) {
	for (boost::filesystem::directory_iterator it(file_dir); it != boost::filesystem::directory_iterator(); ++it) {
		if (boost::filesystem::is_directory(it->status())) {
			std::stringstream ss;
			ss << it->path();
			//pcl::console::print_highlight("Loading %s (%lu models loaded so far).\n", ss.str().c_str(), (unsigned long)models.size());
			loadVFHs(it->path(), stepfile, models);
		}
		if (boost::filesystem::is_regular_file(it->status()) && it->path().filename().string().find(stepfile) != std::string::npos) {
			//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
			/*ifstream in(it->path().string());
			std::cout << "Loading " << it->path().string() << std::endl;
			string line;
			vfh_model m;
			m.second.resize(408);
			std::string dirfile = it->path().filename().string();
			size_t lastindex = dirfile.find_last_of(".");
			m.first = dirfile.substr(0, lastindex);
			int i = 0;
			while (getline(in, line)) {
				//std::cout << line;
				m.second[i] = stof(line);
				i++;
			}
			in.close();

			models.push_back(m);*/


			pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>());

			helper.readVFHinPCD(it->path().string(), vfhs);
			vfh_model m;
			m.second.resize(308);

			std::vector <pcl::PCLPointField> fields;
			pcl::getFieldIndex(*vfhs, "vfh", fields);

			for (size_t i = 0; i < fields[0].count; ++i) {
				m.second[i] = vfhs->points[0].histogram[i];
			}
			m.first = it->path().filename().string();
			models.push_back(m);
		}
	}
}



/*ALL OF BELOW ARE TRIAL ONLY NOT MAIN CODE HOHOHOHO*/





void Recognizer::getCloudToCompare(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr saved) {
	std::string stepfile = "";
	switch (selectedModel) {
		case 1: stepfile = snowcat; break;
		case 2: stepfile = pyramid; break;
		case 3: stepfile = quacktro; break;
		case 4: stepfile = jay; break;
		case 5: stepfile = heart; break;
	}
	
	helper.readPCD("steps/" + stepfile + "_step_" + std::to_string(currStep + 1) + ".pcd", saved);
	//helper.readPCD("templates/duck/duck_step_1_0.pcd", saved);
	std::cout << "Obtained cloud " << saved->size() << std::endl;
	centerCloud(saved);
}




void Recognizer::computePointNormals(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, float val) {
	pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal> norm_est;
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(val);
	norm_est.setInputCloud(inputCloud);
	norm_est.compute(*inputCloud);
}

void Recognizer::obtainKeypoints(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints, float radius) {
	pcl::UniformSampling<pcl::PointXYZRGBA> sampler;
	sampler.setInputCloud(inputCloud);
	sampler.setRadiusSearch(radius);
	sampler.filter(*keypoints);
}

void Recognizer::downsample(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outputCloud, float leafsize) {
	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*inputCloud, *cloud_blob);
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud_blob);
	sor.setLeafSize(leafsize, leafsize, leafsize);
	sor.filter(*cloud_filtered_blob);
	pcl::fromPCLPointCloud2(*cloud_filtered_blob, *outputCloud);
	for (size_t i = 0; i < outputCloud->size(); i++) {
		outputCloud->points[i].a = 255;
	}
}

void Recognizer::computeDescriptor(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::SHOT352>::Ptr descriptors, float radius) {
	pcl::SHOTEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT352> descEstimator;
	descEstimator.setRadiusSearch(radius);
	descEstimator.setInputCloud(keypoints);
	descEstimator.setInputNormals(normals);
	descEstimator.setSearchSurface(inputCloud);
	descEstimator.compute(*descriptors);
}

void Recognizer::findCorrespondences() {
	
	pcl::KdTreeFLANN<pcl::SHOT352> kdsearch;
	kdsearch.setInputCloud(model_descriptors);

	for (size_t i = 0; i < scene_descriptors->size(); ++i) {
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		if (!pcl_isfinite(scene_descriptors->at(i).descriptor[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = kdsearch.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
		if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
		{
			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			corrs->push_back(corr);
		}
	}
	std::cout << "Correspondences found: " << corrs->size() << std::endl;
}
/**void Recognizer::clusterCorrespondences(float binSize, float thresh) {
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector < pcl::Correspondences > clustered_corrs;
	pcl::GeometricConsistencyGrouping<pcl::PointXYZRGBA, pcl::PointXYZRGBA> gc_clusterer;
	gc_clusterer.setGCSize(binSize);
	gc_clusterer.setGCThreshold(thresh);

	gc_clusterer.setInputCloud(model_keypoints);
	gc_clusterer.setSceneCloud(scene_keypoints);
	gc_clusterer.setModelSceneCorrespondences(corrs);

	//gc_clusterer.cluster (clustered_corrs);
	std::cout << "Recognize clusters " << std::endl;
	gc_clusterer.recognize(rototranslations, clustered_corrs);

	
	//std::cout << "Model instances found: " << rototranslations.size() << std::endl;

}
/**/
void Recognizer::computeReferenceFrames(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud, pcl::PointCloud<pcl::ReferenceFrame>::Ptr rf, float radius) {
	pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::ReferenceFrame> rf_est;
	rf_est.setFindHoles(true);
	rf_est.setRadiusSearch(radius);
	rf_est.setInputCloud(keypoints);
	rf_est.setInputNormals(normals);
	rf_est.setSearchSurface(inputCloud);
	rf_est.compute(*rf);
}

void Recognizer::clusterCorrespondences(float binSize, float thresh) {
	pcl::Hough3DGrouping<pcl::PointXYZRGBA, pcl::PointXYZRGBA, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
	clusterer.setHoughBinSize(binSize);
	clusterer.setHoughThreshold(thresh);
	clusterer.setUseInterpolation(true);
	clusterer.setUseDistanceWeight(false);

	clusterer.setInputCloud(model_keypoints);
	clusterer.setInputRf(model_rf);
	clusterer.setSceneCloud(scene_keypoints);
	clusterer.setSceneRf(scene_rf);
	clusterer.setModelSceneCorrespondences(corrs);
	std::cout << "hallo found: " << corrs->size() << std::endl;
	clusterer.recognize(rototranslations, clustCorrs);

	
	std::cout << "Model instances found: " << rototranslations.size () << std::endl;

}


void Recognizer::estimatePose()
{
	// Estimate features
	std::cout << "Estimating features... " << std::endl;
	pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fest;
	fest.setRadiusSearch(0.025);
	fest.setInputCloud(modelPointNormal);
	fest.setInputNormals(modelPointNormal);
	fest.compute(*model_features);

	fest.setInputCloud(scenePointNormal);
	fest.setInputNormals(scenePointNormal);
	fest.compute(*scene_features);

	// Perform alignment
	std::cout << "Starting alignment... " << std::endl;
	pcl::SampleConsensusInitialAlignment<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> align;
	align.setInputSource(modelPointNormal);
	align.setSourceFeatures(model_features);
	align.setInputTarget(scenePointNormal);
	align.setTargetFeatures(scene_features);
	align.setMinSampleDistance(0.05f);

	align.setMaximumIterations(5000); // Number of RANSAC iterations
	//align.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
	//align.setCorrespondenceRandomness(2); // Number of nearest features to use
	//align.setSimilarityThreshold(0.6f); // Polygonal edge length similarity threshold
	align.setMaxCorrespondenceDistance(2.5f * leafsize); // Inlier threshold
	//align.setInlierFraction(0.25f); // Required inlier fraction for accepting a pose hypothesis
	{
		pcl::ScopeTime t("Alignment");
		align.align(*aligned);
	}

	if (align.hasConverged()) {
		trackingActive = true;
		// Print results
		std::cout << "Alignment converged... " << std::endl;

		//printf("Inliers: %i/%i\n", align.getInliers().size(), cloudAgainst->size());

		initialTransform = align.getFinalTransformation();

		pcl::transformPointCloud(*visual, *visual, initialTransform);
		viewer->updatePointCloud(visual, "virtual");
		// Show alignment
		//viewer->addPointCloud(scenePointNormal, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(scenePointNormal, 0.0, 255.0, 0.0), "scene");
		//viewer->updatePointCloud(aligned, "virtual");
		//viewer->addPointCloud(aligned, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(aligned, 0.0, 0.0, 255.0), "object_aligned");		
	}
	else {
		std::cout << "Alignment failed... " << std::endl;
	}
}

void Recognizer::performICP()
{
	copyPointCloud(*nextStepModel, *modelPointNormal);
	copyPointCloud(*input, *scenePointNormal);

	computePointNormals(modelPointNormal, 10);
	computePointNormals(scenePointNormal, 10);
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
	icp.setMaximumIterations(1000);
	icp.setMaxCorrespondenceDistance(0.05);
	icp.setRANSACOutlierRejectionThreshold(1.5);
	icp.setEuclideanFitnessEpsilon(1);
	icp.setTransformationEpsilon(1e-9);

	icp.setInputSource(modelPointNormal);
	icp.setInputTarget(scenePointNormal);
	
	icp.align(*aligned, initialTransform);
	if (icp.hasConverged()) {
		transformation_matrix = icp.getFinalTransformation();
		transformation_matrix(0, 3) = 0.0f;
		transformation_matrix(1, 3) = 0.0f;
		transformation_matrix(2, 3) = 0.0f;
		initialTransform = transformation_matrix;
		//std::cout << "Outliers: " << icp. << std::endl;
		std::cout << "ICP converged, score: " << icp.getFitnessScore() << std::endl;
		print4x4Matrix(transformation_matrix);
		pcl::transformPointCloud(*visual, *visual, transformation_matrix);
		viewer->updatePointCloud(visual, "virtual");
		
		viewer->spinOnce();
	}

	else {
		std::cout << "ICP failed" << std::endl;
	}
	
}

void Recognizer::convertRGBAtoPointNormal(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input, pcl::PointCloud<pcl::PointNormal>::Ptr output)
{
	output->resize(input->points.size());
	for (size_t i = 0; i < input->points.size(); ++i) {
		output->points[i].x = input->points[i].x;
		output->points[i].y = input->points[i].y;
		output->points[i].z = input->points[i].z;
	}
}


void Recognizer::print4x4Matrix(const Eigen::Matrix4f & matrix) {
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void Recognizer::lowerVisibleArea(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  cloud, std::string axis, float min, float max) {
	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName(axis);
	pass.setFilterLimits(min, max);
	pass.filter(*cloud);
}

void Recognizer::initValuesFromFile() {
	ifstream in("config.txt");
	string line;
	std::map<std::string, std::string> keyvalueMap;
	while (getline(in, line)) {
		//do something with the line
		if (line.at(0) != '#') {
			vector<string> vectorLine = split(line, '=');
			keyvalueMap[vectorLine[0]] = vectorLine[1];
		}
	}

	useServer = stoi(keyvalueMap["useServer"]);

	if (!useServer) {
		selectedModel = stoi(keyvalueMap["selectedModel"]);
		maxSteps = stoi(keyvalueMap["maxSteps"]);
	}

	currStep = stoi(keyvalueMap["currStep"]);
	acceptanceThreshold = stof(keyvalueMap["acceptanceThreshold"]);
	threshForNormal = acceptanceThreshold;
	numOfIteration = stoi(keyvalueMap["numOfIteration"]);
	minOccurPercent = stof(keyvalueMap["minOccurPercent"]);
	minOccurPercentForSmall = stof(keyvalueMap["minOccurPercentForSmall"]);
	lowerThreshForSmall = stoi(keyvalueMap["lowerThreshForSmall"]);
	threshForSmall = stof(keyvalueMap["threshForSmall"]);
	numOfCandidates = stoi(keyvalueMap["numOfCandidates"]);
	cloudSizePercent = stof(keyvalueMap["cloudSizePercent"]);
	distanceThreshold = stof(keyvalueMap["distanceThreshold"]);
	cloudSizeRejection = stof(keyvalueMap["cloudSizeRejection"]);
	sizeSmallClouds = stoi(keyvalueMap["sizeSmallClouds"]);
	isStepByStep = stoi(keyvalueMap["isStepByStep"]);
	useLogs = stoi(keyvalueMap["useLogs"]);
	typeOfTest = keyvalueMap["typeOfTest"];

	db_add = keyvalueMap["db_add"];
	db_user = keyvalueMap["db_user"];
	db_pass = keyvalueMap["db_pass"];
	db_name = keyvalueMap["db_name"];

	showIterVals = stoi(keyvalueMap["showIterVals"]);
	showSceneSize = stoi(keyvalueMap["showSceneSize"]);
	pcl::console::print_highlight("Loaded values from text file!\n");
}

float Recognizer::round4f(float f) {
	return roundf(f * 10000) / 10000;
}

double Recognizer::round4d(double d) {
	return round(d * 10000) / 10000;
}

int Recognizer::isolateBricks(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {

	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>  redBricks;
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>  greenBricks;
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>  blueBricks;
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>  yellowBricks;

	//filter per color
	int numBricks = 0;
	numBricks += filterColorBricks(cloud, redBricks, 255, 70, 69, 0, 69, 0);
	numBricks += filterColorBricks(cloud, greenBricks, 100, 0, 255, 101, 100, 0);
	numBricks += filterColorBricks(cloud, blueBricks, 79, 0, 100, 0, 255, 101);
	numBricks += filterColorBricks(cloud, yellowBricks, 255, 101, 255, 101, 100, 0);

	return numBricks;

	//std::cout << "Num RGBY Final Bricks: " << numBricks << std::endl;
}

int Recognizer::filterColorBricks(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &filtered_bricks, int rMax, int rMin, int gMax, int gMin, int bMax, int bMin) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_noA = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	pcl::copyPointCloud(*cloud, *input_noA);
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
	condrem.filter(*input_noA);

	pcl::copyPointCloud(*input_noA, *singlecloud);
	if (singlecloud->size() > 0) {
		return clusterBricks(singlecloud, filtered_bricks);
	}

	return 0;
}

int Recognizer::clusterBricks(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &cloudclusters) {
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
	ec.setClusterTolerance(0.008); // 2cm
	ec.setMinClusterSize(25);
	ec.setMaxClusterSize(1000);
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
		//std::cout << "Cloud cluster Size: " << cloud_cluster->size() << std::endl;
		j++;
	}
	//std::cout << "Cloud Size: " << cloud->size() << " Num Bricks: " << j << std::endl;
	return j;

}


void Recognizer::createColorHistogram(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector<float> &hist) {
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
