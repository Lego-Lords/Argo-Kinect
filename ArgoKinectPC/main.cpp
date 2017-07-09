#include <vtkAutoInit.h>

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkRenderingFreeType);

#include "stdafx.h"
#include "Kinect.h"

#include <wrl/client.h>
#include "SQLConnect.h"
using namespace Microsoft::WRL;

int main(int argc, char* argv[])
{

	try {
		Kinect kinect;
		kinect.run();
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}
	cin.get();

	return 0;
}


void sqlShit()
{
	SQLConnect sqlConnect;
	MYSQL * conn = sqlConnect.setUpConnection("localhost", "root", "", "argo");
	int x = sqlConnect.getSelectedModel(conn);
	int y = sqlConnect.getCurrentStep(conn);
	sqlConnect.updateNextStep(conn, 11);

	cout << x << endl;
	cout << y << endl;
	cin.get();

}

//int main(int argc, char* argv[])
//{
//	ICPCompare icpCompare;
//	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//
//	PCDReader pcdReader;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pcin = pcdReader.readPCD("monkey.pcd");
//	/*pcl::PointCloud<pcl::PointXYZ>::Ptr pcin(new pcl::PointCloud<pcl::PointXYZ>);
//	//Fill in the CloudIn data
//	pcin->width = 100;
//	pcin->height = 50;
//	pcin->is_dense = false;
//	pcin->points.resize(pcin->width * pcin->height);
//	for (size_t i = 0; i < pcin->points.size(); ++i)
//	{
//	pcin->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
//	pcin->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
//	pcin->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
//	}*/
//	//pcl::PointCloud<pcl::PointXYZ>::Ptr pcin = pcdReader.readPCD("snowcat_step_1.pcd");
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pcout = pcdReader.readPCD("snowcat_step_3.pcd");
//	//DITO PASUKAN NG PCD NA ICOCOMPARE
//	icp = icpCompare.comparePCD(pcin, pcout);
//	int x = icp.hasConverged();
//	//icp.hasConverged();
//	//cout << x << icp.getMaximumIterations() << endl;
//	cout << x << x << endl;
//	//VIEW THE SHIT
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//	// Create PCLVisualizer
//	viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("Point Cloud Viewer");
//
//	// Initialize camera position
//	viewer->setCameraPosition(0.0, 0.0, -100, 0.0, 0.0, 0.0);
//
//	// Add Coordinate System
//	viewer->addCoordinateSystem(0.1);
//
//	if (!viewer->updatePointCloud(pcin, "cloud")) {
//		viewer->addPointCloud(pcin, "cloud");
//	}
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;
//	// Create PCLVisualizer
//	viewer2 = boost::make_shared<pcl::visualization::PCLVisualizer>("Point Cloud Viewer");
//
//	// Initialize camera position
//	viewer2->setCameraPosition(0.0, 0.0, -100, 0.0, 0.0, 0.0);
//
//	// Add Coordinate System
//	viewer2->addCoordinateSystem(0.1);
//	if (!viewer2->updatePointCloud(pcout, "cloud")) {
//		viewer2->addPointCloud(pcout, "cloud");
//	}
//
//	// Update Viwer
//	while (!viewer2->wasStopped())
//	{
//		viewer2->spinOnce();
//	}
//
//	// Update Viwer
//	while (!viewer->wasStopped())
//	{
//		viewer->spinOnce();
//	}
//	//END VIEW THE SHIT
//
//	/*try {
//	Kinect kinect;
//	kinect.run();
//	}
//	catch (std::exception& ex) {
//	std::cout << ex.what() << std::endl;
//	}*/
//	cin.get();
//	return 0;
//}
