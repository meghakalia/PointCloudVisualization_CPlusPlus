#pragma once 
#include <vector>
#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkSurfaceReconstructionFilter.h>
#include <vtkProgrammableSource.h>
#include <vtkContourFilter.h>
#include <vtkReverseSense.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkPolyData.h>
#include <vtkCamera.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSphereSource.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkVertexGlyphFilter.h>

//convert to PCL 
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/visualization/pcl_visualizer.h>

//Triangulation 
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/organized_fast_mesh.h>

//Headers for p[oint cloud from openCV 
//#include <glm.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/calib3d/calib3d.hpp"

//Filetering 
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>

//Statistical npise removal 
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/filters/shadowpoints.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/feature.h>
#include <pcl/common/impl/io.hpp>

#include <pcl/filters/radius_outlier_removal.h>

using namespace pcl;
using namespace pcl::io;

int main(int argc, char *argv[])
{

	//check behaviour if vector 
	std::vector<int> PointsColor;
	for (int i = 0; i < 10; i++)
	{
		PointsColor.push_back(i); 
	}

	for (int i = 0; i < 10; i++)
	{
		std::cout <<  PointsColor[i] << std::endl;
	}

	std::cout << " end" << PointsColor.back() << std::endl; 
	///Render point cloud from SBGM algorithm 
	cv::Mat frame; 
	cv::VideoCapture icap("C:\\stereo.avi");

	if (!icap.isOpened())
	{
		return -1; 
	}

	icap >> frame;

	//separate Images into right and Left
	cv::Mat Left;
	Left.create(frame.rows, frame.cols / 2, frame.type());
	frame(cv::Rect(0, 0, (frame.cols / 2), frame.rows)).copyTo(Left);

	cv::Mat Right;
	Right.create(frame.rows, frame.cols / 2, frame.type());
	frame(cv::Rect((frame.cols / 2), 0, (frame.cols / 2), frame.rows)).copyTo(Right);

	// Display functions
	// cv::imshow("Left Frame", Left);
	// cv::namedWindow("Left Frame", CV_WINDOW_AUTOSIZE);

	// cv::imshow("Right Frame", Right);
	// cv::namedWindow("Right Frame", CV_WINDOW_AUTOSIZE);

	// cv::imshow(" Frame", frame);
	// cv::namedWindow(" Frame", CV_WINDOW_NORMAL);
	
	//stereoRectify
	cv::Mat left_cam_matrix(3, 3, CV_64F);
	cv::Mat right_cam_matrix(3, 3, CV_64F);
	cv::Mat left_dist_coeffs(1, 5, CV_64F);
	cv::Mat right_dist_coeffs(1, 5, CV_64F);
	cv::Mat InputRot(3, 3, CV_64F);
	cv::Mat InputTrans(3, 1, CV_64F);
	cv::Mat R1(3, 3, CV_64F);
	cv::Mat R2(3, 3, CV_64F);
	cv::Mat P1(3, 3, CV_64F);
	cv::Mat P2(3, 3, CV_64F);
	cv::Mat Q_openCV;// (4, 4, CV_64F);

	// Load your camera matrices 
	left_cam_matrix = (cv::Mat_<double>(3, 3) << 530.90002,   0, 136.63037, 0, 581.00362, 161.32884, 0, 0, 1);
	right_cam_matrix = (cv::Mat_<double>(3, 3) << 524.84413,   0, 216.17358 ,0, 577.11024,  149.76379, 0, 0, 1);
	left_dist_coeffs = (cv::Mat_<double>(1, 5) << -0.28650,   0.29524, - 0.00212,   0.00152, 0);
	right_dist_coeffs = (cv::Mat_<double>(1, 5) << -0.25745,   0.62307,   0.03660, - 0.01082, 0);
	InputRot = (cv::Mat_<double>(3, 3) << 0.9990, - 0.0112, - 0.0426, 0.0117, 0.9999, 0.0097, 0.0425, - 0.0102, 0.9990);
	InputTrans = (cv::Mat_<double>(3, 1) << -5.49238,   0.04267, - 0.39886);
	cv::stereoRectify(left_cam_matrix, left_dist_coeffs, right_cam_matrix, right_dist_coeffs, Left.size(), InputRot, InputTrans, R1, R2, P1, P2, Q_openCV, 1, -1, Left.size());// , CALIB_ZERO_DISPARITY, 0, frame_size, 0, 0);
	//Depth Map calculation 

	// std::cout << Q_openCV.at<double>(0, 0) << " " << Q_openCV.at<double>(0, 1) << " " << Q_openCV.at<double>(0, 2) << Q_openCV.at<double>(0, 3) << std::endl
	// 	<< Q_openCV.at<double>(1, 0) << " " << Q_openCV.at<double>(1, 1) << " " << Q_openCV.at<double>(1, 2) << " " << Q_openCV.at<double>(1, 3) << std::endl
	// 	<< Q_openCV.at<double>(2, 0) << " " << Q_openCV.at<double>(2, 1) << " " << Q_openCV.at<double>(2, 2) << " " << Q_openCV.at<double>(2, 3) << std::endl
	// 	<< Q_openCV.at<double>(3, 0) << " " << Q_openCV.at<double>(3, 1) << " " << Q_openCV.at<double>(3, 2) << " " << Q_openCV.at<double>(3, 3) << std::endl;

	//ReProject
	cv::Mat map11, map12;
	cv::Mat map21, map22;
	cv::Mat leftStereoUndistorted, rightStereoUndistorted;

	cv::initUndistortRectifyMap(left_cam_matrix, left_dist_coeffs, R1, P1, Left.size(), CV_16SC2, map11, map12);
	cv::initUndistortRectifyMap(right_cam_matrix, right_dist_coeffs, R2, P2, Right.size(), CV_16SC2, map21, map22);
	cv::remap(Left, leftStereoUndistorted, map11, map12, CV_INTER_LINEAR, BORDER_CONSTANT, cv::Scalar());
	cv::remap(Right, rightStereoUndistorted, map21, map22, CV_INTER_LINEAR, BORDER_CONSTANT, cv::Scalar());

	cv::imshow("Retifies Left", leftStereoUndistorted);
	cv::namedWindow("Retifies Left", CV_WINDOW_AUTOSIZE);

	cv::imshow("Retifies Right", rightStereoUndistorted);
	cv::namedWindow("Retifies Right", CV_WINDOW_AUTOSIZE);
	cv::waitKey(0); 

	/*
		Generate 3D scene from stereo images
	*/

	std::size_t vertexcount = 0;
	//finding Projection matrices of two images 
	cv::Mat edges;
	//load a colored image
	cv::Mat object_color = cv::imread("im0.png", CV_LOAD_IMAGE_COLOR);
	cv::Mat object_resize = cv::imread("im0.png", 0);
	cv::Mat scene_resize = cv::imread("im1.png", 0);

	int ndisparities = 16*4 ;  /**< Range of disparity */
	int SADWindowSize = 7;  /**< Size of the block window. Must be odd */

	///Try stereo SGBM
	cv::Mat sgbmimgDisparity16S2 = cv::Mat(frame.rows, frame.cols / 2, CV_16S);
	cv::Mat sgbmimgDisparity8U2 = cv::Mat(frame.rows, frame.cols / 2, CV_8UC1);
	int blockSize = 5;
	int uniqueR = 0;
	int prefilter_cap = 4;
	int speckleRange = 5;
	int speckleWindow = 150;
	int disp12MaxDiff = 10;
	int p1 = 600, p2 = 2400;
	int minDisp = 5;

	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(minDisp, ndisparities, blockSize, p1, p2, disp12MaxDiff, prefilter_cap, uniqueR, speckleWindow, speckleRange, 0);
	sgbm->compute(leftStereoUndistorted, rightStereoUndistorted, sgbmimgDisparity16S2);

	double minVal; double maxVal;
	minMaxLoc(sgbmimgDisparity16S2, &minVal, &maxVal);
	printf("Min disp: %f Max value: %f \n", minVal, maxVal);

	//sgbm
	sgbmimgDisparity16S2.convertTo(sgbmimgDisparity8U2, CV_8UC1, 255 / (maxVal - minVal));

	//floating point disparity Image, To be used further
	cv::Mat dispFloat;
	sgbmimgDisparity16S2.convertTo(dispFloat, CV_32F, 1. / 16);

	cv::namedWindow("Depth Map", CV_WINDOW_AUTOSIZE);
	cv::imshow("Depth Map", sgbmimgDisparity8U2);
	cv::waitKey(0);

	//Q Matrix will change according to New data 
	cv::Mat Q = (cv::Mat_<double>(4, 4) << 1, 0, 0, -1429.219,
		0, 1, 0, -993.403,
		0, 0, 0, 5806.559,
		0, 0, -(1 / -174.019), ((1429.219 - 1543.51) / -174.019));

	cv::Mat points;
	cv::reprojectImageTo3D(dispFloat, points, Q_openCV, false, -1);
	cv::patchNaNs(points, 0);

	//to Decide Clipping planes
	std::vector < cv::Mat> imgChannels;
	split(points, imgChannels);
	double Zmin, Zmax;
	cv::minMaxLoc(imgChannels[2], &Zmin, &Zmax);
	//std::cout << "Zmin: " << Zmin << "Zmax " << Zmax << std::endl; 

	cv::Vec3b intensity;
	float blue, green, red;
	int line_count = 0;
	unsigned char colorVal[3];

	vtkSmartPointer<vtkPoints> pointsVTK = vtkSmartPointer<vtkPoints>::New();

	///for colors 
	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");
	
	pcl::PointCloud<pcl::PointXYZ> cloud_out;
	pcl::PointXYZRGB point;
	pcl::PointXYZ pointBasic;
	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (unsigned int j = 0; j < dispFloat.cols; j++)
	{
		for (unsigned int i = 0; i < dispFloat.rows; i++)
		{
			//For basic point Cloud 
			pointBasic.x = points.at<cv::Vec3f>(i, j)[0];
			pointBasic.y = points.at<cv::Vec3f>(i, j)[1];
			pointBasic.z = points.at<cv::Vec3f>(i, j)[2];

			point.x = points.at<cv::Vec3f>(i, j)[0];
			point.y = points.at<cv::Vec3f>(i, j)[1];
			point.z = points.at<cv::Vec3f>(i, j)[2];

			//Adding color to the cloud
			intensity = Left.at<cv::Vec3b>(i, j);			
			point.r = intensity[2]; 
			point.g = intensity[1]; 
			point.b = intensity[0]; 

			point_cloud_ptr->points.push_back(point);
			basic_cloud_ptr->points.push_back(pointBasic);
			line_count++;
		}
	}

	//Filter the point cloud 
	//Passthough filter 
	//Voxelized filtered approach 

	//Bounding box of point cloud 
	
	// Compute principal directions
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*basic_cloud_ptr, pcaCentroid);
	Eigen::Matrix3f covariance;
	computeCovarianceMatrixNormalized(*basic_cloud_ptr, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  

	// Transform the original cloud to the origin where the principal components correspond to the axes.
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*basic_cloud_ptr, *cloudPointsProjected, projectionTransform);

	// Get the minimum and maximum points of the transformed cloud.
	pcl::PointXYZ minPoint, maxPoint;
	pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
	const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

	// std::cout << " Minimum XYZ " << minPoint.x << " " << minPoint.y << " " << minPoint.z << std::endl; 
	// std::cout << " Maximum XYZ " << maxPoint.x << " " << maxPoint.y << " " << maxPoint.z << std::endl;

	//Downsampling
	//Voxel Grid
	std::cerr << " Voxel PointCloud before filtering: " << point_cloud_ptr->size() << " data points (" << pcl::getFieldsList(*point_cloud_ptr) << ")." << std::endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampling(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(point_cloud_ptr);
	sor.setLeafSize(0.15f, 0.15f, 0.15f); //Working 
	sor.filter(*cloud_downsampling);
	std::cerr << "Voxel PointCloud After filtering: " << cloud_downsampling->size() << " data points (" << pcl::getFieldsList(*cloud_downsampling) << ")." << std::endl;

	///Radial Filter - Apply
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cleaned(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radius_outlier_removal;
	// set input cloud
	radius_outlier_removal.setInputCloud(point_cloud_ptr);
	
	// set radius for neighbor search
	radius_outlier_removal.setRadiusSearch(0.5); //Decreasing the radius search less the number of points
	
	// set threshold for minimum required neighbors neighbors
	radius_outlier_removal.setMinNeighborsInRadius(80);
	
	// do filtering
	radius_outlier_removal.filter(*cleaned);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::cerr << "PointCloud before filtering: " << basic_cloud_ptr->width * basic_cloud_ptr->height << " data points (" << pcl::getFieldsList(*basic_cloud_ptr) << ")." << std::endl;
	
	//Triangulation
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(point_cloud_ptr);
	n.setInputCloud(point_cloud_ptr);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);

	//* normals should not contain the point normals + surface curvatures
	// Concatenate the XYZ and normal fields*

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*basic_cloud_ptr, *normals, *cloud_with_normals);

	//* cloud_with_normals = cloud + normals
	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.5);
	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);
	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	//Get colors in the Generated Mesh ///
	/* Make a copy of the mesh cloud since it will be overwritten later. */
	pcl::PCLPointCloud2 vertexCloud = triangles.cloud;

	pcl::PCLPointCloud2 colorCloud;
	pcl::toPCLPointCloud2(*point_cloud_ptr, colorCloud);

	/*
	* Use all the fields of the input cloud (i.e. PointXYZRGBNormal) but override
	* the xyz field with the xyz data from the poisson reconstruction.
	* Note that the data of the 2nd cloud is used in case the two input cloud
	* have the same fields.
	*/

	pcl::concatenateFields(colorCloud, vertexCloud, triangles.cloud);
	//////////////////////////////////////

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
	<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	//viewer->addPointCloud(basic_cloud_ptr, "example"); //Code to Show polyData in visualizer directly 
	//viewer->addPointCloud(cloud_downsampling, "sample cloud"); //Code to Show pointCloud in visualizer  
	viewer->addPolygonMesh(triangles, "meshes", 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	
	return EXIT_SUCCESS;

}