#pragma once
#include "MyStruct.h"
#include <pcl/registration/transformation_estimation_svd.h> 
#include <pcl/registration/icp.h>
#include <opencv2/opencv.hpp>
#include <pcl/search/kdtree.h>
#include <algorithm>
#include <gdal_priv.h>
#include "BuildTIN.h"
using namespace std;



class Registration
{
public:
	Registration();
	//Constructing features (UAV point cloud, Backpack point cloud, UAV position, Backpack position).
	bool GenerateFeature(lidardata &UAV_Data, lidardata &Backpack_Data, treeLocation &UAV_Tree, treeLocation &Backpack_Tree);

	//Bilding tree position images (input point cloud, input tree positions, tree position images, tree position image coordinates x, tree position image coordinates y).
	bool GenerateTreeLoactionMap(lidardata &data, treeLocation &tree, cv::Mat& image, std::vector<int> &tree_x, std::vector<int> &tree_y);

	//Nearest neighbor search, combining neighboring points into groups (tree image, tree position image coordinates x, tree position image coordinates y, features).
	bool FindAdjacentTree(treeLocation &tree, std::vector<int> &tree_x, std::vector<int> &tree_y, std::vector<treeFeature>& feature);

	//Calculate azimuth angle.
	float CalAzimuth(float xa, float ya, float xb, float yb);

	//Sort by azimuth angle magnitude.
	static bool BiggerAzimuth(singleFeature feature1, singleFeature feature2);
	//Sort by point size.
	static bool SmallerScore(feature_score a, feature_score b);

	//Generate TIN.
	std::vector<TIN> buildTIN(string name, int num, treeFeature &feature, lidardata &data, double &treex, double &treey, std::vector<TIN>& tin);

	//TIN matching.
	bool TINMatch(string Dir_path, lidardata &UAV_Data, lidardata &Backpack_Data, treeLocation &UAV_Tree, treeLocation &Backpack_Tree);
	~Registration();
	
	//lidardata data1;  //Point cloud data.
	//lidardata data2;
	//treeLocation tree1;//Tree position data.
	//treeLocation tree2;
	//UAV1 Backpack2
	std::vector<int> tree1_x;//Image coordinates of tree positions.
	std::vector<int> tree1_y;
	std::vector<int> tree2_x;
	std::vector<int> tree2_y;

	std::vector<treeFeature> tree1_feature;//Tree features.
	std::vector<treeFeature> tree2_feature;

	cv::Mat img1;//Image files used for visualization purposes.
	cv::Mat img2;

	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation;

	mathchPoint match1; 
	mathchPoint match2;

	lidardata test1;  //Point cloud data.
	lidardata test2;
};

