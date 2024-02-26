
#pragma once
#include <liblas/liblas.hpp>


#include <vector>
#include <fstream>
#include <iostream>//Relevant header files for input and output classes in the standard C++ library.
#include <string>



//LAS file attribute information.
typedef struct LasProp {
	int num;
	double X_Offset;
	double Y_Offset;
	double Z_Offset;
	double Max_X;
	double Max_Y;
	double Max_Z;
	double Min_X;
	double Min_Y;
	double Min_Z;
	LasProp(){
		num = 0;
		X_Offset = Y_Offset = Z_Offset = Max_X = Max_Y = Max_Z = Min_X = Min_Y = Min_Z = 0.0;
	}
} LasProp;

//Point cloud data.
typedef struct lidardata
{
	LasProp prop;
	std::vector<double> x;
	std::vector<double> y;
	std::vector<double> z;
	std::vector<int> classify;
	std::vector<int> intensity;
};

//Tree position.
typedef struct treeLocation
{
	int num;
	std::vector<double> x;
	std::vector<double> y;
	std::vector<double> z;
	treeLocation(){
		num = 0;
	}
}treeLocation;

typedef struct singleFeature
{
	int num;
	int x;
	int  y;
	float x_true;
	float y_true;
	float  azimuth;//Azimuth angle
	float  dis2; //Square of the distance.
	singleFeature(){
		x = y = 0;
		azimuth = dis2 = 0.0;
	}
}singleFeature;

typedef struct treeFeature
{
	int num;
	std::vector<singleFeature> features;
	int match_num;
	treeFeature(){
		num = 0;
		match_num = 0;
	}
}treeFeature;

typedef struct mathchPoint
{
	int num;
	std::vector<int> x;
	std::vector<int> y;
	std::vector<int> z;
	mathchPoint(){
		num = 0;
	}
}mathchPoint;

typedef struct feature_score
{
	int num;
	int score;
	feature_score(){
		num = 0;
		score = 0;
	}
}feature_score;

typedef struct all_score
{
	std::vector<feature_score> scores;
	all_score(){
		
	}
}all_score;
typedef struct TIN
{
	double x1;
	double x2;
	double x3;
	double y1;
	double y2;
	double y3;
	double z1;
	double z2;
	double z3;
	float area;
	float max_angle;
	int alive;
	TIN()
	{
		x1 = x2 = x3 = y1 = y2 = y3 = z1 = z2 = z3 = 0;
		alive = 0;
	}
}TIN;

typedef struct all_TIN
{
	std::vector<TIN> tin;
	all_TIN()
	{	
	}
}all_TIN;