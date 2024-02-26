/**       
 * @file    PointCloudStruct.h                                                   
 * @brief                                                                            
 * @author  Ding                                                               
 * @data    2024-02-05                                               
 * @since                                                                            
 * @ingroup                                                                          
 * @Copyright (c)2014-2022,Beijing GreenValley Technologies, Inc.                    
 * @note                                                                             
 * @version <version number>                                                         
 *-------------------------------------------------------------------         
 * Change Log :                                                                      
 * 2024-02-05 | <version number> | Your Name| Create          
 *
 */                                                                      
#pragma once

#include <vector>

struct MYPoint
{
	MYPoint() {};
	MYPoint(double x_, double y_, double z_)
		:x(x_), y(y_), z(z_)
	{}

	double x = 0;
	double y = 0;
	double z = 0;
};

struct MYPointCTreeID : public MYPoint
{
	MYPointCTreeID() {};
	MYPointCTreeID(double x_, double y_, double z_) : MYPoint(x_, y_, z_)
	{}
	unsigned char classfication = 0;
	unsigned int treeID = 0;
	unsigned long long tilePos = 0;
};

struct LasProp
{
	//
};

struct lidardata
{
	LasProp prop;
	std::vector<double> x;
	std::vector<double> y;
	std::vector<double> z;
	std::vector<int> classify;
	std::vector<int> intensity;
};


namespace PointCloudStructUtil
{
	//ding: ¶îÍâ²¹³ä×ª»»º¯Êý
	void lidardata_to_MYPoint(const lidardata& data1, std::vector<MYPoint>& pts);
}