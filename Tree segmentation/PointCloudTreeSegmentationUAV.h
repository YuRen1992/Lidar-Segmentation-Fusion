/*
 //Call example (It is recommended to downsample the point cloud first).

 PointCloudTreeSegmentation::PtCloud ptVec(filterClouds.size());
 //Run single-tree segmentation.
 PointCloudTreeSegmentation pcs_segmentation;
 pcs_segmentation.SetPara(2.0);
 pcs_segmentation.SetCurBox(worldbox.xMinimum(), worldbox.xMaximum(), worldbox.yMinimum(), worldbox.yMaximum());
 pcs_segmentation.setHeightAbouveGround(m_para->HeightAbouveGround);
 pcs_segmentation.SetMinTreeHeight(m_para->MinTreeHeight);

 std::vector<PointCloudTreeSegmentation::st_treeAtt>trees;
 unsigned int treeid = 0;
 if (pcs_segmentation.RunSegmentation(ptVec, trees)) {
	 。。。。。
 }

 //Export to CSV.
 std::string myFilePath = "yourname.csv";
 pcs_segmentation.outputToCSV(trees, myFilePath);
 */    

#pragma once
#include <vector>
#include <fstream>
#include "CircularBuffer.h"

//#define USE_GRID2D
#ifdef USE_GRID2D
#include "CGrid2d.h"
#endif

#include "PointCloudStruct.h"

class PointCloudTreeSegmentation
{
public:
	typedef MYPointCTreeID PointT;
	typedef std::vector<PointT> PtCloud;
	struct st_treeAtt{
		PointT pos;
		double area;
		double diameter;
		double volume;
		int treeID;
		st_treeAtt()
			:pos()
			,area(0)
			,diameter(0)
			,treeID(0)
			,volume(0)
		{

		}
		st_treeAtt(const st_treeAtt & other)
			:pos(other.pos)
			,area(other.area)
			,diameter(other.diameter)
			,treeID(other.treeID)
			,volume(other.volume)
		{
		}
	};
public:
	//Set parameters��
	//Set grid size, default is 2.0.
	void SetPara(double gridsize,bool isMulit_cal = false);
	//et minimum tree height; trees shorter than this height will not be considered in the statistics.
	void SetMinTreeHeight(double minTreeHeight);
	//Set the xy range of the point cloud.
	void SetCurBox(double m_xmin,double m_xmax,double m_ymin,double m_ymax);
	//Set the height above ground points; only points above this height will be considered for segmentation based on their height.
	void setHeightAbouveGround(double heightAbouveGround){m_heightAbouveGround = heightAbouveGround;}
	
	//Perform single-tree segmentation.
	//@PtCloud &cloud Point cloud.
	//@std::vector<st_treeAtt> &trees Output information for individual trees.
	//Parameters need to be set sequentially before using this interface.
	bool RunSegmentation(PtCloud &cloud, std::vector<st_treeAtt>&trees);
	
	//Export the results to CSV.
	//@std::vector<st_treeAtt> &trees Input information for individual trees.
	//@const std::string& csvPath Output path needs to be provided.
	bool outputToCSV(const std::vector<st_treeAtt>&trees, const std::string& csvPath);

private:
	void SortByZ(PtCloud& cloud);
	void SortByIDZ(PtCloud& cloud); //ID in ascending order; Z (height) in descending order.
	void ComputeBoundingBox(PtCloud &cloud);
	void AssignGridCells(PtCloud &cloud);
	void ExtractPointsInBuffer(CircularBuffer& circular_buffer,std::vector<int>& index,int& col_0, int& row_0,float z);
	double FindRadius(int col,int row,double maxz,PtCloud& cloud);
#ifdef USE_GRID2D
	void init2dGrid(const PtCloud& clouds,double cellSize);
#endif
private:
	bool _multi_cal;
	/**
	 * Bounding box representing the horizontal extent of a PointCollection.
	 *
	 */
	struct BoundingBox {
		
		bool availability;
		double width;
		double height;
		double x_min;
		double x_max;
		double y_min;
		double y_max;
		double z_min;
		double z_max;
		unsigned int x_min_idx;
		unsigned int x_max_idx;
		unsigned int y_min_idx;
		unsigned int y_max_idx;
		unsigned int z_min_idx;
		unsigned int z_max_idx;
	};
	/**
	 * Bounding box.
	 *
	 */
	BoundingBox bounding_box_;
	
	/**
	 * Grid.
	 *
	 */
	std::vector<std::vector<int>> idx_grid_;
	
	/**
	 * Number of grid columns.
	 *
	 */
	unsigned int n_cols_;
	
	/**
	 * Number of grid rows.
	 *
	 */
	unsigned int n_rows_;
	
	/**
	 * Coordinate scaling factor (1 metric, 10 = decimetric, 100 = centimetric, etc...) used in gridding.
	 *
	 */
	double gride_size = 2.0;

	double _xmin;
	double _xmax;
	double _ymin;
	double _ymax;
#ifdef USE_GRID2D
	liGrid::CGrid2d<gridCell2d,liCommon::CPointBase > m_grid2d;
	liCommon::Box m_box;
#endif

	double m_heightAbouveGround = 2.0;
	double m_minTreeHeight = 1.0;
};