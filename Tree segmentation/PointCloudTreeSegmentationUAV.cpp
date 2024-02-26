#include "PointCloudTreeSegmentationUAV.h"

#include <vector> 
#include <math.h> 
#define _USE_MATH_DEFINES
#include <algorithm>
#include <iostream>
#include "data.h"
#include "circle.h"
#include "CircleFitByTaubin.h"

using namespace std;

#ifdef USE_GRID2D
struct gridCell2d{
	std::vector<int > index;
	float max_z;
	gridCell2d():max_z(-FLT_MAX){}
	gridCell2d(const gridCell2d & other):max_z(other.max_z),index(other.index){}
	inline void fill(const CPointBase& p,int type){
		index.push_back(type);
		if (max_z<p.z)
			max_z = p.z;
	}
};
#endif

//list of already used point to avoid hull's inner loops
enum HullPointFlags {
	POINT_NOT_USED	= 0,
	POINT_USED		= 1,
	POINT_IGNORED	= 2,
};

bool sortz_TLS(MYPointCTreeID a, MYPointCTreeID b)
{
	return a.z > b.z;
}

bool sortidz_TLS(MYPointCTreeID a, MYPointCTreeID b)
{
	if (a.treeID != b.treeID)
	{
		return a.treeID < b.treeID;
	}
	else
		return a.z > b.z;
}

class FindSegID
{
public:
	int id;
	explicit FindSegID(int ID){id = ID;};
	bool operator()(const MYPointCTreeID &rr){return rr.treeID == id;};
};

//Construct a 3D convex hull and calculate its volume

//Export the results to a CSV file
//@std::vector<st_treeAtt> &trees Input individual tree information
//@const std::string& csvPath An output path must be provided
bool PointCloudTreeSegmentation::outputToCSV(const std::vector<st_treeAtt>&trees, const std::string& csvPath)
{
	std::ofstream csvfile;
	csvfile.precision(3);
	csvfile.setf(std::ios::fixed);
	csvfile.setf(std::ios::showpoint);
	csvfile.open(csvPath);
	//header
	csvfile << "TreeID,TreeLocationX,TreeLocationY,TreeHeight\n" << std::flush;
	for (int i = 0; i < trees.size(); i++) {
		csvfile << trees[i].treeID << "," << trees[i].pos.x << "," << trees[i].pos.y << "," << trees[i].pos.z << "\n";
	}
	csvfile << std::flush;
	csvfile.close();
	return false;
}

void PointCloudTreeSegmentation::SortByZ(PtCloud& cloud)
{
	sort(cloud.begin(), cloud.end(), sortz_TLS/* [](cloud a, cloud b) { return a.z > b.z;}*/);
}

void PointCloudTreeSegmentation::SortByIDZ(PtCloud& cloud)
{
	sort(cloud.begin(), cloud.end(), sortidz_TLS);
}

//template<typename PointT>
void PointCloudTreeSegmentation::ComputeBoundingBox(PtCloud &cloud)
{
	auto result1 = minmax_element(cloud.begin(), cloud.end(), [](MYPointCTreeID a, MYPointCTreeID b) { return a.x > b.x;  });
	auto result2 = minmax_element(cloud.begin(), cloud.end(), [](MYPointCTreeID a, MYPointCTreeID b) { return a.y > b.y;  });
	auto result3 = minmax_element(cloud.begin(), cloud.end(), [](MYPointCTreeID a, MYPointCTreeID b) { return a.z > b.z;  });
	BoundingBox bounding_box;

	bounding_box.x_max_idx = result1.first - cloud.begin();
	bounding_box.x_min_idx = result1.second - cloud.begin();
	bounding_box.y_max_idx = result2.first - cloud.begin();
	bounding_box.y_min_idx = result2.second - cloud.begin();
	bounding_box.z_max_idx = result3.first - cloud.begin();
	bounding_box.z_min_idx = result3.second - cloud.begin();

	bounding_box.x_max = cloud[bounding_box.x_max_idx].x;
	bounding_box.x_min = cloud[bounding_box.x_min_idx].x;
	bounding_box.y_max = cloud[bounding_box.y_max_idx].y;
	bounding_box.y_min = cloud[bounding_box.y_min_idx].y;
	bounding_box.z_max = cloud[bounding_box.z_max_idx].z;
	bounding_box.z_min = cloud[bounding_box.z_min_idx].z;

	bounding_box.availability = true;

	bounding_box_ = bounding_box;
}

//template<typename PointT>
void PointCloudTreeSegmentation::AssignGridCells(PtCloud &cloud)
{
	if (!bounding_box_.availability){
		ComputeBoundingBox(cloud);
	}
#ifdef USE_GRID2D
	n_cols_ = ceil((bounding_box_.x_max - bounding_box_.x_min)/gride_size);
	n_rows_ = ceil((bounding_box_.y_max - bounding_box_.y_min)/gride_size);
#else
	n_cols_ = int((bounding_box_.x_max - bounding_box_.x_min)/gride_size)+1;
	n_rows_ = int((bounding_box_.y_max - bounding_box_.y_min)/gride_size)+1;
#endif

	std::cout << "nrows: " << n_rows_ << endl;
	cout << "ncols: " << n_cols_ << endl;

	vector<vector<int>> idx_grid(n_cols_*n_rows_);
	cout << "size: " <<  n_cols_*n_rows_ << endl;

	for(unsigned int j(0); j < cloud.size(); j++){

		unsigned int tempCol,tempRow;
		tempCol = (cloud[j].x - bounding_box_.x_min)/gride_size;		
#ifdef USE_GRID2D
		//tempRow = (bounding_box_.y_max - cloud[j].y)/gride_size;
		tempRow = (cloud[j].y - bounding_box_.y_min)/gride_size;
#else
		tempRow = (cloud[j].y - bounding_box_.y_min)/gride_size;
#endif
		if (tempCol == n_cols_)
			tempCol = n_cols_ - 1;
		if (tempRow == n_rows_)
			tempRow = n_rows_ - 1;
		if (tempCol >= n_cols_ || tempRow >= n_rows_)
			continue;
		//assert(tempRow*n_cols_ + tempCol < n_cols_* n_rows_);
		idx_grid[  tempRow*n_cols_ + tempCol ].push_back(j);
	}
	idx_grid_ = idx_grid;
}

//template<typename PointT>
void PointCloudTreeSegmentation::ExtractPointsInBuffer(CircularBuffer& circular_buffer,std::vector<int>& index,int& col_0, int& row_0,float z)
{
	//vector<int> tmp_idx;
	int col_idx, row_idx;
#ifdef USE_GRID2D
	const std::vector<gridCell2d *>& data = m_grid2d.constData();
#endif
	
	for(unsigned int j(0); j < circular_buffer.coordinate_offsets_.size(); j++){

		// Check if row and column are inside the grid
		col_idx = col_0 + circular_buffer.coordinate_offsets_[j][0];
		row_idx = row_0 + circular_buffer.coordinate_offsets_[j][1];

		// Check if the kernel cell is located within the grid
		if (((unsigned int)row_idx <= n_rows_-1) && ((unsigned int)row_idx >= 0) && ((unsigned int)col_idx <= n_cols_-1) && ((unsigned int)col_idx >= 0)){

#ifdef USE_GRID2D
			gridCell2d * cell = data[row_idx]+col_idx;
			vector<int>& tmp_idx = cell->index;
#else
			vector<int>&tmp_idx = idx_grid_[n_cols_*row_idx+col_idx]; // Grid cell linear index
#endif
			if (!tmp_idx.empty()
				#ifdef USE_GRID2D
				&& cell->max_z > z
				#endif
				)
			{
				int ncount = index.size();
				index.resize(ncount+tmp_idx.size());
				for (size_t i = ncount; i < index.size(); ++i)
					index[i] = tmp_idx[i - ncount];
// 				for(unsigned int k(0); k < tmp_idx.size(); k++){
// 						index.push_back(tmp_idx[k]); 
// 					}
			}
		}
	}
}

//template<typename PointT>
double PointCloudTreeSegmentation::FindRadius(int col,int row,double maxz,PtCloud& cloud)
{
	//vector<int> tmp_idx;
	int col_idx, row_idx;
	vector<std::array<int,2>>coordinate_offsets;
	coordinate_offsets.resize(8);
	coordinate_offsets[0][0] = 1;  //col
	coordinate_offsets[0][1] = 0; //row X positive direction
	coordinate_offsets[1][0] = 1;
	coordinate_offsets[1][1] = 1;
	coordinate_offsets[2][0] = 0;
	coordinate_offsets[2][1] = 1; // Y positive direction

	coordinate_offsets[3][0] = -1;
	coordinate_offsets[3][1] = 1;
	coordinate_offsets[4][0] = -1;
	coordinate_offsets[4][1] = 0;
	coordinate_offsets[5][0] = -1;
	coordinate_offsets[5][1] = -1;
	coordinate_offsets[6][0] = 0;
	coordinate_offsets[6][1] = -1;
	coordinate_offsets[7][0] = 1;
	coordinate_offsets[7][1] = -1;
	vector<std::array<int,2>>offsets;
	offsets.resize(8);
#ifdef USE_GRID2D
	const std::vector<gridCell2d *>& data = m_grid2d.constData();
#endif
	

	for (unsigned int j=0;j<8;j++)
	{
		col_idx = col;
		row_idx = row;
		double dirz = maxz;
		while (1)
		{
			col_idx+=coordinate_offsets[j][0];
			row_idx+=coordinate_offsets[j][1];
			// Check if the kernel cell is located within the grid
			if (((unsigned int)row_idx <= n_rows_-1) && 
				((unsigned int)row_idx >= 0) &&
				((unsigned int)col_idx <= n_cols_-1) &&
				((unsigned int)col_idx >= 0))
			{
				
#ifdef USE_GRID2D
				gridCell2d * cell = data[row_idx]+col_idx;				
				vector<int> &tmp_idx = cell->index;
#else
				vector<int> &tmp_idx = idx_grid_[n_cols_*row_idx+col_idx]; // Grid cell linear index
#endif
				// Check if the grid cell is non-empty
				if (!tmp_idx.empty()){
					double tempz = 0;
#ifdef USE_GRID2D
					tempz = cell->max_z;
#else
					for(unsigned int k(0); k < tmp_idx.size(); k++){

						// Check if the point is non-segmented
							if (cloud[tmp_idx[k]].z > tempz)
							{
								tempz = cloud[tmp_idx[k]].z;
							}
					} 
#endif
					if (dirz >= tempz && tempz > 0)
					{
						dirz = tempz;
					}
					else
					{
						offsets[j][0] = col_idx-1;
						offsets[j][1] = row_idx-1;
						break;
					}


				}
			}
			else
				break;
		}
	}
	vector<double> result;
	for (int i=0;i<8;i++)
	{
		if ((offsets[i][0]<=0) && (offsets[i][1]<= 0))
		{
			continue;
		}
		result.push_back((offsets[i][0]-col)*(offsets[i][0]-col)+(offsets[i][1]-row)*(offsets[i][1]-row));
	}
	//result.push_back((offsets[0][0]-col)*(offsets[0][0]-col)+(offsets[0][1]-row)*(offsets[0][1]-row));
	//result.push_back((offsets[1][0]-col)*(offsets[1][0]-col)+(offsets[1][1]-row)*(offsets[1][1]-row));
	//result.push_back((offsets[2][0]-col)*(offsets[2][0]-col)+(offsets[2][1]-row)*(offsets[2][1]-row));
	//result.push_back((offsets[3][0]-col)*(offsets[3][0]-col)+(offsets[3][1]-row)*(offsets[3][1]-row));
	//result.push_back((offsets[4][0]-col)*(offsets[4][0]-col)+(offsets[4][1]-row)*(offsets[4][1]-row));
	//result.push_back((offsets[5][0]-col)*(offsets[5][0]-col)+(offsets[5][1]-row)*(offsets[5][1]-row));
	//result.push_back((offsets[6][0]-col)*(offsets[6][0]-col)+(offsets[6][1]-row)*(offsets[6][1]-row));
	//result.push_back((offsets[7][0]-col)*(offsets[7][0]-col)+(offsets[7][1]-row)*(offsets[7][1]-row));
	double radius = 0;
	if (result.size()>3)
	{
		sort(result.begin(),result.end());
		for (int i=1;i<(result.size()-1);i++)
		{
			radius += sqrt(result[i]);
		}
		radius = radius/(result.size()-2);
	}
	else
		radius = 2;

	return radius/*(sqrt(result[3])+sqrt(result[4]))/2.0*/;
	/*vector<double> temp;
	double xdir = (sqrt(result[0])+sqrt(result[4]))/2.0;
	double ydir = (sqrt(result[2])+sqrt(result[6]))/2.0;
	double x45  = (sqrt(result[1])+sqrt(result[5]))/2.0; 
	double x135 = (sqrt(result[3])+sqrt(result[7]))/2.0;
	temp.push_back(xdir);
	temp.push_back(ydir);
	temp.push_back(x45);
	temp.push_back(x135);
	sort(temp.begin(), temp.end());
	return (temp[2]+temp[3])/2.0*gride_size;*/
}

void PointCloudTreeSegmentation::SetPara(double m_gridsize,bool isMulti_cal/* = false*/)
{
	gride_size = m_gridsize;
	_multi_cal = isMulti_cal;
}

void PointCloudTreeSegmentation::SetMinTreeHeight( double minTreeHeight )
{
	m_minTreeHeight = minTreeHeight;
}

void PointCloudTreeSegmentation::SetCurBox(double m_xmin,double m_xmax,double m_ymin,double m_ymax)
{
	_xmin = m_xmin;
	_xmax = m_xmax;
	_ymin = m_ymin;
	_ymax = m_ymax;
}

//template<typename PointT>
bool PointCloudTreeSegmentation::RunSegmentation(PtCloud &cloud,std::vector<PointCloudTreeSegmentation::st_treeAtt>&trees)
{
#ifdef USE_GRID2D
	m_box.setMinimal();
	for (auto itr = cloud.begin();itr!=cloud.end();++itr){
		liCommon::CVector3D<double> p(itr->x,itr->y,itr->z);
		m_box.include(p);
	}
#endif


	ComputeBoundingBox(cloud);
	double xmin = bounding_box_.x_min;
	double ymin = bounding_box_.y_min;
	double zmin = bounding_box_.z_min;
	//ƽ��BoundingBox
	bounding_box_.x_min -= xmin;
	bounding_box_.y_min -= ymin;
	bounding_box_.z_min -= zmin;
	bounding_box_.x_max -= xmin;
	bounding_box_.y_max -= ymin;
	bounding_box_.z_max -= zmin;

	for(int i=0;i<cloud.size();i++)
	{
		cloud[i].x -= xmin;
		cloud[i].y -= ymin;
		cloud[i].z -= zmin;
	}
	SortByZ(cloud);

#ifdef USE_GRID2D
	m_box._min.x -= xmin;
	m_box._min.y -= ymin;
	m_box._min.z -= zmin;
	m_box._max.x -= xmin;
	m_box._max.y -= ymin;
	m_box._max.z -= zmin;
	init2dGrid(cloud,gride_size);
#else
	AssignGridCells(cloud);
#endif
	

	int sid = 0;
	int np = -1;

	std::vector<int> index;
	std::vector<bool> gridstatus;
	gridstatus.resize((unsigned long long)(n_cols_)*n_rows_,false);
// 	for(int i=0;i<n_cols_*n_rows_;i++)
// 	{
// 		gridstatus.push_back(0);
// 	}
	/*ofstream o_file("liforestsegmentationTreeTop.csv");*/
	PtCloud treeTop;
	std::vector<double> treeTopRadius;
	//CLog::instance()->print("Begin1");
#pragma region  //segmentation
	for (unsigned int i = 0; i < cloud.size(); i++)
	{
		if (cloud[i].treeID != 0)
		{
			continue;
		}
		int tempCol, tempRow;
#ifdef USE_GRID2D
		tempCol = m_grid2d.col(cloud[i].x);
		tempRow = m_grid2d.row(cloud[i].y);

		if (tempCol == m_grid2d.width())
			--tempCol;
		if (tempRow == m_grid2d.height())
			--tempRow;

		if (tempCol < 0 ||
			tempCol > m_grid2d.width() ||
			tempRow < 0 ||
			tempRow > m_grid2d.height())
		{
			continue;
		}
#else
		tempCol = (cloud[i].x - bounding_box_.x_min) / gride_size;
		tempRow = (cloud[i].y - bounding_box_.y_min) / gride_size;
#endif

		double radius = 2;
		if (!gridstatus[n_cols_*tempRow + tempCol] /*== 0*/)
		{
			radius = FindRadius(tempCol, tempRow, cloud[i].z, cloud);
			if (radius >= cloud[i].z)
			{
				radius = cloud[i].z;
			}
			gridstatus[n_cols_*tempRow + tempCol] = true/*1*/;
		}
		index.clear();
		CircularBuffer tempbuffer(radius, gride_size);
		ExtractPointsInBuffer(tempbuffer, index, tempCol, tempRow, cloud[i].z);
		if (index.size() > 0)
		{
			double dis_t;
			for (unsigned int j = 0; j < index.size(); j++)
			{
				if (i != index[j])
				{
					if (cloud[i].z </*=*/ cloud[index[j]].z)
					{

						double DistanceJ = (cloud[i].x - cloud[index[j]].x)*(cloud[i].x - cloud[index[j]].x) +
							(cloud[i].y - cloud[index[j]].y)*(cloud[i].y - cloud[index[j]].y);
						//if (gridstatus[n_cols_*tempRow+tempCol] == 0) //Local maximum value
						//{
						double distanceRJ = (cloud[i].x - treeTop[cloud[index[j]].treeID - 1].x)*(cloud[i].x - treeTop[cloud[index[j]].treeID - 1].x) +
							(cloud[i].y - treeTop[cloud[index[j]].treeID - 1].y)*(cloud[i].y - treeTop[cloud[index[j]].treeID - 1].y);
						if (distanceRJ >= treeTopRadius[cloud[index[j]].treeID - 1]/**treeTopRadius[cloud[index[j]].treeID-1]*/)
							continue;
						//}
						if (np < 0)
						{
							np = j;
							dis_t = DistanceJ;
						}
						else if (np >= 0)
						{
							// 							double Distancenp = (cloud[i].x-cloud[index[np]].x)*(cloud[i].x-cloud[index[np]].x)+
							// 								(cloud[i].y-cloud[index[np]].y)*(cloud[i].y-cloud[index[np]].y);
														/*if (gridstatus[n_cols_*tempRow+tempCol] == 0)
														{*/
														//double DistancerRnp = (cloud[i].x-treeTop[cloud[index[np]].treeID-1].x)*(cloud[i].x-treeTop[cloud[index[np]].treeID-1].x)+
														//	(cloud[i].y-treeTop[cloud[index[np]].treeID-1].y)*(cloud[i].y-treeTop[cloud[index[np]].treeID-1].y);
														//if(DistancerRnp >= treeTopRadius[cloud[index[np]].treeID-1]/**treeTopRadius[cloud[index[np]].treeID-1]*/)
														//	continue;
													//}
							if (DistanceJ < dis_t/*Distancenp*/)
							{
								np = j;
								dis_t = DistanceJ;
							}

						}
					}
				}
			}

			if (np < 0)
			{
				sid++;
				cloud[i].treeID = sid;
				treeTop.push_back(cloud[i]);
				treeTopRadius.push_back(radius*radius);
				//o_file << cloud[i].x+xmin << ", " << cloud[i].y+ymin << ", " << cloud[i].z+zmin << ", " << cloud[i].treeID <<","<<radius<<  endl;

			}
			else
			{
				cloud[i].treeID = cloud[index[np]].treeID;
				np = -1;
			}
		}
#ifdef USE_GRID2D
		else {
			if (np < 0)
			{
				sid++;
				cloud[i].treeID = sid;
				treeTop.push_back(cloud[i]);
				treeTopRadius.push_back(radius*radius);
			}
		}
#endif

		}

	SortByIDZ(cloud);
	//CLog::instance()->print("Begin2");
#pragma endregion

#pragma region  //merge
	int idnum = cloud[cloud.size() - 1].treeID;
	int* idindex = new int[idnum + 1];
	for (int i = 0; i < idnum; i++)
	{
		idindex[i] = std::distance(cloud.begin(), find_if(cloud.begin(), cloud.end(), FindSegID(i + 1)));
	}
	idindex[idnum] = cloud.size();
	vector<int>areastatis;
	vector<vector<int>> treeid;
	for (int i = 0; i < idnum; i++)
	{
		areastatis.clear();
		int size = idindex[i + 1] - idindex[i];
		if (size < 50)  //Filter out if the number of points is less than 50
		{
			for (int j = idindex[i]; j < idindex[i + 1]; j++)
			{
				cloud[j].treeID = 0; //-1
			}
			continue;
		}
		////Filter out if the area is less than a certain value
		for (int j = idindex[i]; j < idindex[i + 1]; j++)
		{
			int tempCol, tempRow;
			tempCol = (cloud[j].x - bounding_box_.x_min) / gride_size;
			tempRow = (cloud[j].y - bounding_box_.y_min) / gride_size;
			std::vector<int>::iterator iter = std::find(areastatis.begin(), areastatis.end(), tempRow*n_cols_ + tempCol);
			if (iter == areastatis.end())
			{
				areastatis.push_back(tempRow*n_cols_ + tempCol);
			}
		}

		if (areastatis.size() < 3)
		{
			for (int j = idindex[i]; j < idindex[i + 1]; j++)
			{
				cloud[j].treeID = 0; // -1
			}
			continue;
		}
		vector<int> temptreeid;
		for (int j = idindex[i]; j < idindex[i + 1]; j++)
		{
			temptreeid.push_back(j);
		}
		treeid.push_back(temptreeid);
	}
	//Release memory
	if (idindex != NULL)
	{
		delete idindex;
		idindex = NULL;
	}
#pragma  endregion

#pragma region //Correct the TreeID
	int sortid = 1;
	treeTop.clear();
	for (int i = 0; i < treeid.size(); i++)
	{
		for (int j = 0; j < treeid[i].size(); j++)
		{
			cloud[treeid[i][j]].treeID = sortid;
			if (j == 0)
			{
				treeTop.push_back(cloud[treeid[i][j]]);
			}
		}
		sortid++;
	}
#pragma endregion

#pragma region //Merge fragmented patches

	for (unsigned int i = 0; i < cloud.size(); i++)
	{
		if (cloud[i].treeID == 0)   // -1
		{
			np = -1;
			for (int j = 0; j < treeTop.size(); j++)
			{
				if (cloud[i].z <= treeTop[j].z)
				{

					double DistanceJ = (cloud[i].x - treeTop[j].x)*(cloud[i].x - treeTop[j].x) +
						(cloud[i].y - treeTop[j].y)*(cloud[i].y - treeTop[j].y);
					if (np < 0)
					{
						np = j;
					}
					else if (np >= 0)
					{
						double Distancenp = (cloud[i].x - treeTop[np].x)*(cloud[i].x - treeTop[np].x) +
							(cloud[i].y - treeTop[np].y)*(cloud[i].y - treeTop[np].y);
						if (DistanceJ < Distancenp)
						{
							np = j;
						}

					}
				}
			}
			if (np >= 0)
			{
				cloud[i].treeID = treeTop[np].treeID;
			}
		}
	}

	SortByIDZ(cloud);
#pragma endregion

#pragma  region //Calculate attributes


	idnum = cloud[cloud.size() - 1].treeID;
	if (idnum < 0 || _isnan(idnum))
	{
		return false;
	}
	idindex = new(std::nothrow) int[idnum + 1];
	if (idindex == nullptr)
	{
		return false;
	}
	for (int i = 0; i < idnum; i++)
	{
		idindex[i] = std::distance(cloud.begin(), find_if(cloud.begin(), cloud.end(), FindSegID(i + 1)));
	}
	idindex[idnum] = cloud.size();
	treeid.clear();
	for (int i = 0; i < idnum; i++)
	{
		bool isboundary = false;
		for (int j = idindex[i]; j < idindex[i + 1]; j++)
		{
			if ((cloud[j].x < (_xmin - xmin)) || (cloud[j].x > (_xmax - xmin)) || (cloud[j].y < (_ymin - ymin)) || (cloud[j].y > (_ymax - ymin)))  //�����ӱ�����
			{
				isboundary = true;
				j = idindex[i + 1]; //Break out of the loop
			}
		}
		if (!isboundary)
		{
			vector<int> temptreeid;
			for (int j = idindex[i]; j < idindex[i + 1]; j++)
			{
				temptreeid.push_back(j);
			}
			treeid.push_back(temptreeid);
		}
		else
		{
			for (int j = idindex[i]; j < idindex[i + 1]; j++)
			{
				cloud[j].treeID = 0;
			}
		}
	}
	//Correct the TreeId
	sortid = 1;
	treeTop.clear();
	for (int i = 0; i < treeid.size(); i++)
	{
		for (int j = 0; j < treeid[i].size(); j++)
		{
			cloud[treeid[i][j]].treeID = sortid;
			if (j == 0)
			{
				treeTop.push_back(cloud[treeid[i][j]]);
			}
		}
		sortid++;
	}
	//Begin calculating attributes
	SortByIDZ(cloud);
	//Release memory
	if (idindex != NULL)
	{
		delete idindex;
		idindex = NULL;
	}
	idnum = cloud[cloud.size() - 1].treeID;
	idindex = new(std::nothrow) int[idnum + 1];
	if (idindex == nullptr)
	{
		return false;
	}
	for (int i = 0; i < idnum; i++)
	{
		idindex[i] = std::distance(cloud.begin(), find_if(cloud.begin(), cloud.end(), FindSegID(i + 1)));
	}
	idindex[idnum] = cloud.size();
	for (int i = 0; i < idnum; i++)
	{
		double area = 0;
		double volume = 0;
		double X = cloud[idindex[i]].x + xmin /*+ offset.x*/;
		double Y = cloud[idindex[i]].y + ymin /*+ offset.y*/;
		double Z = cloud[idindex[i]].z + zmin/*+zoffset*/;
		//Add a minimum tree height restriction
		if (Z < m_minTreeHeight)
		{
			for (int j = idindex[i]; j < idindex[i + 1]; j++)
			{
				cloud[j].treeID = 0;
			}
			continue;
		}

		PtCloud simpleTreeCloud(idindex[i + 1] - idindex[i]);
		for (int j = idindex[i]; j < idindex[i + 1]; j++)
		{
			simpleTreeCloud[j - idindex[i]].x = cloud[j].x;
			simpleTreeCloud[j - idindex[i]].y = cloud[j].y;
			simpleTreeCloud[j - idindex[i]].z = cloud[j].z;
		}
		/*liAlgorithm::CExtractSimpleTreeAttribution<PointT> att;
		att.computeAreaAndVolume(simpleTreeCloud, area, volume, false);*/

		double r = 2 * sqrt(area / 3.1415926);
		PointCloudTreeSegmentation::st_treeAtt treept;
		treept.treeID = cloud[idindex[i]].treeID /*+ addTreeid*/;
		treept.pos = PointT(X, Y, Z);
		treept.area = area;
		treept.diameter = r;
		treept.volume = volume;
		trees.push_back(treept);
		//o<<cloud[idindex[i]].treeID+addTreeid<<","<<X<< "," <<Y<< "," <<Z<< "," <<r<< "," <<area<< "," <<volume<<"\n"<<std::flush;
	}

	//Release memory
	if (idindex != NULL)
	{
		delete idindex;
		idindex = NULL;
	}

	//Calculate attributes
#pragma endregion


	for (int i = 0; i < cloud.size(); i++)
	{
		cloud[i].x += xmin;
		cloud[i].y += ymin;
		cloud[i].z += zmin;
	}
	return true;
}

#ifdef USE_GRID2D

void PointCloudTreeSegmentation::init2dGrid(const PtCloud& clouds,double cellSize){

	/*unsigned int w*/n_cols_ = ceil(m_box.width()/cellSize);
	/*unsigned int h*/n_rows_ = ceil(m_box.length()/cellSize);
	//m_grid2d.clear();
	m_grid2d.init(n_cols_,n_rows_);
	m_grid2d.setBox(m_box);
	m_grid2d.setXGridStep(cellSize);
	m_grid2d.setYGridStep(cellSize);

	//fill
	for (int i = 0;i<clouds.size();++i){
		liCommon::CPointBase p(clouds[i].x,clouds[i].y,clouds[i].z);
		m_grid2d.fill(p,i);
	}
}
#endif // USE_GRID2D
