#include "PointCloudTreeSegmentationBackpack.h"

#include <algorithm>
#include <iostream>

#include "OctreeThin.h"
#include "dbscan.h"
#include "utils.h"
#include "kdtree2.hpp"
#include "CircleFitByTaubin.h"
using namespace std;

//list of already used point to avoid hull's inner loops
enum HullPointFlags 
{
	POINT_NOT_USED = 0,
	POINT_USED = 1,
	POINT_IGNORED = 2,
};

bool sortz(MYPointCTreeID a, MYPointCTreeID b)
{
	return a.z > b.z;
}

bool sortidz(MYPointCTreeID a, MYPointCTreeID b)
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
	explicit FindSegID(int ID) { id = ID; };
	bool operator()(const MYPointCTreeID &rr) { return rr.treeID == id; };
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//TLS Point Cloud Segmentation
CTLSPointCloudTreeSegmentation::CTLSPointCloudTreeSegmentation(const CTLSPointCloudTreeSegmentationPara& para)
	:m_para(para)
{

}

bool CTLSPointCloudTreeSegmentation::doSegment(PtCloud& pInCloud)
{
	bool re1 = TlsPointThin(pInCloud, m_para.m_thinGridSize);
	bool re2 = TlsTrunkSearch(m_para.trunkheight, m_para.m_buffer);
	bool re3 = TlsTreeHeightCalculate();
	bool re4 = SegmentationOriginTree(pInCloud);
	return re1 && re2 && re3 && re4;
}

CTLSPointCloudTreeSegmentation::~CTLSPointCloudTreeSegmentation()
{
	ThinPoint.clear();
	TreeSeed.clear();
}

bool CTLSPointCloudTreeSegmentation::TlsPointThin(PtCloud& pInCloud, double ThinGridSize)
{
	//Do not thin out
	for (int i = 0; i<pInCloud.size(); i++){
		PointT temppoint;
		temppoint.x = pInCloud[i].x;
		temppoint.y = pInCloud[i].y;
		temppoint.z = pInCloud[i].z;
		temppoint.treeID = 0;
		ThinPoint.push_back(temppoint);
	}
	return true;
	
	//This version always recreates the index file
	if (ThinGridSize <= 0.01)
		ThinGridSize = 0.01;

	double** xyz_in = NULL;
	xyz_in = (double**)malloc(pInCloud.size() * sizeof(double*));
	if (!xyz_in)
	{
		//logfile<<"New **xyz_in Failed!"<<"\n"<<std::flush;
		return false;
	}
	xyz_in[0] = (double*)malloc(pInCloud.size() * 3 * sizeof(double));
	if (!xyz_in[0])
	{
		//logfile<<"New *xyz_in Failed!"<<"\n"<<std::flush;
		free(xyz_in);
		return false;
	}

	double *xtest = *xyz_in;
	for (int i = 1; i<pInCloud.size(); i++)
		xyz_in[i] = *xyz_in + i * 3;
	memset(xyz_in[0], 0, pInCloud.size() * 3);

	for (unsigned int i = 0; i<pInCloud.size(); i++)
	{
		xyz_in[i][0] = pInCloud[i].x;   //Add calculation of maximum and minimum values
		xyz_in[i][1] = pInCloud[i].y;
		xyz_in[i][2] = pInCloud[i].z;
	}
	//logfile<<"xyz_in Assignment successful!"<<"\n"<<std::flush;
	BOctTree<double> *oct = new BOctTree<double>(xyz_in, pInCloud.size(), ThinGridSize, PointType(0), true);
	vector<double*> center;
	center.clear();
	center.swap(center);
	oct->GetOctTreeRandom(center);
	for (int i = 0; i<center.size(); i++)
	{
		PointT temppoint;
		temppoint.x = center[i][0];
		temppoint.y = center[i][1];
		temppoint.z = center[i][2];
		temppoint.treeID = 0;
		ThinPoint.push_back(temppoint);
	}
	delete oct;
	center.clear();
	center.swap(center);
	free(xtest);
	xtest = NULL;
	free(xyz_in);
	xyz_in = NULL;
	return true;
}

bool CTLSPointCloudTreeSegmentation::TlsPointNormalization(double gridsize)
{
	auto result1 = minmax_element(ThinPoint.begin(), ThinPoint.end(), [](PointT a, PointT b) { return a.x > b.x;  });
	auto result2 = minmax_element(ThinPoint.begin(), ThinPoint.end(), [](PointT a, PointT b) { return a.y > b.y;  });
	auto result3 = minmax_element(ThinPoint.begin(), ThinPoint.end(), [](PointT a, PointT b) { return a.z > b.z;  });
	unsigned int x_max_idx = result1.first - ThinPoint.begin();
	unsigned int x_min_idx = result1.second - ThinPoint.begin();
	unsigned int y_max_idx = result2.first - ThinPoint.begin();
	unsigned int y_min_idx = result2.second - ThinPoint.begin();
	unsigned int z_max_idx = result3.first - ThinPoint.begin();
	unsigned int z_min_idx = result3.second - ThinPoint.begin();

	//Need to add translation operation in the Z direction
	XMax = ThinPoint[x_max_idx].x;
	XMin = ThinPoint[x_min_idx].x;
	YMax = ThinPoint[y_max_idx].y;
	YMin = ThinPoint[y_min_idx].y;
	ZMax = ThinPoint[z_max_idx].z;
	ZMin = ThinPoint[z_min_idx].z;   

	//for (int i=0;i<ThinPoint.size();i++)
	//{
	//	ThinPoint[i].x -= XMin;
	//	ThinPoint[i].y -= YMin;
	//	ThinPoint[i].z -= ZMin;
	//}
	//XMax -= XMin;
	//XMin -= YMin;
	//YMax -= YMin;
	//YMin -= YMin;
	//ZMax -= ZMin;
	//ZMin -= ZMin;
	//double gridsize = 1;
	//if (progress != NULL)
	//{
	//	bool bIsCancel = progress->SetPosition(0.6);
	//	progress->SetMessage("Data Normalization!...");
	//}

	int demheight = int((YMax - YMin) / gridsize) + 1;
	int demwidth = int((XMax - XMin) / gridsize) + 1;
	double *demdata = new double[demheight*demwidth];
	if (demdata == NULL)
		return false;
	for (int i = 0; i<demheight*demwidth; i++)
		demdata[i] = DBL_MIN;
	//omp_set_num_threads(1);
	double start = omp_get_wtime();
	//#pragma omp parallel for
	for (int i = 0; i<ThinPoint.size(); i++)
	{
		int column = int((ThinPoint[i].x - XMin) / gridsize);
		int row = int((ThinPoint[i].y - YMin) / gridsize);
		if (abs(demdata[row*demwidth + column] - DBL_MIN) < 0.000001)
		{
			demdata[row*demwidth + column] = ThinPoint[i].z;
		}
		if (demdata[row*demwidth + column]>ThinPoint[i].z)   //Take the lowest value
			demdata[row*demwidth + column] = ThinPoint[i].z;

		//if (progress != NULL)
		//{
		//	//ii,jj,m_cellsX,m_cellsX,take 80%
		//	double tempposition = 0.6+(i/ThinPoint.size()*0.05);
		//	bool bIsCancel = progress->SetPosition(tempposition);
		//	progress->SetMessage("Data Normalization!...");
		//	if (!bIsCancel)
		//	{
		//		progress->SetMessage("Data Normalization Cancel! Exiting...");
		//		return false;
		//	}
		//}
	}
	//Export DEM
	/*std::ofstream demfile;
	demfile.precision (3);
	demfile.setf(std::ios::fixed);
	demfile.setf(std::ios::showpoint);
	demfile.open(m_terrainfile.c_str());
	demfile<<"XMin,"<<XMin<<",XMax,"<<XMax<<",YMin,"<<YMin<<",YMax,"<<YMax<<"\n"<<std::flush;
	demfile<<"GridSize,"<<gridsize<<",DEMHeight,"<<demheight<<",DEMWidth,"<<demwidth<<"\n"<<std::flush;
	for(int i=0;i<demheight*demwidth;i++)
	{
	if((i%(demwidth)==0) && (i !=0))
	demfile<<"\n"<<std::flush;
	if((i+1)%demwidth == 0)
	demfile<<demdata[i]<<std::flush;
	else
	demfile<<demdata[i]<<","<<std::flush;
	}
	demfile.close();*/
	//#pragma omp parallel for
	for (int i = 0; i<ThinPoint.size(); i++)
	{
		int column = int((ThinPoint[i].x - XMin) / gridsize);
		int row = int((ThinPoint[i].y - YMin) / gridsize);
		ThinPoint[i].z = ThinPoint[i].z - demdata[row*demwidth + column];

		//if (progress != NULL)
		//{
		//	//ii,jj,m_cellsX,m_cellsX,take 80%
		//	double tempposition = 0.65+i/ThinPoint.size()*0.05;
		//	bool bIsCancel = progress->SetPosition(tempposition);
		//	progress->SetMessage("Data Normalization!...");
		//	if (!bIsCancel)
		//	{
		//		progress->SetMessage("Data Normalization Cancel! Exiting...");
		//		return false;
		//	}
		//}
	}
	delete demdata;
	demdata = NULL;
	return true;
}

bool CTLSPointCloudTreeSegmentation::TlsTrunkSearch(double trunkheight, double buffer)
{
	//if (progress != NULL)
	//{
	//	bool bIsCancel = progress->SetPosition(0.7);
	//	progress->SetMessage("Tree Segmentation!...");
	//}
	//double trunkheight = 5.0;
	//double buffer = 0.3;
	NWUClustering::ClusteringAlgo dbs;
	dbs.set_dbscan_params(0.2, 5);
	dbs.m_pts = new NWUClustering::Points;
	//Trunk buffer output
	//std::ofstream trunkbuffercsvfile;
	//trunkbuffercsvfile.precision (3);
	//trunkbuffercsvfile.setf(std::ios::fixed);
	//trunkbuffercsvfile.setf(std::ios::showpoint);
	//trunkbuffercsvfile.open("D:\\indextest\\trunkbufferliforest.csv");
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	int trunkbufferpointnumber = 0;
	for (int i = 0; i<ThinPoint.size(); i++)
	{
		if ((ThinPoint[i].z >= trunkheight) && (ThinPoint[i].z <= (trunkheight + buffer)))
			trunkbufferpointnumber++;

		//if (progress != NULL)
		//{
		//	//ii,jj,m_cellsX,m_cellsX,take 80%
		//	double tempposition = 0.7+i/ThinPoint.size()*0.05;
		//	bool bIsCancel = progress->SetPosition(tempposition);
		//	progress->SetMessage("Tree Segmentation!...");
		//	if (!bIsCancel)
		//	{
		//		progress->SetMessage("Tree Segmentation Cancel! Exiting...");
		//		return false;
		//	}
		//}
	}
	dbs.m_pts->m_i_dims = 3;
	dbs.m_pts->m_i_num_points = trunkbufferpointnumber;
	dbs.m_pts->m_points.resize(trunkbufferpointnumber);
	for (int i = 0; i < trunkbufferpointnumber; i++)
		dbs.m_pts->m_points[i].resize(3);
	int k = 0;
	for (int i = 0; i<ThinPoint.size(); i++)
	{
		if ((ThinPoint[i].z >= trunkheight) && (ThinPoint[i].z <= (trunkheight + buffer)))
		{
			dbs.m_pts->m_points[k][0] = ThinPoint[i].x;
			dbs.m_pts->m_points[k][1] = ThinPoint[i].y;
			dbs.m_pts->m_points[k][2] = ThinPoint[i].z;
			//dbs.m_pts->m_points.push_back(dbscanpoint);
			//trunkbuffercsvfile<<dbs.m_pts->m_points[k][0]<< "," <<dbs.m_pts->m_points[k][1]<< "," <<dbs.m_pts->m_points[k][2]<<"\n"<<std::flush;
			k++;
		}

		//if (progress != NULL)
		//{
		//	//ii,jj,m_cellsX,m_cellsX,take 80%
		//	double tempposition = 0.75+i/ThinPoint.size()*0.05;
		//	bool bIsCancel = progress->SetPosition(tempposition);
		//	progress->SetMessage("Tree Segmentation!...");
		//	if (!bIsCancel)
		//	{
		//		progress->SetMessage("Tree Segmentation Cancel! Exiting...");
		//		return false;
		//	}
		//}
	}
	//trunkbuffercsvfile.close();
	//cout << "Write Trunk Buffer Point File " << omp_get_wtime() - start << " seconds." << endl;
	// build kdtree for the points
	//start = omp_get_wtime();
	dbs.build_kdtree();
	//cout << "Build kdtree took " << omp_get_wtime() - start << " seconds." << endl;
	//start = omp_get_wtime();
	//run_dbscan_algo(dbs);
	run_dbscan_algo_uf(dbs);
	//cout << "DBSCAN (total) took " << omp_get_wtime() - start << " seconds." << endl;
	//std::ofstream csvfile;
	//csvfile.precision (3);
	//csvfile.setf(std::ios::fixed);
	//csvfile.setf(std::ios::showpoint);
	//csvfile.open("D:\\indextest\\trunkcenter.csv");
	//std::vector<NWUClustering::XYZ>treeseed;
	dbs.writeClusters_uf(TreeSeed);
	//csvfile.close();
	return true;
}
bool CTLSPointCloudTreeSegmentation::TlsTreeHeightCalculate()
{
	if (TreeSeed.empty())
	{
		return false;
	}
	/*if (progress != NULL)
	{
	bool bIsCancel = progress->SetPosition(0.8);
	progress->SetMessage("Calculate Tree Height!...");
	}*/
	for (int i = 0; i<ThinPoint.size(); i++)
	{
		double distance = 0;
		int id = 0;
		for (int j = 0; j<TreeSeed.size(); j++)
		{
			double distancetemp = (ThinPoint[i].x - TreeSeed[j].x)*(ThinPoint[i].x - TreeSeed[j].x) + (ThinPoint[i].y - TreeSeed[j].y)*(ThinPoint[i].y - TreeSeed[j].y);
			if (j == 0)
				distance = distancetemp;
			if (distancetemp < distance) //modify
			{
				distance = distancetemp;
				id = j;
			}
		}
		ThinPoint[i].treeID = id;

		//if (progress != NULL)
		//{
		//	//ii,jj,m_cellsX,m_cellsX,ȡ80%
		//	double tempposition = 0.80+i/ThinPoint.size()*0.05;
		//	bool bIsCancel = progress->SetPosition(tempposition);
		//	progress->SetMessage("Calculate Tree Height!...");
		//	if (!bIsCancel)
		//	{
		//		progress->SetMessage("Calculate Tree Height Cancel! Exiting...");
		//		return false;
		//	}
		//}
	}
	for (int i = 0; i<ThinPoint.size(); i++)
	{
		int j = ThinPoint[i].treeID;
		if (TreeSeed[j].z<ThinPoint[i].z)
			TreeSeed[j].z = ThinPoint[i].z;

		//if (progress != NULL)
		//{
		//	//ii,jj,m_cellsX,m_cellsX,take 80%
		//	double tempposition = 0.85+i/ThinPoint.size()*0.05;
		//	bool bIsCancel = progress->SetPosition(tempposition);
		//	progress->SetMessage("Calculate Tree Height!...");
		//	if (!bIsCancel)
		//	{
		//		progress->SetMessage("Calculate Tree Height Cancel! Exiting...");
		//		return false;
		//	}
		//}
	}
	//Add output for thinned segmentation results
	ofstream o_file("liforestsegmentationthin.csv");
	for (int k = 0; k<ThinPoint.size(); k++)
	{
		o_file << ThinPoint[k].x << ", " << ThinPoint[k].y << ", " << ThinPoint[k].z << ", " << ThinPoint[k].treeID << endl;
	}
	return true;
}

bool CTLSPointCloudTreeSegmentation::TlsDBHCalculate(double dbh_height, double dbh_buffer, ostream& o, double xoffset, double yoffset)
{
	//if (progress != NULL)
	//{
	//	bool bIsCancel = progress->SetPosition(0.9);
	//	progress->SetMessage("Calculate DBH!...");
	//}
	sort(ThinPoint.begin(), ThinPoint.end(), sortidz);
	//double dbh_height = 5;
	//double dbh_buffer = 0.3;
	/*	std::ofstream dbhfile;
	dbhfile.precision (3);
	dbhfile.setf(std::ios::fixed);
	dbhfile.setf(std::ios::showpoint);
	dbhfile.open("D:\\indextest\\dbh.csv")*/;

	//std::ofstream dbhdatafile;
	//dbhdatafile.precision (3);
	//dbhdatafile.setf(std::ios::fixed);
	//dbhdatafile.setf(std::ios::showpoint);
	//dbhdatafile.open("D:\\indextest\\dbhdata.csv");
	for (int j = 0; j<TreeSeed.size(); j++)
	{
		int k1 = std::distance(ThinPoint.begin(), find_if(ThinPoint.begin(), ThinPoint.end(), FindSegID(j)));
		int k2 = 0;
		if (j == TreeSeed.size() - 1)
			k2 = TreeSeed.size();
		else
			k2 = std::distance(ThinPoint.begin(), find_if(ThinPoint.begin(), ThinPoint.end(), FindSegID(j + 1)));


		std::vector<double>X;
		std::vector<double>Y;
		for (int i = k1; i<k2; i++)
		{
			if (ThinPoint[i].z >= dbh_height && ThinPoint[i].z <= dbh_height + dbh_buffer)
			{
				X.push_back(ThinPoint[i].x);
				Y.push_back(ThinPoint[i].y);
				//dbhdatafile<<ThinPoint[i].X<< "," <<ThinPoint[i].Y<< "," <<ThinPoint[i].Z<< "," <<ThinPoint[i].id<<"\n"<<std::flush;
			}
		}
		Data dbhdata(X, Y);
		X.clear();
		Y.clear();
		std::vector<double>(X).swap(X);
		std::vector<double>(Y).swap(Y);
		//dbhdata.n = X.size();
		Circle dbhcircle = CircleFitByTaubin(dbhdata);
		//circle.a - the X-coordinate of the center of the fitting circle
		//   circle.b - the Y-coordinate of the center of the fitting circle
		//    circle.r - the radius of the fitting circle
		//    circle.s - the root mean square error (the estimate of sigma)
		//    circle.j - the total number of iterations
		if (2 * dbhcircle.r >12)
			continue;
		if (abs(dbhcircle.r)<0.05)
			continue;
		o << j + 1 << "," << TreeSeed[j].x + xoffset << "," << TreeSeed[j].y + yoffset << "," << TreeSeed[j].z << "," << dbhcircle.r * 2 << "\n" << std::flush;
		//o<<dbhcircle.a<< "," <<dbhcircle.b<< "," <<TreeSeed[j].Z<< ","<<dbhcircle.r<< ","<<dbhcircle.s<<"\n"<<std::flush;
		//if (progress != NULL)
		//{
		//	//ii,jj,m_cellsX,m_cellsX,take 80%
		//	double tempposition = 0.90+j/TreeSeed.size()*0.1;
		//	bool bIsCancel = progress->SetPosition(tempposition);
		//	progress->SetMessage("Calculate DBH!...");
		//	if (!bIsCancel)
		//	{
		//		progress->SetMessage("Calculate DBH Cancel! Exiting...");
		//		return false;
		//	}
		//}
	}
	/*if (progress != NULL)
	{
	bool bIsCancel = progress->SetPosition(1);
	progress->SetMessage("Finished!...");
	}*/
	//dbhdatafile.close();
	return true;
}

bool CTLSPointCloudTreeSegmentation::SegmentationOriginTree(PtCloud& pInCloud)
{
	for (int i = 0; i<pInCloud.size(); i++)
	{
		double distance = 0;
		int id = 0;
		for (int j = 0; j<TreeSeed.size(); j++)
		{
			double distancetemp = (pInCloud[i].x - TreeSeed[j].x)*(pInCloud[i].x - TreeSeed[j].x) + (pInCloud[i].y - TreeSeed[j].y)*(pInCloud[i].y - TreeSeed[j].y);
			if (j == 0)
				distance = distancetemp;
			if (distancetemp < distance) //modify
			{
				distance = distancetemp;
				id = j;
			}
		}
		pInCloud[i].treeID = id + 1;
	}
	return true;
}

//Export the results to a CSV file
//@std::vector<st_treeAtt> &trees Input of individual tree information
//@const std::string& csvPath An output path must be provided
bool CTLSPointCloudTreeSegmentation::outputToCSV(const std::string& csvPath)
{
	std::ofstream csvfile;
	csvfile.precision(3);
	csvfile.setf(std::ios::fixed);
	csvfile.setf(std::ios::showpoint);
	csvfile.open(csvPath);
	//header
	csvfile << "TreeID,TreeLocationX,TreeLocationY,TreeHeight\n" << std::flush;
	for (int i = 0; i < TreeSeed.size(); i++) {
		csvfile << TreeSeed[i].treeID << "," << TreeSeed[i].x << "," << TreeSeed[i].y << "," << TreeSeed[i].z << "\n";
	}
	csvfile << std::flush;
	csvfile.close();
	return false;
}
