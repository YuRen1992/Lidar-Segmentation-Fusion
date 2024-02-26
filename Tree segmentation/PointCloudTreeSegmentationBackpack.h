/*
 //Example Call
	//Assign values to ptVec.
	PointCloudTreeSegmentation::PtCloud ptVec(IOPointCloud.size());
	
	//Set parameters and run.
	CTLSPointCloudTreeSegmentation::CTLSPointCloudTreeSegmentationPara myPara;
	CTLSPointCloudTreeSegmentation pcoTreeSegmentation(myPara);
	pcoTreeSegmentation.doSegment(ptVec);

	//Export to CSV.
	std::string myFilePath = cloudObject0->getFullPath();
	liBase::renameExt(myFilePath, std::string(".csv"));
	pcoTreeSegmentation.outputToCSV(myFilePath);

 */                                                                      
#pragma once

#include <vector>
#include "PointCloudStruct.h"

class CTLSPointCloudTreeSegmentation
{
public:
	struct CTLSPointCloudTreeSegmentationPara
	{
		//Downsampling grid size.
		double m_thinGridSize = 0.25;
		//Detect tree trunk height by inputting a relatively clean height matching the point cloud.
		double trunkheight = 5.0;
		// Detect layer height. Use the point cloud from trunkheight to trunkheight + m_buffer to detect the tree trunk.
		double m_buffer = 0.3;

	};

	typedef MYPointCTreeID PointT;
	typedef std::vector<PointT> PtCloud;

	double XMin;
	double XMax;
	double YMin;
	double YMax;
	double ZMin;
	double ZMax;
	//std::string m_terrainfile;
	PtCloud ThinPoint;
	PtCloud TreeSeed;

public:
	CTLSPointCloudTreeSegmentation(const CTLSPointCloudTreeSegmentationPara& para);
	~CTLSPointCloudTreeSegmentation();

	//Perform single-tree segmentation, requiring the input to be a normalized point cloud.
	bool doSegment(PtCloud& pInCloud);

	//Export the results to CSV.
	//@const std::string& csvPath Output path with the file extension as CSV.
	bool outputToCSV(const std::string& csvPath);

	//After running the single-tree segmentation, you can obtain seed points.
	PtCloud getSeed() {return TreeSeed;}

private:
	//1
	bool TlsPointThin(PtCloud& pInCloud, double ThinGridSize);
	//no
	bool TlsPointNormalization(double gridsize);
	//2
	bool TlsTrunkSearch(double trunkheight, double buffer);
	//3
	bool TlsTreeHeightCalculate();
	//no
	bool TlsDBHCalculate(double dbh_height, double dbh_buffer, std::ostream& o, double xoffset, double yoffset);
	//4
	bool SegmentationOriginTree(PtCloud& pInCloud);

	CTLSPointCloudTreeSegmentationPara m_para;
};