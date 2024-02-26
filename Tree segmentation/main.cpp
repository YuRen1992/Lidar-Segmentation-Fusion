#include "PointCloudTreeSegmentation.h"
#include "PointCloudTreeSegmentationTLS.h"

#include <algorithm>
//Individual tree segmentation of UAV/airborne
void runClassicALSSegment()
{
	lidardata data1;
	PointCloudTreeSegmentation::PtCloud ptVec;
	PointCloudStructUtil::lidardata_to_MYPoint(data1, ptVec);

	double xMinimum = DBL_MAX;
	double xMaximum = -DBL_MAX;
	double yMinimum = DBL_MAX;
	double yMaximum = -DBL_MAX;
	for (int i = 0; i < ptVec.size(); i++) {
		xMinimum = std::min(ptVec[i].x, xMinimum);
		xMaximum = std::max(ptVec[i].x, xMaximum);

		yMinimum = std::min(ptVec[i].y, yMinimum);
		yMaximum = std::max(ptVec[i].y, yMaximum);
	}
	double xminimum;
	//run individual tree segmentation
	PointCloudTreeSegmentation pcs_segmentation;
	pcs_segmentation.SetPara(2.0);
	pcs_segmentation.SetCurBox(xMinimum, xMaximum, yMinimum, yMaximum);
	pcs_segmentation.setHeightAbouveGround(2.0);
	pcs_segmentation.SetMinTreeHeight(1.0);
	std::vector<PointCloudTreeSegmentation::st_treeAtt> trees;
	unsigned int treeid = 0;
	if (pcs_segmentation.RunSegmentation(ptVec, trees)) {
		//Export csv
		std::string myFilePath = "yourName.csv";
		pcs_segmentation.outputToCSV(trees, myFilePath);
	}

}

//Individual tree segmentation of backpack/TLS
void runClassicTLSSegment()
{
	lidardata data1;
	PointCloudTreeSegmentation::PtCloud ptVec;
	PointCloudStructUtil::lidardata_to_MYPoint(data1, ptVec);

	CTLSPointCloudTreeSegmentation::CTLSPointCloudTreeSegmentationPara myPara;
	CTLSPointCloudTreeSegmentation pcoTreeSegmentation(myPara);
	pcoTreeSegmentation.doSegment(ptVec);

	//Export csv
	std::string myFilePath = "yourName.csv";
	pcoTreeSegmentation.outputToCSV(myFilePath);
}
