
# Lidar-Segmentation-Fusion

<font size="4">
Here we provide a workflow, including individual tree segmentation of Unmanned Aerial Vehicle (UAV)/airborne data and backpack/Terrestrial Laser Scanning (TLS) data, and a multi-platform data fusion method based on tree locations. These algorithms were previously developed by us and implemented in C++. For details on the algorithms for each section, please refer to the references at the end of the readme.
</font>

---

## Individual tree segmentation of UAV and backpack lidar data 


### 1. Individual tree segmentation of UAV lidar data

Within the `tree segmentation`folder, `PointCloudTreeSegmentationUAV.h`serves as the interface for the individual tree segmentation algorithm based on UAV lidar data `(with comments containing examples)`. An example of how to call it  is as following:

```cpp
 //Example Invocation  (it is recommended to thin the point cloud first).

 PointCloudTreeSegmentation::PtCloud ptVec(filterClouds.size());
//Individual tree segmentation
 PointCloudTreeSegmentation pcs_segmentation;
 pcs_segmentation.SetPara(2.0);
 pcs_segmentation.SetCurBox(worldbox.xMinimum(), worldbox.xMaximum(), worldbox.yMinimum(), worldbox.yMaximum());
 pcs_segmentation.setHeightAbouveGround(m_para->HeightAbouveGround);
 pcs_segmentation.SetMinTreeHeight(m_para->MinTreeHeight);

 std::vector<PointCloudTreeSegmentation::st_treeAtt>trees;
 unsigned int treeid = 0;
 if (pcs_segmentation.RunSegmentation(ptVec, trees)) {
	 ······
 }

 //Export csv
 std::string myFilePath = "yourname.csv";
 pcs_segmentation.outputToCSV(trees, myFilePath); 
```

### 2. Individual tree segmentation of backpack lidar data

Within the `tree segmentation` folder, `PointCloudTreeSegmentationBackpack.h` serves as the interface for the individual tree segmentation algorithm based on backpack lidar data `(with comments containing examples)`. An example of how to call it is as following:

```cpp
//Example Invocation 
//Assign a value to ptVec
PointCloudTreeSegmentation::PtCloud ptVec(IOPointCloud.size());

//Set parameters and run
CTLSPointCloudTreeSegmentation::CTLSPointCloudTreeSegmentationPara myPara;
CTLSPointCloudTreeSegmentation pcoTreeSegmentation(myPara);
pcoTreeSegmentation.doSegment(ptVec);

//Export csv
std::string myFilePath = cloudObject0->getFullPath();
liBase::renameExt(myFilePath, std::string(".csv"));
pcoTreeSegmentation.outputToCSV(myFilePath);

```


* * * 

## Fusion of multiplatform lidar data based on tree locations

Based on the individual tree segmentation results from both UAV and backpack lidar, data fusion is performed based on tree locations, with the main program being `\registration\registration_no_GUI.cpp`. This program contains detailed code comments, allowing users to adjust parameters according to their own data.
In the `Test data.zip` file, there are test datasets, including `backpack.las` and `UAV.las` for lidar data. `B_tree` and `U_tree` represent tree location data from backpack and UAV data sources, respectively.

----

## Reference


Guan, H.C., Su, Y.J., Hu, T.Y., Wang, R., Ma, Q., Yang, Q.L., Sun, X.L., Li, Y.M., Jin, S.C., Zhang, J., Liu, M., Wu, F.Y. & Guo, Q.H. (2020) A Novel Framework to Automatically Fuse 		Multiplatform LiDAR Data in Forest Environments Based on Tree Locations. IEEE Transactions on Geoscience and Remote Sensing, 58, 2165-2177.

Li, W., Guo, Q., Jakubowski, M.K. & Kelly, M. (2012) A new method for segmenting individual trees from the lidar point cloud. Photogrammetric Engineering & Remote Sensing, 78, 75-84.

Tao, S., Wu, F., Guo, Q., Wang, Y., Li, W., Xue, B., Hu, X., Li, P., Tian, D. & Li, C. (2015) Segmenting tree crowns from terrestrial and mobile LiDAR data by exploring ecological theories. ISPRS Journal of Photogrammetry and Remote Sensing, 110, 66-76.
