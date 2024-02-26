//registration_no_GUI.cpp : Defines the entry point for the console application.

#include "stdafx.h"
#include "MyStruct.h"
#include "Registration.h"
using namespace std;

#pragma region Read data
//Read las data (file path, output result)
bool Readlas(string path, lidardata &data)   
{
	//Read las data
	std::ifstream ifs;
	ifs.open(path.data(), std::ios::in | std::ios::binary);
	if (!ifs)
	{
		cout << "Invalid point cloud." << endl;
		return false;
	}

	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);
	liblas::Header const& header = reader.GetHeader();

	//Point cloud attributes
	data.prop.num = header.GetPointRecordsCount();
	data.prop.X_Offset = header.GetOffsetX();
	data.prop.Y_Offset = header.GetOffsetY();
	data.prop.Z_Offset = header.GetOffsetZ();
	data.prop.Max_X = header.GetMaxX();
	data.prop.Max_Y = header.GetMaxY();
	data.prop.Max_Z = header.GetMaxZ();
	data.prop.Min_X = header.GetMinX();
	data.prop.Min_Y = header.GetMinY();
	data.prop.Min_Z = header.GetMinZ();

	//Point cloud xyz values - offset
	data.x.reserve(data.prop.num);
	data.y.reserve(data.prop.num);
	data.z.reserve(data.prop.num);

	int i = 0;
	while (reader.ReadNextPoint())
	{
		liblas::Point const&p = reader.GetPoint();
		data.x.push_back(p.GetX()- data.prop.X_Offset);
		data.y.push_back(p.GetY()- data.prop.Y_Offset);
		data.z.push_back(p.GetZ()- data.prop.Z_Offset);
		++i;
	}

}

//Read tree location data (file path, output result)
bool ReadTreeLocation(string path, treeLocation &tree)
{
	FILE* fp;
	fopen_s(&fp, path.data(), "r");
	double tmp_point[3];
	char temp[1024];
	int tmp;
	while (!feof(fp))
	{
		if (NULL == fgets(temp, 1024, fp))
		{
			continue;
		}
		if (temp[0] != '\n')
		{
			tree.num++;
		}

	}
	fclose(fp);
	fopen_s(&fp, path.data(), "r");

	for (int i = 0; i < tree.num; ++i)
	{

		fscanf_s(fp, "%d %lf %lf %lf \n", &tmp, &(tmp_point[0]), &(tmp_point[1]), &(tmp_point[2]));
		tree.x.push_back(tmp_point[0]);
		tree.y.push_back(tmp_point[1]);
		tree.z.push_back(tmp_point[2]);
	}

	return true;
}

#pragma endregion


int main()
{
	////Input path
	//string UAV_Lidar_Path = "F:/code data/UAV.las";
	//string Backpack_Lidar_Path = "F:/code data/Backpack.las";
	//string UAV_Tree_Path = "F:/code data/U_tree.txt";
	//string Backpack_Tree_Path = "F:/code data/B_tree.txt";
	//
	////Output path
	//string Dir_path = "F:/code data";


	//Data
	lidardata UAV_Data;
	lidardata Backpack_Data;
	treeLocation UAV_Tree;
	treeLocation Backpack_Tree;

	//Read data
	Readlas(UAV_Lidar_Path, UAV_Data);
	Readlas(Backpack_Lidar_Path, Backpack_Data);
	ReadTreeLocation(UAV_Tree_Path, UAV_Tree);
	ReadTreeLocation(Backpack_Tree_Path, Backpack_Tree);
	for (int i=0;i<UAV_Tree.num;++i)
	{
		UAV_Tree.x[i] -= UAV_Data.prop.X_Offset;
		UAV_Tree.y[i] -= UAV_Data.prop.Y_Offset;
		UAV_Tree.z[i] -= UAV_Data.prop.Z_Offset;
	}
	for (int i = 0; i < Backpack_Tree.num; ++i)
	{
		Backpack_Tree.x[i] -= Backpack_Data.prop.X_Offset;
		Backpack_Tree.y[i] -= Backpack_Data.prop.Y_Offset;
		Backpack_Tree.z[i] -= Backpack_Data.prop.Z_Offset;
	}

	Registration re;
	//Step1 Generate location image map, establish tree location features
	re.GenerateFeature(UAV_Data, Backpack_Data,UAV_Tree, Backpack_Tree);
	//Step2 TIN matching
	re.TINMatch(Dir_path,UAV_Data,Backpack_Data,UAV_Tree,Backpack_Tree);
    return 0;
}

