#include "PointCloudStruct.h"

namespace PointCloudStructUtil
{
	//Supplement additionally the conversion function
	void lidardata_to_MYPoint(const lidardata& data1, std::vector<MYPoint>& pts)
	{
		pts.resize(data1.x.size());
		for (int i = 0; i < data1.x.size(); i++) {
			pts[i].x = data1.x[i];
			pts[i].y = data1.y[i];
			pts[i].z = data1.z[i];
		}
	}
}