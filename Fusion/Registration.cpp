#include "stdafx.h"
#include "Registration.h"
#include<windows.h>
bool Registration::BiggerAzimuth(singleFeature feature1, singleFeature feature2)
{
	return(feature1.azimuth < feature2.azimuth);
}

bool Registration::SmallerScore(feature_score a, feature_score b)
{
	return(a.score > b.score);
}

Registration::Registration()
{
}

//Create tree location image (input point cloud, number of input positions, tree location image, tree location image coordinates x, tree location image coordinates y)
bool Registration::GenerateTreeLoactionMap(lidardata &data, treeLocation &tree, cv::Mat& image, std::vector<int> &tree_x, std::vector<int> &tree_y)
{
	//Generate the raster image based on the point cloud range and tree locations
	double xmax = data.prop.Max_X - data.prop.X_Offset;
	double xmin = data.prop.Min_X - data.prop.X_Offset;
	double ymax = data.prop.Max_Y - data.prop.Y_Offset;
	double ymin = data.prop.Min_Y - data.prop.Y_Offset;

	//Set the image resolution to 0.1m*0.1m
	float pixelWidth = 0.1;
	float pixelHeight = 0.1;

	int nrow = static_cast<int>((ymax - ymin) / pixelHeight);
	int ncol = static_cast<int>((xmax - xmin) / pixelWidth);

	//Create an empty image
	cv::Mat blockimage(nrow, ncol, CV_8UC3, cv::Scalar(255, 255, 255));
	image = blockimage.clone();

	//Calculate the position of trees in the grid
	int posx = 0;
	int posy = 0;
	//The pixel coordinates corresponding to each tree point

	for (int i = 0; i < tree.num; ++i)
	{

		posx = static_cast<int>((tree.y[i] - ymin) / pixelHeight);
		posy = static_cast<int>((tree.x[i] - xmin) / pixelWidth);
		image.at<cv::Vec3b>(posx, posy)[0] = 255;
		image.at<cv::Vec3b>(posx, posy)[1] = 0;
		image.at<cv::Vec3b>(posx, posy)[2] = 0;
		tree_x.push_back(posx);
		tree_y.push_back(posy);
		char text[10];
		_itoa((i + 1), text, 10);
		cv::Point orign;
		orign.x = posy;
		orign.y = posx;
		cv::putText(image, text, orign, cv::FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(0, 0, 255), 0.4, 8, 0);
	}

	return true;
}

//Calculate the azimuth angle
float Registration::CalAzimuth(float xa, float ya, float xb, float yb)
{
	float dx = xb - xa;
	float dy = yb - ya;
	if (dy == 0.0)
	{
		if (dx == 0)
		{
			return 0;
		}
		if (dx > 0)
		{
			return CV_PI / 2;
		}
		if (dx < 0)
		{
			return CV_PI / 2 * 3;
		}
	}
	else
	{
		float Az = CV_PI - atan(static_cast<double>(dx) / dy) - abs(dy) / dy*CV_PI / 2;
		return Az;
	}

}


//Nearest neighbor search, neighboring points grouped together (tree image, tree location image coordinate x, tree location image coordinate y, features)
bool Registration::FindAdjacentTree(treeLocation &tree, std::vector<int> &tree_x, std::vector<int> &tree_y, std::vector<treeFeature>& feature)
{

	//Generate kd_tree index for tree locations
	pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>);
	cloud->width = tree.num;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	for (int i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = tree.x[i];
		cloud->points[i].y = tree.y[i];
	}

	pcl::search::KdTree<pcl::PointXY>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXY>);
	kdtree->setInputCloud(cloud);

	//kd_tree search
	pcl::PointXY searchPoint;
	int k = 9;//Search for the nearest 8 points
	std::vector<int> pointIdxNKNSearch(k);
	std::vector<float> pointNKNSquaredDistance(k);


	for (int i = 0; i < tree.num; ++i)
	{
		searchPoint.x = tree.x[i];
		searchPoint.y = tree.y[i];
		if (kdtree->nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			treeFeature tmpFeature;
			singleFeature tmpsingle;
			for (int i = 1; i < pointIdxNKNSearch.size(); ++i)  //Calculate the features of the search point and its 8 nearest neighbors
			{
				tmpsingle.num = pointIdxNKNSearch[i];
				tmpsingle.x = tree_x[pointIdxNKNSearch[i]];
				tmpsingle.y = tree_y[pointIdxNKNSearch[i]];
				tmpsingle.x_true = tree.x[pointIdxNKNSearch[i]];
				tmpsingle.y_true = tree.y[pointIdxNKNSearch[i]];
				float az = CalAzimuth(searchPoint.x, searchPoint.y, tree.x[pointIdxNKNSearch[i]], tree.y[pointIdxNKNSearch[i]]);
				tmpsingle.azimuth = az;
				tmpsingle.dis2 = pointNKNSquaredDistance[i];
				tmpFeature.features.push_back(tmpsingle);
				tmpFeature.num++;
			}
			feature.push_back(tmpFeature);
		}
	}
	return true;
}

//Construct features (UAV point cloud, Backpack point cloud, UAV tree position, Backpack tree position)
bool Registration::GenerateFeature(lidardata &UAV_Data,lidardata &Backpack_Data,treeLocation &UAV_Tree, treeLocation &Backpack_Tree)
{
	cv::Mat f1;
	cv::Mat f2;

	//Generate tree location images
	GenerateTreeLoactionMap(UAV_Data, UAV_Tree, img1, tree1_x, tree1_y);
	GenerateTreeLoactionMap(Backpack_Data, Backpack_Tree, img2, tree2_x, tree2_y);
	imwrite("tree locations in UAVlidar.tif", img1);
	imwrite("tree locations in backpack lidar.tif", img2);

	//Generate features
	FindAdjacentTree(UAV_Tree, tree1_x, tree1_y, tree1_feature);
	FindAdjacentTree(Backpack_Tree, tree2_x, tree2_y, tree2_feature);

	//Azimuth sorting
	for (int i = 0; i < tree1_feature.size(); ++i)
	{
		sort(tree1_feature[i].features.begin(), tree1_feature[i].features.end(), BiggerAzimuth);
	}
	for (int i = 0; i < tree2_feature.size(); ++i)
	{
		sort(tree2_feature[i].features.begin(), tree2_feature[i].features.end(), BiggerAzimuth);
	}

	return true;
}

//Generate TIN
std::vector<TIN> Registration::buildTIN(string name, int num, treeFeature &feature, lidardata &data, double &treex, double &treey, std::vector<TIN>& tin)
{
	GDALAllRegister();
	double range[6];
	range[0] = data.prop.Min_X;
	range[1] = data.prop.Max_X;
	range[2] = data.prop.Min_Y;
	range[3] = data.prop.Max_Y;
	range[4] = 0;
	range[5] = 0;
	BuildTIN mubuild;
	//float* point_buffer = (float*)malloc(sizeof(float) * 3 * (feature.num + 1));
	//int* index_map = (int*)malloc(sizeof(int)*(feature.num + 1));
	//mubuild.TINclean(feature.num + 1);
	//for (int i = 0; i < feature.num + 1; ++i)
	//{
	//	if (i<feature.num)
	//	{
	//		point_buffer[3 * i + 0] = static_cast<float>(feature.features[i].x_true - range[0]);
	//		point_buffer[3 * i + 1] = static_cast<float>(feature.features[i].y_true - range[2]);
	//		point_buffer[3 * i + 2] = static_cast<float>(0.0);
	//	}
	//	else
	//	{
	//		point_buffer[3 * i + 0] = static_cast<float>(treex- range[0]);
	//		point_buffer[3 * i + 1] = static_cast<float>(treey - range[2]);
	//		point_buffer[3 * i + 2] = static_cast<float>(0.0);
	//	}
	//	
	//	if (index_map) index_map[i] = i;
	//	// add the point to the triangulation
	//	mubuild.TINadd(&(point_buffer[3 * i]));
	//}
	float* point_buffer = (float*)malloc(sizeof(float) * 3 * (feature.num));
	int* index_map = (int*)malloc(sizeof(int)*(feature.num));
	mubuild.TINclean(feature.num);
	for (int i = 0; i < feature.num; ++i)
	{

		point_buffer[3 * i + 0] = static_cast<float>(feature.features[i].x_true - range[0]);
		point_buffer[3 * i + 1] = static_cast<float>(feature.features[i].y_true - range[2]);
		point_buffer[3 * i + 2] = static_cast<float>(0.0);

		if (index_map) index_map[i] = i;
		// add the point to the triangulation
		mubuild.TINadd(&(point_buffer[3 * i]));
	}
	mubuild.TINfinish();
	TIN tmp;



	TINtriangle* tt = mubuild.TINget_triangle(0);
	int tinnum = mubuild.TINget_size();
	for (int i = 0; i < tinnum; ++i, ++tt)
	{
		if (tt->next < 0) // if not deleted
		{
			if (tt->V[0]) // if not infinite
			{
				tmp.x1 = tt->V[0][0] + range[0];
				tmp.y1 = tt->V[0][1] + range[2];
				tmp.z1 = tt->V[0][2] + range[4];
				tmp.x2 = tt->V[1][0] + range[0];
				tmp.y2 = tt->V[1][1] + range[2];
				tmp.z2 = tt->V[1][2] + range[4];
				tmp.x3 = tt->V[2][0] + range[0];
				tmp.y3 = tt->V[2][1] + range[2];
				tmp.z3 = tt->V[2][2] + range[4];
				//Calculate  lengths
				float a;
				float b;
				float c;
				float t;
				a = sqrt((tmp.x1 - tmp.x2)*(tmp.x1 - tmp.x2) + (tmp.y1 - tmp.y2)*(tmp.y1 - tmp.y2));
				b = sqrt((tmp.x1 - tmp.x3)*(tmp.x1 - tmp.x3) + (tmp.y1 - tmp.y3)*(tmp.y1 - tmp.y3));
				c = sqrt((tmp.x3 - tmp.x2)*(tmp.x3 - tmp.x2) + (tmp.y3 - tmp.y2)*(tmp.y3 - tmp.y2));

				//Length sorting
				if (a > b)
				{
					t = a;
					a = b;
					b = t;
				}
				if (a > c)
				{
					t = a;
					a = c;
					c = t;
				}
				if (b > c)
				{
					t = b;
					b = c;
					c = t;
				}

				tmp.area = (a + b + c) / 2;
				tmp.max_angle = (a*a + b*b - c*c) / (2 * a*b);
				tmp.max_angle = acos(tmp.max_angle);
				tmp.alive = 0;
				tin.push_back(tmp);
			}
		}
	}

	
	//Output if the folder exists, otherwise bug
	//string left_tin_path = name + "/" + std::to_string((num + 1)) + ".shp";

	//FILE* file_out = NULL;


	//file_out = fopen(left_tin_path.data(), "wb");

	/////////////////////////
	//int nfaces = tin.size();

	//int content_length = (76 + (nfaces * 104)) / 2;
	//// write header

	//int int_output;
	//int_output = 9994; mubuild.to_big_endian(&int_output);
	//fwrite(&int_output, sizeof(int), 1, file_out); // file code (BIG)
	//int_output = 0; mubuild.to_big_endian(&int_output);
	//fwrite(&int_output, sizeof(int), 1, file_out); // unused (BIG)
	//fwrite(&int_output, sizeof(int), 1, file_out); // unused (BIG)
	//fwrite(&int_output, sizeof(int), 1, file_out); // unused (BIG)
	//fwrite(&int_output, sizeof(int), 1, file_out); // unused (BIG)
	//fwrite(&int_output, sizeof(int), 1, file_out); // unused (BIG)
	//int_output = 50 + 4 + content_length; mubuild.to_big_endian(&int_output);
	//fwrite(&int_output, sizeof(int), 1, file_out); // file length (BIG)
	//int_output = 1000; mubuild.to_little_endian(&int_output);
	//fwrite(&int_output, sizeof(int), 1, file_out); // version (LITTLE)
	//int_output = 31; mubuild.to_little_endian(&int_output);
	//fwrite(&int_output, sizeof(int), 1, file_out); // shape type (LITTLE) = MultiPatch
	//double double_output;
	//double_output = range[0]; mubuild.to_little_endian(&double_output);
	//fwrite(&double_output, sizeof(double), 1, file_out); // xmin (LITTLE)
	//double_output = range[2]; mubuild.to_little_endian(&double_output);
	//fwrite(&double_output, sizeof(double), 1, file_out); // ymin (LITTLE)
	//double_output = range[1]; mubuild.to_little_endian(&double_output);
	//fwrite(&double_output, sizeof(double), 1, file_out); // xmax (LITTLE)
	//double_output = range[3]; mubuild.to_little_endian(&double_output);
	//fwrite(&double_output, sizeof(double), 1, file_out); // ymax (LITTLE)
	//double_output = range[4]; mubuild.to_little_endian(&double_output);
	//fwrite(&double_output, sizeof(double), 1, file_out); // zmin (LITTLE)
	//double_output = range[5]; mubuild.to_little_endian(&double_output);
	//fwrite(&double_output, sizeof(double), 1, file_out); // zmax (LITTLE)
	//double_output = 0.0; mubuild.to_little_endian(&double_output);
	//fwrite(&double_output, sizeof(double), 1, file_out); // mmin (LITTLE)
	//double_output = 0.0; mubuild.to_little_endian(&double_output);
	//fwrite(&double_output, sizeof(double), 1, file_out); // mmax (LITTLE)

	//													 // write contents

	//int_output = 1; mubuild.to_big_endian(&int_output);
	//fwrite(&int_output, sizeof(int), 1, file_out); // record number (BIG)
	//int_output = content_length; mubuild.to_big_endian(&int_output);
	//fwrite(&int_output, sizeof(int), 1, file_out); // content length (BIG)

	//int_output = 31; mubuild.to_little_endian(&int_output);
	//fwrite(&int_output, sizeof(int), 1, file_out); // shape type (LITTLE) = MultiPatch

	//double_output = range[0]; mubuild.to_little_endian(&double_output);
	//fwrite(&double_output, sizeof(double), 1, file_out); // xmin (LITTLE)
	//double_output = range[2]; mubuild.to_little_endian(&double_output);
	//fwrite(&double_output, sizeof(double), 1, file_out); // ymin (LITTLE)
	//double_output = range[1]; mubuild.to_little_endian(&double_output);
	//fwrite(&double_output, sizeof(double), 1, file_out); // xmax (LITTLE)
	//double_output = range[3]; mubuild.to_little_endian(&double_output);
	//fwrite(&double_output, sizeof(double), 1, file_out); // ymax (LITTLE)

	//int_output = nfaces; mubuild.to_little_endian(&int_output);
	//fwrite(&int_output, sizeof(int), 1, file_out); // number of parts (LITTLE)
	//int_output = nfaces * 3; mubuild.to_little_endian(&int_output);
	//fwrite(&int_output, sizeof(int), 1, file_out); // number of points (LITTLE)
	//											   // index to first point in each part (here as many parts as triangles)
	//for (int i = 0; i < nfaces; i++)
	//{
	//	int_output = 3 * i; mubuild.to_little_endian(&int_output);
	//	fwrite(&int_output, sizeof(int), 1, file_out); // (LITTLE)
	//}
	//// type of each part (here each parts is a triangle strip)
	//for (int i = 0; i < nfaces; i++)
	//{
	//	int_output = 0; mubuild.to_little_endian(&int_output);
	//	fwrite(&int_output, sizeof(int), 1, file_out); // (LITTLE)
	//}

	//// write points x and y


	//for (int count = 0; count < tin.size(); count++)
	//{

	//	double_output = tin[count].x1; mubuild.to_little_endian(&double_output);
	//	fwrite(&double_output, sizeof(double), 1, file_out); // x of point (LITTLE)
	//	double_output = tin[count].y1; mubuild.to_little_endian(&double_output);
	//	fwrite(&double_output, sizeof(double), 1, file_out); // y of point (LITTLE)

	//	double_output = tin[count].x2; mubuild.to_little_endian(&double_output);
	//	fwrite(&double_output, sizeof(double), 1, file_out); // x of point (LITTLE)
	//	double_output = tin[count].y2; mubuild.to_little_endian(&double_output);
	//	fwrite(&double_output, sizeof(double), 1, file_out); // y of point (LITTLE)

	//	double_output = tin[count].x3; mubuild.to_little_endian(&double_output);
	//	fwrite(&double_output, sizeof(double), 1, file_out); // x of point (LITTLE)
	//	double_output = tin[count].y3; mubuild.to_little_endian(&double_output);
	//	fwrite(&double_output, sizeof(double), 1, file_out); // y of point (LITTLE)
	//}

	//// write z
	//double_output = range[4]; mubuild.to_little_endian(&double_output);
	//fwrite(&double_output, sizeof(double), 1, file_out); // zmin (LITTLE)
	//double_output = range[5]; mubuild.to_little_endian(&double_output);
	//fwrite(&double_output, sizeof(double), 1, file_out); // zmax (LITTLE)

	//for (int count = 0; count < tin.size(); count++)
	//{
	//	double_output = tin[count].z1; mubuild.to_little_endian(&double_output);
	//	fwrite(&double_output, sizeof(double), 1, file_out); // z of point (LITTLE)

	//	double_output = tin[count].z2; mubuild.to_little_endian(&double_output);
	//	fwrite(&double_output, sizeof(double), 1, file_out); // z of point (LITTLE)

	//	double_output = tin[count].z3; mubuild.to_little_endian(&double_output);
	//	fwrite(&double_output, sizeof(double), 1, file_out); // z of point (LITTLE)
	//}

	//// write m

	//double_output = 0.0; mubuild.to_little_endian(&double_output);
	//fwrite(&double_output, sizeof(double), 1, file_out); // mmin (LITTLE)
	//fwrite(&double_output, sizeof(double), 1, file_out); // mmax (LITTLE)
	//for (int i = 0; i < nfaces; i++)
	//{
	//	for (int j = 0; j < 3; j++)
	//	{
	//		fwrite(&double_output, sizeof(double), 1, file_out); // m of point (LITTLE)
	//	}
	//}

	//fclose(file_out);
	mubuild.TINdestroy();
	free(point_buffer);
	return tin;
}



//TIN matching
bool Registration::TINMatch(string Dir_path, lidardata &UAV_Data, lidardata &Backpack_Data, treeLocation &UAV_Tree, treeLocation &Backpack_Tree)
{
	std::vector < all_TIN > left_tin;
	std::vector < all_TIN > right_tin;
	float th_angle = 0.2;
	float th_area = 0.2;

	left_tin.resize(tree1_feature.size());
	right_tin.resize(tree2_feature.size());
	
	
	string Backpack_TIN_path = Dir_path + "/" + "Backpack_TIN";
	CreateDirectory(Dir_path.c_str(), NULL);
	Backpack_TIN_path = Backpack_TIN_path + "/";

	string UAV_TIN_path = Dir_path + "/" + "UAV_TIN";
	CreateDirectory(Dir_path.c_str(), NULL);
	UAV_TIN_path = UAV_TIN_path + "/";

	//tmp
	UAV_TIN_path = "F:/r/1";
	Backpack_TIN_path= "F:/r/2";


	//Construct TIN
	for (int i = 0; i < tree1_feature.size(); ++i)
	{
		std::vector<TIN> tin_tmp;
		tin_tmp = buildTIN(UAV_TIN_path, i, tree1_feature[i], UAV_Data, UAV_Tree.x[i], UAV_Tree.y[i], tin_tmp);
		left_tin[i].tin = tin_tmp;
	}
	for (int i = 0; i < tree2_feature.size(); ++i)
	{
		std::vector<TIN> tin_tmp;
		tin_tmp = buildTIN(Backpack_TIN_path, i, tree2_feature[i], Backpack_Data, Backpack_Tree.x[i], Backpack_Tree.y[i], tin_tmp);
		right_tin[i].tin = tin_tmp;
	}

	//Vote scoring
	std::vector<all_score> scores;
	scores.resize(tree1_feature.size());
	for (int i = 0; i < tree1_feature.size(); ++i)
	{
		for (int j = 0; j < tree2_feature.size(); ++j)
		{
			feature_score tmp_score;
			tmp_score.score = 0;
			tmp_score.num = j;
			std::vector<bool> label;
			label.resize(right_tin[j].tin.size());
			for (int m = 0; m < left_tin[i].tin.size(); ++m)
			{
				for (int n = 0; n < right_tin[j].tin.size(); ++n)
				{
					if (label[n] == false)
					{
						if (left_tin[i].tin[m].area <= right_tin[j].tin[n].area*(1 + th_area) && left_tin[i].tin[m].area >= right_tin[j].tin[n].area*(1 - th_area))
						{
							if (left_tin[i].tin[m].max_angle <= right_tin[j].tin[n].max_angle*(1 + th_angle) && left_tin[i].tin[m].max_angle >= right_tin[j].tin[n].max_angle*(1 - th_angle))
							{
								tmp_score.score++;
								label[n] = true;
							}
						}
					}
				}
			}
			scores[i].scores.push_back(tmp_score);
		}
	}

	//Sorting
	for (int i = 0; i < tree1_feature.size(); ++i)
	{
		sort(scores[i].scores.begin(), scores[i].scores.end(), SmallerScore);
	}


	//Extract candidate points and process 
	std::vector<int> match_point;
	for (int i = 0; i < tree1_feature.size(); ++i)
	{
		std::vector<int> candidate;
		if (scores[i].scores[0].score <= 6)  //Exclusion of scores less than 6
		{
			match_point.push_back(-1);
			continue;
		}
		else
		{
			//Extract candidate point
			candidate.push_back(scores[i].scores[0].num);
			for (int j = 1; j < scores[i].scores.size(); ++j)
			{
				if (scores[i].scores[j].score == scores[i].scores[0].score)
				{
					candidate.push_back(scores[i].scores[j].num);
				}
				else
				{
					break;
				}
			}
			//Select the best point from candidates
			int loop_count = 0;
			float th_angle_new = th_angle;
			float th_area_new = th_area;
			std::vector<feature_score> candidate_scores;
			while (loop_count < 2 && candidate.size()>1)
			{
				loop_count++;
				th_angle_new = th_angle_new / 2;
				th_area_new = th_area_new / 2;
				//Recalculate scores
				for (int j = 0; j < candidate.size(); ++j)
				{
					feature_score tmp_score;
					tmp_score.score = 0;
					tmp_score.num = candidate[j];
					std::vector<bool> label;
					label.resize(right_tin[candidate[j]].tin.size());
					for (int m = 0; m < left_tin[i].tin.size(); ++m)
					{
						for (int n = 0; n < right_tin[candidate[j]].tin.size(); ++n)
						{
							if (label[n] == false)
							{
								if (left_tin[i].tin[m].area <= right_tin[candidate[j]].tin[n].area*(1 + th_area_new) && left_tin[i].tin[m].area >= right_tin[candidate[j]].tin[n].area*(1 - th_area_new))
								{
									if (left_tin[i].tin[m].max_angle <= right_tin[candidate[j]].tin[n].max_angle*(1 + th_angle_new) && left_tin[i].tin[m].max_angle >= right_tin[candidate[j]].tin[n].max_angle*(1 - th_angle_new))
									{
										tmp_score.score++;
										label[n] = true;
									}
								}
							}
						}
					}
					candidate_scores.push_back(tmp_score);
				}

				//Clear candidates
				candidate.swap(std::vector<int>());


				//resorting
				sort(candidate_scores.begin(), candidate_scores.end(), SmallerScore);

				candidate.push_back(candidate_scores[0].num);
				for (int j = 1; j < candidate_scores.size(); ++j)
				{
					if (candidate_scores[0].score == candidate_scores[j].score)
					{
						candidate.push_back(candidate_scores[j].num);
					}
				}

				//Clear candidate_scores
				candidate_scores.swap(std::vector<feature_score>());
			}
			match_point.push_back(candidate[0]);
		}

	}

	//Display matching results
	//Remove duplicate corresponding points
	std::vector<int> match_point_copy;
	std::vector<int> repeat_point;
	for (int i = 0; i < match_point.size(); ++i)
	{
		match_point_copy.push_back(match_point[i]);
	}
	sort(match_point_copy.begin(), match_point_copy.end(), std::greater<int>());
	for (int i = 0; i < match_point_copy.size() - 1; ++i)
	{
		int flag = 0;
		for (int j = i + 1; j < match_point_copy.size(); ++j)
		{
			if (match_point_copy[j] != match_point_copy[i])
			{
				break;
			}
			else
			{
				flag = 1;
				match_point_copy.erase(match_point_copy.begin() + j);
				j--;
			}
		}
		if (flag == 1)
		{
			repeat_point.push_back(match_point_copy[i]);
		}
	}

	//Set the correspondence of duplicate points to -1
	for (int i = 0; i < match_point.size(); ++i)
	{
		for (int j = 0; j < repeat_point.size(); ++j)
		{
			if (match_point[i] == repeat_point[j])
			{
				match_point[i] = -1;
			}
		}
	}
	std::vector<cv::KeyPoint> keyPoint1, keyPoint2;
	std::vector<int>keyPoint1_index, keyPoint2_index;
	std::vector<cv::DMatch> matchePoints;
	int badcount = 0;
	for (int i = 0; i < tree1_feature.size(); ++i)
	{
		if (match_point[i] == -1)
		{
			badcount++;
		}
		if (match_point[i] != -1)
		{
			cv::DMatch tmp_match;
			tmp_match.trainIdx = i - badcount;
			tmp_match.queryIdx = i - badcount;
			matchePoints.push_back(tmp_match);

			cv::KeyPoint tmp1, tmp2;
			tmp1.pt.x = tree1_y[i];
			tmp1.pt.y = tree1_x[i];
			tmp2.pt.x = tree2_y[match_point[i]];
			tmp2.pt.y = tree2_x[match_point[i]];
			keyPoint1.push_back(tmp1);
			keyPoint2.push_back(tmp2);
			keyPoint1_index.push_back(i);
			keyPoint2_index.push_back(match_point[i]);
		}

	}

	//ransac
	cv::Mat p1(keyPoint1.size(), 2, CV_64F);
	cv::Mat p2(keyPoint1.size(), 2, CV_64F);
	for (int i = 0; i < keyPoint1.size(); ++i)
	{
		//p1.at<float>(i, 0) = keyPoint1[i].pt.x;          //Pixel coordinates
		//p1.at<float>(i, 1) = keyPoint1[i].pt.y;
		//p2.at<float>(i, 0) = keyPoint2[i].pt.x;
		//p2.at<float>(i, 1) = keyPoint2[i].pt.y;
		p1.at<double>(i, 0) = UAV_Tree.x[keyPoint1_index[i]];   //Actual coordinates
		p1.at<double>(i, 1) = UAV_Tree.y[keyPoint1_index[i]];
		p2.at<double>(i, 0) = Backpack_Tree.x[keyPoint2_index[i]];
		p2.at<double>(i, 1) = Backpack_Tree.y[keyPoint2_index[i]];

	}
	std::vector<uchar>inliersMask(keyPoint1.size());
	cv::Mat MatH = findHomography(p1, p2, CV_FM_RANSAC, 3.0, inliersMask);
	std::vector<cv::DMatch>inliers;
	for (size_t i = 0; i < inliersMask.size(); i++) {
		if (inliersMask[i])
			inliers.push_back(matchePoints[i]);
	}
	matchePoints.swap(inliers);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZ>());
	cloud_left->points.resize(matchePoints.size());
	cloud_right->points.resize(matchePoints.size());
	for (int i = 0; i < matchePoints.size(); i++)
	{
		int pos1 = keyPoint1_index[matchePoints[i].queryIdx];
		int pos2 = keyPoint2_index[matchePoints[i].trainIdx];
		cloud_left->points[i].x = UAV_Tree.x[pos1];
		cloud_left->points[i].y = UAV_Tree.y[pos1];
		cloud_left->points[i].z = 1;
		cloud_right->points[i].x = Backpack_Tree.x[pos2];
		cloud_right->points[i].y = Backpack_Tree.y[pos2];
		cloud_right->points[i].z = 1;
	}

	//Solve the transformation matrix using the SVD method 
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
	TESVD.estimateRigidTransformation(*cloud_left, *cloud_right, transformation);

	//Output the result after rotating and translating the left tree
	string result2D_path = Dir_path + "/" + "2d_tree_result_UAV.txt";
	FILE *out;
	out = fopen(result2D_path.data(), "w");
	for (int i = 0; i < UAV_Tree.num; ++i)
	{
		double newx = UAV_Tree.x[i] * transformation(0, 0) + UAV_Tree.y[i] * transformation(0, 1) + transformation(0, 3);
		double newy = UAV_Tree.x[i] * transformation(1, 0) + UAV_Tree.y[i] * transformation(1, 1) + transformation(1, 3);
		fprintf(out, "%lf %lf %lf \n", newx, newy, 1.0);
	}
	fclose(out);

	//Output the comparison right tree result
	string result2D_path2 = Dir_path + "/" + "2d_tree_result_Backpack.txt";
	out = fopen(result2D_path2.data(), "w");
	for (int i = 0; i < Backpack_Tree.num; ++i)
	{
		fprintf(out, "%lf %lf %lf \n", Backpack_Tree.x[i], Backpack_Tree.y[i], 1.0);
	}
	fclose(out);


	//Elevation direction matching
	pcl::PointXY searchPoint;
	float radius = 2.0;
	//left
	pcl::PointCloud<pcl::PointXY>::Ptr cloud1(new pcl::PointCloud<pcl::PointXY>);
	cloud1->width = UAV_Data.prop.num;
	cloud1->height = 1;
	cloud1->points.resize(cloud1->width * cloud1->height);
	for (int i = 0; i < cloud1->points.size(); ++i)
	{
		cloud1->points[i].x = UAV_Data.x[i];
		cloud1->points[i].y = UAV_Data.y[i];
	}

	pcl::search::KdTree<pcl::PointXY>::Ptr kdtree1(new pcl::search::KdTree<pcl::PointXY>);
	kdtree1->setInputCloud(cloud1);
	std::vector<int> pointIdxRadiusSearch1;
	std::vector<float> pointRadiusSquaredDistance1;
	std::vector<double> data1_max_z;

	for (int i = 0; i < cloud_left->points.size(); ++i)
	{
		searchPoint.x = cloud_left->points[i].x;
		searchPoint.y = cloud_left->points[i].y;
		double max_z = -9999;
		if (kdtree1->radiusSearch(searchPoint, radius, pointIdxRadiusSearch1, pointRadiusSquaredDistance1) > 0)
		{

			for (int j = 0; j < pointIdxRadiusSearch1.size(); ++j)  
			{
				int pos = pointIdxRadiusSearch1[j];
				/*if (i == 0)
				{

					test1.x.push_back(UAV_Data.x[pos]);
					test1.y.push_back(UAV_Data.y[pos]);
					test1.z.push_back(UAV_Data.z[pos]);
				}*/
				max_z = (max_z < UAV_Data.z[pos]) ? UAV_Data.z[pos] : max_z;
			}
			data1_max_z.push_back(max_z);
		}
	}

	//right
	pcl::PointCloud<pcl::PointXY>::Ptr cloud2(new pcl::PointCloud<pcl::PointXY>);
	cloud2->width = Backpack_Data.prop.num;
	cloud2->height = 1;
	cloud2->points.resize(cloud2->width * cloud2->height);
	for (int i = 0; i < cloud2->points.size(); ++i)
	{
		cloud2->points[i].x = Backpack_Data.x[i];
		cloud2->points[i].y = Backpack_Data.y[i];
	}
	pcl::search::KdTree<pcl::PointXY>::Ptr kdtree2(new pcl::search::KdTree<pcl::PointXY>);
	kdtree2->setInputCloud(cloud2);
	std::vector<int> pointIdxRadiusSearch2;
	std::vector<float> pointRadiusSquaredDistance2;
	std::vector<double> data2_max_z;

	for (int i = 0; i < cloud_right->points.size(); ++i)
	{
		searchPoint.x = cloud_right->points[i].x;
		searchPoint.y = cloud_right->points[i].y;
		double max_z = -9999;
		if (kdtree2->radiusSearch(searchPoint, radius, pointIdxRadiusSearch2, pointRadiusSquaredDistance2) > 0)
		{

			for (int j = 0; j < pointIdxRadiusSearch2.size(); ++j)  //Calculate the features of the search point and its 8 nearest neighbors
			{
				int pos = pointIdxRadiusSearch2[j];
				max_z = (max_z < Backpack_Data.z[pos]) ? Backpack_Data.z[pos] : max_z;
				/*if (i == 0)
				{

					test2.x.push_back(Backpack_Data.x[pos]);
					test2.y.push_back(Backpack_Data.y[pos]);
					test2.z.push_back(Backpack_Data.z[pos]);
				}*/
			}
			data2_max_z.push_back(max_z);
		}
	}

	//Calculate the difference
	std::vector<double> dx;
	double dxall = 0;
	double dx_avg = 0;
	double error = 0;
	for (int i = 0; i < data1_max_z.size(); ++i)
	{
		dx.push_back(data2_max_z[i] - data1_max_z[i]);
		dxall += data2_max_z[i] - data1_max_z[i];
	}
	dx_avg = dxall / data1_max_z.size();
	for (int i = 0; i < data1_max_z.size(); ++i)
	{
		error += (dx[i] - dx_avg)*(dx[i] - dx_avg);
	}
	error = sqrt(error / data1_max_z.size() - 1);
	std::vector<double> good_dx;
	for (int i = 0; i < dx.size(); ++i)
	{
		double dis = dx[i] - dx_avg;
		if (dis < 3 * error)
		{
			good_dx.push_back(dx[i]);
		}
	}

	double tran_z = 0;;
	if (good_dx.size() > 0)
	{
		for (int i = 0; i < good_dx.size(); ++i)
		{
			tran_z += good_dx[i];
		}
		tran_z /= good_dx.size();
	}
	else
	{
		tran_z = dx_avg;
	}
	transformation(2, 3) = tran_z;
	string trans2d_path = Dir_path + "/" + "trans.txt";   //Get the folder path
	//Output the transformation matrix
	out = fopen(trans2d_path.data(), "w");
	fprintf(out, "%lf %lf %lf  %lf\n", transformation(0, 0), transformation(0, 1), transformation(0, 2), transformation(0, 3));
	fprintf(out, "%lf %lf %lf  %lf\n", transformation(1, 0), transformation(1, 1), transformation(1, 2), transformation(1, 3));
	fprintf(out, "%lf %lf %lf  %lf\n", transformation(2, 0), transformation(2, 1), transformation(2, 2), transformation(2, 3));
	fprintf(out, "%lf %lf %lf  %lf\n", transformation(3, 0), transformation(3, 1), transformation(3, 2), transformation(3, 3));
	fclose(out);

	char* t1 = "UAV_coarse.txt";
	//Output the transformation matrix
	out = fopen(t1, "w");
	for (int i = 0; i < UAV_Data.x.size(); ++i)
	{
		double x = UAV_Data.x[i] * transformation(0, 0) + UAV_Data.y[i] * transformation(0, 1) + transformation(0, 3)+ Backpack_Data.prop.X_Offset;
		double y = UAV_Data.x[i] * transformation(1, 0) + UAV_Data.y[i] * transformation(1, 1) + transformation(1, 3) + Backpack_Data.prop.Y_Offset;
		double z = UAV_Data.z[i] + transformation(2, 3) + Backpack_Data.prop.Z_Offset;
		fprintf(out, "%lf %lf %lf  \n", x, y, z);

	}
	fclose(out);
	char* t2 = "Backpack_coarse.txt";
	//Output the transformation matrix
	out = fopen(t2, "w");
	for (int i = 0; i < Backpack_Data.x.size(); ++i)
	{
		fprintf(out, "%lf %lf %lf  \n", Backpack_Data.x[i]+ Backpack_Data.prop.X_Offset, Backpack_Data.y[i] + Backpack_Data.prop.Y_Offset, Backpack_Data.z[i] + Backpack_Data.prop.Z_Offset);

	}
	fclose(out);

	
	string match_path = Dir_path + "/" + "match.png";   //Get the folder path																	  
	cv::Mat imageOutput;
	drawMatches(img1, keyPoint1, img2, keyPoint2, matchePoints, imageOutput, cv::Scalar::all(-1),
		cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	imwrite(match_path.data(), imageOutput);
	cv::namedWindow("match", CV_WINDOW_AUTOSIZE);
	cv::imshow("match", imageOutput);
	//cv::waitKey(0);

	return true;
}

Registration::~Registration()
{
}

