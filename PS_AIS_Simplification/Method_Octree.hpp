#pragma once
#include <pcl/io/pcd_io.h>  //File input$output
#include <pcl/octree/octree_search.h>  //octree define
#include <pcl/point_types.h>  //point type
#include <iostream>
#include <vector>

using namespace std;

typedef std::vector< pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > AlignedPointTVector;

class PCL_octree {

private:

	int regularPoint = 80000;

public:

	vector<vector<vector<double>>> PCL_Octree_Simplification(vector<vector<double>> pData,
		vector<vector<double>> nData) {

		cout << "PCL down-sampling start:" << endl;
		clock_t t1;
		clock_t t2;
		float resolution = PCL_Octree_Resolution(pData);		
		t1 = clock();
		vector<vector<double>> resultP;
		vector<vector<double>> resultN;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		// fills a PointCloud with random data
		for (int i = 0; i < pData.size(); i++)
		{
			pcl::PointXYZ cloud_i;
			cloud_i.x = pData[i][0];
			cloud_i.y = pData[i][1];
			cloud_i.z = pData[i][2];
			cloud->push_back(cloud_i);
		}
		// construct orctree
		//float resolution = 0.03f; 
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution); 
		octree.setInputCloud(cloud); 
		octree.addPointsFromInputCloud(); 
		AlignedPointTVector voxel_center_list_arg;
		octree.getOccupiedVoxelCenters(voxel_center_list_arg);
		for (int i = 0; i < voxel_center_list_arg.size(); i++) {
			pcl::PointXYZ pdi = voxel_center_list_arg[i];
			int K = 1;
			std::vector<int > pointIdxNKNSearch;
			std::vector<float> pointNKNSquaredDistance;
			octree.nearestKSearch(pdi, K, pointIdxNKNSearch, pointNKNSquaredDistance);
			vector<double> pi;
			vector<double> ni;
			pi.push_back(pData[pointIdxNKNSearch[0]][0]);
			pi.push_back(pData[pointIdxNKNSearch[0]][1]);
			pi.push_back(pData[pointIdxNKNSearch[0]][2]);
			ni.push_back(nData[pointIdxNKNSearch[0]][0]);
			ni.push_back(nData[pointIdxNKNSearch[0]][1]);
			ni.push_back(nData[pointIdxNKNSearch[0]][2]);
			resultP.push_back(pi);
			resultN.push_back(ni);
			
		}
		t2 = clock();
		std::cout << "Octree down-sampling:" << (t2 - t1) / 1000.0 << "s" << endl;
		cout << "Original Point:" << pData.size() << endl;
		cout << "Down-sampling Point:" << resultP.size() << endl;
		vector<vector<vector<double>>> result;
		result.push_back(resultP);
		result.push_back(resultN);
		return result;

	}

	vector<vector<double>> PCL_Octree_Simplification_WithOutNormal(vector<vector<double>> pData) {		
				
		float resolution = PCL_Octree_Resolution(pData);
		vector<vector<double>> resultP;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		// fills a PointCloud with random data
		for (int i = 0; i < pData.size(); i++)
		{
			pcl::PointXYZ cloud_i;
			cloud_i.x = pData[i][0];
			cloud_i.y = pData[i][1];
			cloud_i.z = pData[i][2];
			cloud->push_back(cloud_i);
		}		
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution); 
		octree.setInputCloud(cloud); 
		octree.addPointsFromInputCloud(); 
		AlignedPointTVector voxel_center_list_arg;
		octree.getOccupiedVoxelCenters(voxel_center_list_arg);
		for (int i = 0; i < voxel_center_list_arg.size(); i++) {
			pcl::PointXYZ pdi = voxel_center_list_arg[i];
			int K = 1;
			std::vector<int > pointIdxNKNSearch;
			std::vector<float> pointNKNSquaredDistance;
			octree.nearestKSearch(pdi, K, pointIdxNKNSearch, pointNKNSquaredDistance);
			vector<double> pi;
			pi.push_back(pData[pointIdxNKNSearch[0]][0]);
			pi.push_back(pData[pointIdxNKNSearch[0]][1]);
			pi.push_back(pData[pointIdxNKNSearch[0]][2]);
			resultP.push_back(pi);
		}		
		return resultP;
	}

private:

	double PCL_Octree_Estimate_Radius(vector<vector<double>> pData, int kn) {

		pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSeed;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->width = pData.size();
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);
		// fills a PointCloud with random data
		for (int i = 0; i < pData.size(); i++)
		{
			pcl::PointXYZ pxyz;
			cloud->points[i].x = pData[i][0];
			cloud->points[i].y = pData[i][1];
			cloud->points[i].z = pData[i][2];

		}
		kdtreeSeed.setInputCloud(cloud);
		//int kn = 7;
		double radiusSum = 0;
		for (int i = 0; i < 1000; i++) {
			vector<double> seedPoints_i = pData[i];
			vector<int> pointIdxNKNSearch(kn);
			vector<float> pointNKNSquaredDistance(kn);
			std::vector<int> pointNeibor_i;
			pcl::PointXYZ searchPoint;
			searchPoint.x = seedPoints_i[0];
			searchPoint.y = seedPoints_i[1];
			searchPoint.z = seedPoints_i[2];
			kdtreeSeed.nearestKSearch(searchPoint, kn, pointIdxNKNSearch, pointNKNSquaredDistance);
			double radius_i = sqrt(pointNKNSquaredDistance[pointNKNSquaredDistance.size() - 1]);
			radiusSum = radiusSum + radius_i;
		}
		radiusSum = radiusSum / 1000;
		return radiusSum;
	}

	double PCL_Octree_Resolution(vector<vector<double>> pData) {

		float resolution;
		if (pData.size() < regularPoint) {
			resolution = PCL_Octree_Estimate_Radius(pData, 2);
		}		
		else {
			double multiDown = pData.size() / regularPoint;
			int multiDownInt = (int)multiDown;
			if (multiDownInt >= 5) {
				resolution = PCL_Octree_Estimate_Radius(pData, 35);
			}
			else {
				resolution = PCL_Octree_Estimate_Radius(pData, 7 * multiDownInt);
			}
		}
		return resolution;	
	}

};


