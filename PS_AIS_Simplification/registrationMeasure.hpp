/*********************************************************************************

			  Point Cloud Registration Quality Measurement

						Updating in 2021/02/24

						   By Dr. Chenlei Lv

			The functions includes:
			1. MSE, RMSE, MAE computation

*********************************************************************************/

#pragma once
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>

using namespace std;

class PCR_QM {

private:

	vector<vector<double>> a;
	vector<vector<double>> t;
	vector<double> MSERA;

public:

	void PCR_QM_init(vector<vector<double>> alignV, vector<vector<double>> templateV) {

		a = alignV;
		t = templateV;
		PCR_QM_Start();

	}

	vector<double> PCR_QM_ReturnResult() {

		return MSERA;

	}

private:

	void PCR_QM_Start() {

		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;//kdtree
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->width = t.size();
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);
		// fills a PointCloud with random data
		for (int i = 0; i < t.size(); i++)
		{
			pcl::PointXYZ pxyz;
			cloud->points[i].x = t[i][0];
			cloud->points[i].y = t[i][1];
			cloud->points[i].z = t[i][2];

		}
		kdtree.setInputCloud(cloud);
		int K = 1;

		double mseSum = 0;
		double maeSum = 0;
		double rmseSum = 0;
		for (int i = 0; i < a.size(); i++) {

			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);
			std::vector<int> result;
			double r_i = pointNKNSquaredDistance[pointNKNSquaredDistance.size() - 1];
			pcl::PointXYZ searchPoint;
			searchPoint.x = a[i][0];
			searchPoint.y = a[i][1];
			searchPoint.z = a[i][2];
			kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
			double mse_distancei = pointNKNSquaredDistance[0];
			double mae_distancei = sqrt(mse_distancei);
			mseSum = mseSum + mse_distancei;
			maeSum = maeSum + mae_distancei;

		}
		mseSum = mseSum / (double)a.size();
		maeSum = maeSum / (double)a.size();
		rmseSum = sqrt(mseSum);
		std::cout << "Result:" << endl;
		std::cout << "MSE:  " << mseSum << endl;
		std::cout << "RMSE: " << rmseSum << endl;
		std::cout << "MAE:  " << maeSum << endl;

		MSERA.clear();
		MSERA.push_back(mseSum);
		MSERA.push_back(rmseSum);
		MSERA.push_back(maeSum);
	}
};



