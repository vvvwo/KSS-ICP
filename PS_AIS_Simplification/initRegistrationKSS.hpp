/*********************************************************************************

			 Init Point Cloud Position For Shape Registration

						Updating in 2021/03/02

						   By Dr. Chenlei Lv

			The functions includes:
			1. Scale estimation
			2. Rotation estimation

*********************************************************************************/

#pragma once
#include <iostream>
#include <math.h> 
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>


using namespace std;

class initRegistration_KSS{

private:

	double step;
	vector<vector<vector<double>>> value;
	int irange, jrange, krange;
	int r = 2; //kernel radius
public:

	double x_middle_S;//target middle point
	double y_middle_S;
	double z_middle_S;
	double x_middle;//source to target middle vector
	double y_middle;
	double z_middle;
	double scale;//source transfer scale
	vector<double> angle;
	vector<vector<double>> angleList;
	vector<vector<double>> pointSource;
	vector<vector<double>> pointTarget;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	vector<double> rotationRecord;

public:

	void initRegistration_init(vector<vector<double>> pointinput, vector<vector<double>> pointinput2, double accurate) {

		step = accurate;

		cout << "initRegistration start." << endl;
		pointSource = pointinput;
		pointTarget = pointinput2;
		cout << "initRegistration middle align." << endl;
		initRegistration_MiddleAlign();
		cout << "initRegistration rotation." << endl;
		//initRegistration_Rotation();

		long start = clock();
		//initRegistration_RotationP();
		initRegistration_Rotation();
		long end = clock();
		cout << "alignment time cost:" << (end - start) / 1000 << "s" << endl;


	}
	
	vector<vector<double>> initRegistration_Rotation(vector<vector<double>> sourceOri) {		

		for (int i = 0; i < sourceOri.size(); i++) {
			sourceOri[i][0] = sourceOri[i][0] + x_middle;
			sourceOri[i][1] = sourceOri[i][1] + y_middle;
			sourceOri[i][2] = sourceOri[i][2] + z_middle;
			sourceOri[i][0] = x_middle_S + (sourceOri[i][0] - x_middle_S) * scale;
			sourceOri[i][1] = y_middle_S + (sourceOri[i][1] - y_middle_S) * scale;
			sourceOri[i][2] = z_middle_S + (sourceOri[i][2] - z_middle_S) * scale;
		}

		vector<vector<double>> r1 = initRegistration_Transfer(1, angle[0], sourceOri);
		vector<vector<double>> r2 = initRegistration_Transfer(2, angle[1], r1);
		vector<vector<double>> r3 = initRegistration_Transfer(3, angle[2], r2);
		return r3;
	
	}
	
	vector<vector<double>> initRegistration_Rotation_Angle(vector<vector<double>> sourceOri, vector<double> angle_T) {

		for (int i = 0; i < sourceOri.size(); i++) {
			sourceOri[i][0] = sourceOri[i][0] + x_middle;
			sourceOri[i][1] = sourceOri[i][1] + y_middle;
			sourceOri[i][2] = sourceOri[i][2] + z_middle;
			sourceOri[i][0] = x_middle_S + (sourceOri[i][0] - x_middle_S) * scale;
			sourceOri[i][1] = y_middle_S + (sourceOri[i][1] - y_middle_S) * scale;
			sourceOri[i][2] = z_middle_S + (sourceOri[i][2] - z_middle_S) * scale;
		}

		vector<vector<double>> r1 = initRegistration_Transfer(1, angle_T[0], sourceOri);
		vector<vector<double>> r2 = initRegistration_Transfer(2, angle_T[1], r1);
		vector<vector<double>> r3 = initRegistration_Transfer(3, angle_T[2], r2);
		return r3;

	}

	vector<vector<double>> initRegistration_Rotation_Axis(vector<vector<double>> sourceOri, int axis, double angleV) {

		for (int i = 0; i < sourceOri.size(); i++) {
			sourceOri[i][0] = sourceOri[i][0] - x_middle_S;
			sourceOri[i][1] = sourceOri[i][1] - x_middle_S;
			sourceOri[i][2] = sourceOri[i][2] - x_middle_S;		
		}
		vector<vector<double>> rv;
		if (axis == 1) {
			rv = initRegistration_Transfer(1, angleV, sourceOri);			
		}
		else if (axis == 2) {
			rv = initRegistration_Transfer(2, angleV, sourceOri);				
		}
		else if (axis == 3) {
			rv = initRegistration_Transfer(3, angleV, sourceOri);					
		}
		else {
			cout << "error! illegal rotation" << endl;
			return sourceOri;		
		}

		for (int i = 0; i < rv.size(); i++) {
			rv[i][0] = rv[i][0] + x_middle_S;
			rv[i][1] = rv[i][1] + x_middle_S;
			rv[i][2] = rv[i][2] + x_middle_S;
		}
		return rv;
		
	}

private:

	void initRegistration_MiddleAlign() {

		double xSum = 0;
		double ySum = 0;
		double zSum = 0;
		double avergeScale = 0;
		for (int i = 0; i < pointSource.size(); i++) {
			xSum = xSum + pointSource[i][0];
			ySum = ySum + pointSource[i][1];
			zSum = zSum + pointSource[i][2];
		}
		xSum = xSum / pointSource.size();
		ySum = ySum / pointSource.size();
		zSum = zSum / pointSource.size();		

		double lengthMax1 = -1;
		for (int i = 0; i < pointSource.size(); i++) {
			double xl = pointSource[i][0] - xSum;
			double yl = pointSource[i][1] - ySum;
			double zl = pointSource[i][2] - zSum;
			double length_i = sqrt(xl * xl + yl * yl + zl * zl);
			avergeScale = avergeScale + length_i;
			//if (length_i > lengthMax1) {
				//lengthMax1 = length_i;			
			//}
		}
		//avergeScale = lengthMax1;
		avergeScale = avergeScale / pointSource.size();

		double xSum2 = 0;
		double ySum2 = 0;
		double zSum2 = 0;
		double avergeScale2 = 0;
		for (int i = 0; i < pointTarget.size(); i++) {
			xSum2 = xSum2 + pointTarget[i][0];
			ySum2 = ySum2 + pointTarget[i][1];
			zSum2 = zSum2 + pointTarget[i][2];
		}
		xSum2 = xSum2 / pointTarget.size();
		ySum2 = ySum2 / pointTarget.size();
		zSum2 = zSum2 / pointTarget.size();

		x_middle_S = xSum2;
		y_middle_S = ySum2;
		z_middle_S = zSum2;

		x_middle = xSum2 - xSum;
		y_middle = ySum2 - ySum;
		z_middle = zSum2 - zSum;

		//double lengthMax2 = -1;
		for (int i = 0; i < pointTarget.size(); i++) {
			double xl = pointTarget[i][0] - xSum2;
			double yl = pointTarget[i][1] - ySum2;
			double zl = pointTarget[i][2] - zSum2;
			double length_i = sqrt(xl * xl + yl * yl + zl * zl);				
			avergeScale2 = avergeScale2 + length_i;
			//if (length_i > lengthMax2) {
				//lengthMax2 = length_i;			
			//}			
		}

		//avergeScale2 = lengthMax2;
		avergeScale2 = avergeScale2 / pointTarget.size();

		scale = avergeScale2 / avergeScale;

		//scale unifrom
		for (int i = 0; i < pointSource.size(); i++) {
			pointSource[i][0] = pointSource[i][0] + x_middle;
			pointSource[i][1] = pointSource[i][1] + y_middle;
			pointSource[i][2] = pointSource[i][2] + z_middle;
			pointSource[i][0] = xSum2 + (pointSource[i][0] - xSum2) * scale;
			pointSource[i][1] = ySum2 + (pointSource[i][1] - ySum2) * scale;
			pointSource[i][2] = zSum2 + (pointSource[i][2] - zSum2) * scale;
		}
	}

	void initRegistration_Rotation() {
		//kd-tree init		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->width = pointTarget.size();
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);
		// fills a PointCloud with random data
		for (int i = 0; i < pointTarget.size(); i++)
		{
			pcl::PointXYZ pxyz;
			cloud->points[i].x = pointTarget[i][0];
			cloud->points[i].y = pointTarget[i][1];
			cloud->points[i].z = pointTarget[i][2];
		}
		kdtree.setInputCloud(cloud);

		vector<vector<double>> ps_final;
		double errorT = 9999;
		bool bJudge = true;		
		double iG, jG, kG;

		value.clear();

		for (double i = 0; i < 6.3; i = i + 6.3 / step) {
			cout << "iter:" << (6.3 - i) / (6.3 / step) << ";";
			vector<vector<double>> ps_x = initRegistration_Transfer(1, i, pointSource);
			vector<vector<double>> vi;
			for (double j = 0; j < 6.3; j = j + 6.3 / step) {
				vector<vector<double>> ps_xy = initRegistration_Transfer(2, j, ps_x);
				vector<double> vj;
				for (double k = 0; k < 6.3; k = k + 6.3 / step) {
					vector<vector<double>> ps_xyz = initRegistration_Transfer(3, k, ps_xy);
					//double error_ijk = initRegistration_Error(ps_xyz);
					double error_ijk = initRegistration_Error_Ave(ps_xyz);
					//double error_ijk = initRegistration_Error_Diff(ps_xyz);	
					vj.push_back(error_ijk);									
					if (error_ijk < errorT) {
						iG = i;
						jG = j;
						kG = k;
						errorT = error_ijk;
						ps_final.clear();
						ps_final = ps_xyz;						
					}
				}
				vi.push_back(vj);
			}
			value.push_back(vi);
		}

		irange = value.size(); 
		jrange = value[0].size();
		krange = value[0][0].size();
       
		if (errorT > 0.01) {
			for (int i = 0; i < value.size(); i++) {
				for (int j = 0; j < value[i].size(); j++) {
					for (int k = 0; k < value[i][j].size(); k++) {
						bool resultJudge = initRegistration_kernel(i, j, k);
						if (resultJudge) {
							vector<double> angleijk;
							angleijk.push_back((double)i * 6.3 / (double)step);
							angleijk.push_back((double)j * 6.3 / (double)step);
							angleijk.push_back((double)k * 6.3 / (double)step);
							angleList.push_back(angleijk);
						}
					}
				}			
			}		
		}


		angle.push_back(iG);
		angle.push_back(jG);
		angle.push_back(kG);
		cout << "i:" << iG << "j:" << jG << "k:" << kG << endl;
		cout << endl;
	}

	void initRegistration_RotationP() {
		//kd-tree init		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->width = pointTarget.size();
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);
		// fills a PointCloud with random data
		for (int i = 0; i < pointTarget.size(); i++)
		{
			pcl::PointXYZ pxyz;
			cloud->points[i].x = pointTarget[i][0];
			cloud->points[i].y = pointTarget[i][1];
			cloud->points[i].z = pointTarget[i][2];
		}
		kdtree.setInputCloud(cloud);

		vector<vector<double>> ps_final;
		double errorT = 9999;
		bool bJudge = true;
		double step = 6;

		vector<vector<vector<double>>> list;

		for (double i = 0; i < 6; i = i + 6 / step) {
			cout << "iter:" << (6 - i) / (6 / step) << ";";
			vector<vector<double>> ps_x = initRegistration_Transfer(1, i, pointSource);
			for (double j = 0; j < 6; j = j + 6 / step) {
				vector<vector<double>> ps_xy = initRegistration_Transfer(2, j, ps_x);
				for (double k = 0; k < 6; k = k + 6 / step) {
					vector<vector<double>> ps_xyz = initRegistration_Transfer(3, k, ps_xy);
					list.push_back(ps_xyz);
				}
			}
		}
		cout << endl;
		vector<double> errorList(list.size(), 0);

#pragma omp parallel for
		for (int i = 0; i < list.size(); i++) {
			errorList[i] = initRegistration_Error(list[i]);
		}

		for (int i = 0; i < errorList.size(); i++) {
			if (errorList[i] < errorT) {
				errorT = errorList[i];
				ps_final.clear();
				ps_final = list[i];
			}
		}

		pointSource.clear();
		pointSource = ps_final;

		for (int i = 0; i < pointTarget.size(); i++) {
			pointTarget[i][0] = pointTarget[i][0] + x_middle;
			pointTarget[i][1] = pointTarget[i][1] + y_middle;
			pointTarget[i][2] = pointTarget[i][2] + z_middle;
		}

		for (int i = 0; i < pointSource.size(); i++) {
			pointSource[i][0] = pointSource[i][0] + x_middle;
			pointSource[i][1] = pointSource[i][1] + y_middle;
			pointSource[i][2] = pointSource[i][2] + z_middle;
		}

	}

	vector<vector<double>> initRegistration_Transfer(int cord, double angle, vector<vector<double>> pointResample) {//cord 1:x, 2:y, 3:z; angle transfer angle

		if (cord == 1) {

			for (int i = 0; i < pointResample.size(); i++) {
				double xi = pointResample[i][0];
				double yi = pointResample[i][1] * cos(angle) - pointResample[i][2] * sin(angle);
				double zi = pointResample[i][1] * sin(angle) + pointResample[i][2] * cos(angle);
				pointResample[i][0] = xi;
				pointResample[i][1] = yi;
				pointResample[i][2] = zi;
			}
		}
		else if (cord == 2) {

			for (int i = 0; i < pointResample.size(); i++) {
				double xi = pointResample[i][2] * sin(angle) + pointResample[i][0] * cos(angle);
				double yi = pointResample[i][1];
				double zi = pointResample[i][2] * cos(angle) - pointResample[i][0] * sin(angle);
				pointResample[i][0] = xi;
				pointResample[i][1] = yi;
				pointResample[i][2] = zi;
			}
		}
		else {

			for (int i = 0; i < pointResample.size(); i++) {
				double xi = pointResample[i][0] * cos(angle) - pointResample[i][1] * sin(angle);
				double yi = pointResample[i][0] * sin(angle) + pointResample[i][1] * cos(angle);
				double zi = pointResample[i][2];
				pointResample[i][0] = xi;
				pointResample[i][1] = yi;
				pointResample[i][2] = zi;
			}

		}

		return pointResample;

	}

	double initRegistration_Error(vector<vector<double>> pointS) {

		int K = 2;
		double distanceMax = -9999;
		for (int i = 0; i < pointS.size(); i++) {

			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);
			std::vector<int> result;
			pcl::PointXYZ searchPoint;
			searchPoint.x = pointS[i][0];
			searchPoint.y = pointS[i][1];
			searchPoint.z = pointS[i][2];
			kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
			double distance_i = pointNKNSquaredDistance[0];
			if (distance_i > distanceMax) {
				distanceMax = distance_i;
			}

		}
		return distanceMax;

	}

	double initRegistration_Error_Ave(vector<vector<double>> pointS) {

		int K = 2;
		double distanceSum = 0;
		for (int i = 0; i < pointS.size(); i++) {

			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);
			std::vector<int> result;
			pcl::PointXYZ searchPoint;
			searchPoint.x = pointS[i][0];
			searchPoint.y = pointS[i][1];
			searchPoint.z = pointS[i][2];
			kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
			double distance_i = sqrt(pointNKNSquaredDistance[0]);
			distanceSum = distanceSum + distance_i;

		}
		return distanceSum / pointS.size();

	}

	double initRegistration_Error_Diff(vector<vector<double>> pointS) {

		int K = 2;
		double distanceMax = -9999;
		double distanceMin = 9999;
		double distanceSum = 0;
		for (int i = 0; i < pointS.size(); i++) {

			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);
			std::vector<int> result;
			pcl::PointXYZ searchPoint;
			searchPoint.x = pointS[i][0];
			searchPoint.y = pointS[i][1];
			searchPoint.z = pointS[i][2];
			kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
			double distance_i = sqrt(pointNKNSquaredDistance[0]);
			distanceSum = distanceSum + distance_i;
			if (distanceMax < distance_i) {
				distanceMax = distance_i;

			}
		}


		return distanceMax - distanceSum / pointS.size();

	}
	
	bool initRegistration_kernel(int i, int j, int k) {

		double valueCenter = value[i][j][k];

		int id = i - r;
		int iu = i + r;
		int jd = j - r;
		int ju = j + r;
		int kd = k - r;
		int ku = k + r;

		if (id < 0) {
			id = 0;		
		}
		if (iu >= irange) {
			iu = irange - 1;
		}
		if (jd < 0) {
			jd = 0;
		}
		if (ju >= jrange) {
			ju = jrange - 1;
		}
		if (kd < 0) {
			kd = 0;
		}
		if (ku >= krange) {
			ku = krange - 1;
		}		
	
		for (int ii = id; ii <= iu; ii++) {
			for (int jj = jd; jj <= ju; jj++) {
				for (int kk = kd; kk <= ku; kk++) {					
					if (valueCenter > value[ii][jj][kk]) {
						return false;					
					}
				}
			}		
		}
		return true;
	
	}

};


