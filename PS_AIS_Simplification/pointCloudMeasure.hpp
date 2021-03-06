/*********************************************************************************

			 Point Cloud SImplification and Resampling Evaluation

						Updating in 2020/08/13

						   By Dr. Chenlei Lv

			The functions includes:
			1. Achieve the Hausdorff Distance and Mean Distance 
			   between two point clouds;
			


*********************************************************************************/


#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <GL/freeglut.h> 
#include "GLM/glm.h"
#include "GLM/glmVector.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h> 
#include "ballRegionCompute.hpp"
#include "normalCompute.hpp"
//#include "NormalEstimation.hpp"

#define Pi 3.1415926

class simMeasurement{

public:

	vector<vector<double>> pointSet_Simplification;
	vector<vector<double>> pointSet_Orignal_New;
	BallRegion pointSet_Orignal_br;//origanal ballregion
	BallRegion pointSet_Simplification_br;//new origanal
	NormalEstimation ne;
	vector<double> pointSet_ShapeValue;
	double h; //estimate distance between different points	

public:

	void simMeasurement_init(vector<vector<double>> pointSet_Simplification_Input, BallRegion pointSet_input) {

		cout << "simplification init run:" << endl;
		pointSet_Simplification = pointSet_Simplification_Input;
		pointSet_Orignal_New = pointSet_input.pointCloudData;
		pointSet_Orignal_br = pointSet_input;
		cout << "simplification points normal estimate:" << endl;
		//ne.estimateNormal(pointSet_Simplification_Input,'j');
		ne.estimateNormal_PCL_MP(pointSet_Simplification_Input);
		cout << "simplification points radius estimate:" << endl;


		vector<int> pointBorder_input = simMeasurement_PointSetBorder(pointSet_Simplification);
		pointSet_Simplification_br.BallRegion_init(pointSet_Simplification, ne.normalVector, pointBorder_input);
		//pointSet_Simplification_br.BallRegion_init_simpli(pointSet_Simplification, ne.normalVector, pointSet_Orignal_br, 12);
		h = pointSet_Simplification_br.radius;
		cout << "simplification init finish:" << endl;
	}
	
	vector<double> simMeasurement_ShapeValue_fast() {

		pointSet_ShapeValue.resize(pointSet_Orignal_br.pointCloudData.size());

		//set neibor num
		int neiorNumber = pointSet_Simplification_br.pointNumEsti;
		int K = neiorNumber + 1;
		//construct neibor information
		vector<vector<int>> p_sim_neibor(pointSet_Orignal_br.pointCloudData.size());
		vector<vector<float>> p_sim_neibor_distance(pointSet_Orignal_br.pointCloudData.size());
		vector<vector<vector<double>>> pointNeiborPoints(pointSet_Orignal_br.pointCloudData.size());
		vector<vector<vector<double>>> pointNeiborNormal(pointSet_Orignal_br.pointCloudData.size());
		clock_t t1 = clock();
#pragma omp parallel for
		for (int i = 0; i < pointSet_Orignal_br.pointCloudData.size(); ++i) {
			if (i % 10000 == 0) {
				cout << (pointSet_Orignal_br.pointCloudData.size() - i) / 10000 << ",";
			}
			p_sim_neibor[i].resize(K);
			p_sim_neibor_distance[i].resize(K);
			std::vector<int> result;
			pcl::PointXYZ searchPoint;
			searchPoint.x = pointSet_Orignal_br.pointCloudData[i][0];
			searchPoint.y = pointSet_Orignal_br.pointCloudData[i][1];
			searchPoint.z = pointSet_Orignal_br.pointCloudData[i][2];
			pointSet_Simplification_br.kdtree.nearestKSearch(searchPoint, K, p_sim_neibor[i], p_sim_neibor_distance[i]);

			if (p_sim_neibor_distance[i][0] == 0) {
				result.insert(result.end(), p_sim_neibor[i].begin() + 1, p_sim_neibor[i].end());
			}
			else {
				result.insert(result.end(), p_sim_neibor[i].begin(), p_sim_neibor[i].end() - 1);
			}
			p_sim_neibor[i].clear();
			p_sim_neibor[i].insert(p_sim_neibor[i].end(), result.begin(), result.end());
			pointNeiborPoints[i].resize(neiorNumber);
			pointNeiborNormal[i].resize(neiorNumber);
			for (int j = 0; j < neiorNumber; j++) {
				pointNeiborPoints[i][j] = pointSet_Simplification_br.pointCloudData[p_sim_neibor[i][j]];
				pointNeiborNormal[i][j] = pointSet_Simplification_br.pointNormal[p_sim_neibor[i][j]];
			}
			for (int j = 1; j < pointNeiborNormal[i].size(); j++) {
				if (acos(pointNeiborNormal[i][0][0] * pointNeiborNormal[i][j][0] + pointNeiborNormal[i][0][1] * pointNeiborNormal[i][j][1] +
					pointNeiborNormal[i][0][2] * pointNeiborNormal[i][j][2]) >
					acos(-pointNeiborNormal[i][0][0] * pointNeiborNormal[i][j][0] - pointNeiborNormal[i][0][1] * pointNeiborNormal[i][j][1] -
						pointNeiborNormal[i][0][2] * pointNeiborNormal[i][j][2])) {
					pointNeiborNormal[i][j][0] = -pointNeiborNormal[i][j][0];
					pointNeiborNormal[i][j][1] = -pointNeiborNormal[i][j][1];
					pointNeiborNormal[i][j][2] = -pointNeiborNormal[i][j][2];
				}
			}

		}
		cout << endl;		

		vector<vector<int>>(p_sim_neibor).swap(p_sim_neibor);
		vector<vector<float>>(p_sim_neibor_distance).swap(p_sim_neibor_distance);		

		//construct normal information
		int iterNum = 10;
		double errorExist = 0.0001;
		double geometricErrorSum = 0;
		double geometricMax = -1;
		double geometricAvg = 0;
		int geometricMaxIndex = 0;
#pragma omp parallel for
		for (int i = 0; i < pointSet_Orignal_br.pointCloudData.size(); i++) {
			int iter = iterNum;
			if (i % 10000 == 0) {
				cout << (pointSet_Orignal_br.pointCloudData.size() - i) / 10000 << ",";
			}			
			vector<double> px;
			px.insert(px.end(), pointSet_Orignal_br.pointCloudData[i].begin(), pointSet_Orignal_br.pointCloudData[i].end());
			vector<double> px_store(3);
			vector<double> ax;//new point position
			ax.push_back(0);
			ax.push_back(0);
			ax.push_back(0);
			vector<double> nx;//new point normal
			nx.push_back(0);
			nx.push_back(0);
			nx.push_back(0);
			double errorEndTem;//record new 
			double errorStore = 9999;
			double weight;
			while (iter) {				
				double fenmu = 0;
				for (int j = 0; j < pointNeiborPoints[i].size(); j++) {
					double dis_i = sqrt((pointNeiborPoints[i][j][0] - px[0]) * (pointNeiborPoints[i][j][0] - px[0]) +
						(pointNeiborPoints[i][j][1] - px[1]) * (pointNeiborPoints[i][j][1] - px[1]) +
						(pointNeiborPoints[i][j][2] - px[2]) * (pointNeiborPoints[i][j][2] - px[2]));
					if (dis_i == 0) {
						continue;
					}
					double eData = -((dis_i / h) * (dis_i / h));
					eData = exp(eData);
					ax[0] = ax[0] + pointNeiborPoints[i][j][0] * eData;
					ax[1] = ax[1] + pointNeiborPoints[i][j][1] * eData;
					ax[2] = ax[2] + pointNeiborPoints[i][j][2] * eData;
					nx[0] = nx[0] + pointNeiborNormal[i][j][0] * eData;
					nx[1] = nx[1] + pointNeiborNormal[i][j][1] * eData;
					nx[2] = nx[2] + pointNeiborNormal[i][j][2] * eData;
					fenmu = fenmu + eData;
				}
				if (fenmu != 0) {
					ax[0] = ax[0] / fenmu;
					ax[1] = ax[1] / fenmu;
					ax[2] = ax[2] / fenmu;
					nx[0] = nx[0] / fenmu;
					nx[1] = nx[1] / fenmu;
					nx[2] = nx[2] / fenmu;
				}
				fenmu = 0;

				//5.3 Set x' = x - n(x')T(a(x')-x)n(x'), weight = n(x')T(a(x')-x)
				weight = nx[0] * (pointSet_Orignal_br.pointCloudData[i][0] - ax[0]) +
					nx[1] * (pointSet_Orignal_br.pointCloudData[i][1] - ax[1]) + nx[2] * (pointSet_Orignal_br.pointCloudData[i][2] - ax[2]);
				px_store[0] = px[0];
				px_store[1] = px[1];
				px_store[2] = px[2];
				px[0] = pointSet_Orignal_br.pointCloudData[i][0] - weight * nx[0];
				px[1] = pointSet_Orignal_br.pointCloudData[i][1] - weight * nx[1];
				px[2] = pointSet_Orignal_br.pointCloudData[i][2] - weight * nx[2];

				//5.4 ||n(x')T(a(x')-x)n(x')||>errorEndTem
				ax[0] = 0;
				ax[1] = 0;
				ax[2] = 0;
				nx[0] = 0;
				nx[1] = 0;
				nx[2] = 0;				

				for (int j = 0; j < pointNeiborPoints[i].size(); j++) {
					double dis_i = sqrt((pointNeiborPoints[i][j][0] - px[0]) * (pointNeiborPoints[i][j][0] - px[0]) +
						(pointNeiborPoints[i][j][1] - px[1]) * (pointNeiborPoints[i][j][1] - px[1]) +
						(pointNeiborPoints[i][j][2] - px[2]) * (pointNeiborPoints[i][j][2] - px[2]));
					if (dis_i == 0) {
						continue;
					}
					double eData = -((dis_i / h) * (dis_i / h));
					eData = exp(eData);
					ax[0] = ax[0] + pointNeiborPoints[i][j][0] * eData;
					ax[1] = ax[1] + pointNeiborPoints[i][j][1] * eData;
					ax[2] = ax[2] + pointNeiborPoints[i][j][2] * eData;
					nx[0] = nx[0] + pointNeiborNormal[i][j][0] * eData;
					nx[1] = nx[1] + pointNeiborNormal[i][j][1] * eData;
					nx[2] = nx[2] + pointNeiborNormal[i][j][2] * eData;
					fenmu = fenmu + eData;
				}
				if (fenmu != 0) {
					ax[0] = ax[0] / fenmu;
					ax[1] = ax[1] / fenmu;
					ax[2] = ax[2] / fenmu;
					nx[0] = nx[0] / fenmu;
					nx[1] = nx[1] / fenmu;
					nx[2] = nx[2] / fenmu;
				}

				weight = nx[0] * (pointSet_Orignal_br.pointCloudData[i][0] - ax[0]) +
					nx[1] * (pointSet_Orignal_br.pointCloudData[i][1] - ax[1]) + nx[2] * (pointSet_Orignal_br.pointCloudData[i][2] - ax[2]);

				ax[0] = 0;
				ax[1] = 0;
				ax[2] = 0;
				nx[0] = 0;
				nx[1] = 0;
				nx[2] = 0;

				errorEndTem = abs(weight);
				if (errorEndTem < errorStore) {
					errorStore = errorEndTem;
				}
				else {
					px[0] = px_store[0];
					px[1] = px_store[1];
					px[2] = px_store[2];
					break;
				}
				if (errorEndTem < errorExist) {//|| errorEndTem > errorstore
					break;
				}
				iter--;
			}
			pointSet_Orignal_New[i][0] = px[0];
			pointSet_Orignal_New[i][1] = px[1];
			pointSet_Orignal_New[i][2] = px[2];
			pointSet_ShapeValue[i] = sqrt((px[0] - pointSet_Orignal_br.pointCloudData[i][0]) * (px[0] - pointSet_Orignal_br.pointCloudData[i][0]) +
				(px[1] - pointSet_Orignal_br.pointCloudData[i][1]) * (px[1] - pointSet_Orignal_br.pointCloudData[i][1]) +
				(px[2] - pointSet_Orignal_br.pointCloudData[i][2]) * (px[2] - pointSet_Orignal_br.pointCloudData[i][2]));
			if (pointSet_ShapeValue[i] > geometricMax) {
				geometricMax = pointSet_ShapeValue[i];
			}
		}
		for (int i = 0; i < pointSet_ShapeValue.size(); i++) {
			geometricErrorSum = geometricErrorSum + pointSet_ShapeValue[i];
		}

		std::cout << endl;
		clock_t t2 = clock();
		std::cout << "geometric error running time:" << (t2 - t1) / 1000.0 << "s" << endl;
		geometricAvg = geometricErrorSum / (double)pointSet_Orignal_br.pointCloudData.size();

		cout << "Result:" << endl;
		cout << "h:" << h << endl;
		cout << "maxError:" << geometricMax << endl;
		cout << "avgError:" << geometricAvg << endl;
		cout << "SamplingRate:" << (double)pointSet_Simplification.size() / (double)pointSet_Orignal_New.size() << endl;
		cout << "Detail:" << pointSet_Simplification.size() << "/" << pointSet_Orignal_New.size() << endl;


		vector<double> resultFinal;
		resultFinal.push_back(geometricAvg);
		resultFinal.push_back(geometricMax);
		return resultFinal;

	}
	
private:

	vector<int> simMeasurement_PointSetBorder(vector<vector<double>> pointSet) {

		double maxx, maxy, maxz, minx, miny, minz;
		double cx, cy, cz, w, h, d;
		double scale;
		maxx = minx = pointSet[0][0];
		maxy = miny = pointSet[0][1];
		maxz = minz = pointSet[0][2];

		int indexMaxX = 0;
		int indexMaxY = 0;
		int indexMaxZ = 0;
		int indexMinX = 0;
		int indexMinY = 0;
		int indexMinZ = 0;

		for (int i = 0; i < pointSet.size(); i++) {
			double xi = pointSet[i][0];
			double yi = pointSet[i][1];
			double zi = pointSet[i][2];
			if (xi < minx) {
				minx = xi;
				indexMinX = i;
			}
			if (xi > maxx) {
				maxx = xi;
				indexMaxX = i;
			}
			if (yi < miny) {
				miny = yi;
				indexMinY = i;
			}
			if (yi > maxy) {
				maxy = yi;
				indexMaxY = i;
			}
			if (zi < minz) {
				minz = zi;
				indexMinZ = i;
			}
			if (zi > maxz) {
				maxz = zi;
				indexMaxZ = i;
			}
		}

		vector<int> indexBorder;
		indexBorder.push_back(indexMinX);
		indexBorder.push_back(indexMinY);
		indexBorder.push_back(indexMinZ);
		indexBorder.push_back(indexMaxX);
		indexBorder.push_back(indexMaxY);
		indexBorder.push_back(indexMaxZ);
		return indexBorder;
	
	}



};
