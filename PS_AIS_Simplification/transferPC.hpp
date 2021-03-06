/*********************************************************************************

			 Point Cloud Transfer based on Resampling Method

						Updating in 2021/01/05

						   By Dr. Chenlei Lv

			The functions includes:
			1. Resampling point cloud by two method: Gird and Wlop
			2. Transfer point cloud by transfer matrix

*********************************************************************************/

#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h> 
#include <time.h>
#include <functional>
#include <algorithm>
#include "GLM/glm.h"
#include "GLM/glmVector.h"
#include "ballRegionCompute.hpp"
#include "Method_CGAL.hpp"
#include "pointPipeline.hpp"
using namespace std;

class TransferPC {

public:

	string filewlop;
	string filegird;



private:
	
	vector<vector<double>> pointIndex;
	vector<vector<double>> pointResampleWlop;
	vector<vector<double>> pointResampleGird;
	string filePathG;
	double radiusG;

public:

	void TransferPC_init(string filePath) {

		pointPipeline pp;
		pp.pointPipeline_init(filePath, false);
		filePathG = filePath;
		pointIndex = pp.br.pointCloudData;
		radiusG = pp.br.radius;
		int index = filePathG.find_last_of(".");
		filewlop = filePathG.substr(0, index) + ".wlop";
		filegird = filePathG.substr(0, index) + ".gird";
		TransferPC_Resample();
	
	}
	
	void TransferPC_Transfer(int cord, double angle) {//cord 1:x, 2:y, 3:z; angle transfer angle

		if (cord == 1) {
			for (int i = 0; i < pointResampleGird.size(); i++) {
				double xi = pointResampleGird[i][0];
				double yi = pointResampleGird[i][1] * cos(angle) - pointResampleGird[i][2] * sin(angle);
				double zi = pointResampleGird[i][1] * sin(angle) + pointResampleGird[i][2] * cos(angle);
				pointResampleGird[i][0] = xi;
				pointResampleGird[i][1] = yi;
				pointResampleGird[i][2] = zi;
			}		
		}
		else if (cord == 2) {
			for (int i = 0; i < pointResampleGird.size(); i++) {
				double xi = pointResampleGird[i][2] * sin(angle) + pointResampleGird[i][0] * cos(angle);
				double yi = pointResampleGird[i][1];
				double zi = pointResampleGird[i][2] * cos(angle) - pointResampleGird[i][0] * sin(angle);
				pointResampleGird[i][0] = xi;
				pointResampleGird[i][1] = yi;
				pointResampleGird[i][2] = zi;
			}		
		}
		else {	
			for (int i = 0; i < pointResampleGird.size(); i++) {
				double xi = pointResampleGird[i][0] * cos(angle) - pointResampleGird[i][1] * sin(angle);
				double yi = pointResampleGird[i][0] * sin(angle) + pointResampleGird[i][1] * cos(angle);
				double zi = pointResampleGird[i][2];
				pointResampleGird[i][0] = xi;
				pointResampleGird[i][1] = yi;
				pointResampleGird[i][2] = zi;
			}		
		}
	}
	
	void TransferPC_Scale(double rate) {

		double x_sum = 0;
		double y_sum = 0;
		double z_sum = 0;

		for (int i = 0; i < pointResampleGird.size(); i++) {
			x_sum = x_sum + pointResampleGird[i][0];
			y_sum = y_sum + pointResampleGird[i][1];
			z_sum = z_sum + pointResampleGird[i][2];			
		}

		x_sum = x_sum / pointResampleGird.size();
		y_sum = y_sum / pointResampleGird.size();
		z_sum = z_sum / pointResampleGird.size();

		for (int i = 0; i < pointResampleGird.size(); i++) {
			pointResampleGird[i][0] = (pointResampleGird[i][0] - x_sum)* rate + x_sum;
			pointResampleGird[i][1] = (pointResampleGird[i][1] - y_sum) * rate + y_sum;
			pointResampleGird[i][2] = (pointResampleGird[i][2] - z_sum) * rate + z_sum;
		}	
	}

	void TransferPC_Translate(double dis) {		

		for (int i = 0; i < pointResampleGird.size(); i++) {
			pointResampleGird[i][0] = pointResampleGird[i][0] + dis;
			pointResampleGird[i][1] = pointResampleGird[i][1] + dis;
			pointResampleGird[i][2] = pointResampleGird[i][2] + dis;
		}
	}

	vector<vector<vector<double>>> TransferPC_ReturnPoints() {

		TransferPC_SavePC();
		vector<vector<vector<double>>> result;
		result.push_back(pointResampleWlop);
		result.push_back(pointResampleGird);
		return result;
	
	}

private:

	void TransferPC_Resample() {

		simplification_Method_CGAL smc;
		smc.simplification_Method_CGAL_init(pointIndex, radiusG);
		pointResampleWlop = smc.simplification_Method_CGAL_WLOP(8000);
		pointResampleGird = smc.simplification_Method_CGAL_Grid(radiusG / 1.5);

	}

	void TransferPC_SavePC() {	
		

		if (filewlop.size() <= 2) {
			cout << "normal file name is empty!" << endl;
		}
		else {
			ofstream fout(filewlop, ios::app);
			fout << pointResampleWlop.size() << endl;
			for (int i = 0; i < pointResampleWlop.size(); i++) {
				fout << pointResampleWlop[i][0] << " " << pointResampleWlop[i][1] << " " << pointResampleWlop[i][2] << endl;
			}
			fout << endl;
			fout.close();
		}
		if (filegird.size() <= 2) {
			cout << "normal file name is empty!" << endl;
		}
		else {
			ofstream fout1(filegird, ios::app);
			fout1 << pointResampleGird.size() << endl;
			for (int i = 0; i < pointResampleGird.size(); i++) {
				fout1 << pointResampleGird[i][0] << " " << pointResampleGird[i][1] << " " << pointResampleGird[i][2] << endl;
			}
			fout1 << endl;
			fout1.close();
		}	
	}  

};
