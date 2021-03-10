/*********************************************************************************

			 Main View for Point Cloud Shape Registration

						Updating in 2021/03/05

						   By Dr. Chenlei Lv

			The functions includes:
			1. Shape Registration;
			2. Show Result;


*********************************************************************************/
/*
#pragma region Include
#pragma once
#include "View.h"
#include <stdio.h>
#include <stdlib.h>
#include <GL/freeglut.h>
#include "FileProcess/LoadFileDlg.h"
#include "trackball.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "pointProcessPipeline.hpp"
#include "pointPipeline.hpp"
#include "denoising.hpp"
#include "pointCloudMeasure.hpp"
#include "Method_AIVS_SimPro.hpp"
#include "Method_Octree.hpp"
#include "initRegistration.hpp"
#include "transferPC.hpp"
#include "KSS_ICP.hpp"
#include "registrationMeasure.hpp"
using namespace std;
#pragma endregion

vector<vector<double>> Load_PLY(string FileName) {

	CPLYLoader plyLoader;
	char* p = new char[strlen(FileName.c_str()) + 1];
	strcpy(p, FileName.c_str());
	plyLoader.LoadModel(p);

	vector<vector<double>> points = plyLoader.points;
	return points;
}

void save_PointCloud(vector<vector<double>> pointCloud, string Path) {

	ofstream fout(Path, ios::app);
	fout << pointCloud.size() << endl;
	for (int i = 0; i < pointCloud.size(); i++) {
		fout << pointCloud[i][0] << " " << pointCloud[i][1] << " " << pointCloud[i][2] << endl;
	}
	fout << endl;
	fout.close();

}

vector<vector<double>> loadPoints(string filePath) {

	vector<vector<double>> normalVector;

	ifstream fin(filePath);
	if (fin)
	{
		int numSum;
		fin >> numSum;
		for (int i = 0; i < numSum; i++) {
			double x_i;
			double y_i;
			double z_i;
			fin >> x_i >> y_i >> z_i;
			vector<double> p_n_i;
			p_n_i.push_back(x_i);
			p_n_i.push_back(y_i);
			p_n_i.push_back(z_i);
			normalVector.push_back(p_n_i);
		}
		fin.close();
		return normalVector;
	}
	else {
		fin.close();
		return normalVector;
	}


}

vector<vector<double>> centerTransfer(vector<vector<double>> p) {

	double xc = 0;
	double yc = 0;
	double zc = 0;

	for (int i = 0; i < p.size(); i++) {

		xc = xc + p[i][0];
		yc = yc + p[i][1];
		zc = zc + p[i][2];

	}

	xc = xc / p.size();
	yc = yc / p.size();
	zc = zc / p.size();

	for (int i = 0; i < p.size(); i++) {
		p[i][0] = p[i][0] - xc;
		p[i][1] = p[i][1] - yc;
		p[i][2] = p[i][2] - zc;
	}

	return p;


}

int main(int argc, char* argv[])
{
	string objName1 = "_2000";
	string objName2 = "_20000";
	string objName3 = "_50000";
	string objName4 = "_100000";
	vector<string> list;
	list.push_back(objName1);
	list.push_back(objName2);
	list.push_back(objName3);
	list.push_back(objName4);
	vector<double> timeRecordV;
	vector<vector<double>> measureR;

	for (int i = 0; i < list.size(); i++) {
		clock_t t1 = clock();
		string objName = list[i];
		string fileSource = "E://chen_database//_Registration//Time//" + objName + "source.ply";
		string fileTarget = "E://chen_database//_Registration//Time//" + objName + "target.ply";
		vector<vector<double>> pointSource = Load_PLY(fileSource);
		vector<vector<double>> pointTarget = Load_PLY(fileTarget);
		vector<vector<double>> pointAlign;
		std::cout << "load ply finished." << endl;
		std::cout << "registration runing." << endl;

		KSSICP ki;
		ki.KSSICP_init(pointSource, pointTarget, 6);
		ki.KSSICP_Registration(1000);
		pointAlign = ki.pointAlign;
		clock_t t2 = clock();
		double timeRecord_i = (t2 - t1) / 1000.0;
		timeRecordV.push_back(timeRecord_i);

		std::cout << "registration finished." << endl;
		std::cout << "Measurement:" << endl;
		
		PCR_QM pq;
		pq.PCR_QM_init(pointAlign, pointTarget);
		vector<double> measure_i = pq.PCR_QM_ReturnResult();
		std::cout << "Registration Measure" << ":" << "MSE: "
			<< measure_i[0] << " RMSE: " << measure_i[1] << " MAE: " << measure_i[2] << endl;
		measureR.push_back(measure_i);
	}	

	for (int i = 0; i < list.size(); i++) {

		std::cout << list[i] << ":" << "time: " << timeRecordV[i] << endl;

	}

	for (int i = 0; i < list.size(); i++) {

		std::cout << list[i] << ":" << "MSE: " << measureR[i][0] << " RMSE: " << measureR[i][1] << " MAE: " << measureR[i][2] << endl;

	}

}
*/





































