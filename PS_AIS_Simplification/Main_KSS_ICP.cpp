/*********************************************************************************

			 Main View for Point Cloud Shape Registration

						Updating in 2021/03/05

						   By Dr. Chenlei Lv

			The functions includes:
			1. Shape Registration;
			2. Show Result;


*********************************************************************************/

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
#include "pointPipeline.hpp"
#include "pointCloudMeasure.hpp"
#include "Method_AIVS_SimPro.hpp"
#include "Method_Octree.hpp"
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

int main(int argc, char* argv[])
{
	std::cout << "start!" << endl;
	std::cout << "load ply:" << endl;	
	
	//source point cloud ply
	string fileSource = "E://chen_database//_Registration//_MiddleResult//centuarPart.ply";
	//target point cloud ply
	string fileTarget = "E://chen_database//_Registration//_MiddleResult//centuar.ply";
	//strore registration result .xyz
	string fileSaveSource = "E://chen_database//_Registration//_MiddleResult//Registration.xyz";
	
	vector<vector<double>> pointSource = Load_PLY(fileSource);
	vector<vector<double>> pointTarget = Load_PLY(fileTarget);
	vector<vector<double>> pointAlign;
	std::cout << "load ply finished." << endl;
	std::cout << "registration runing." << endl;

	KSSICP ki;
	ki.KSSICP_init(pointSource, pointTarget, 8);	
	ki.KSSICP_Registration(1000);
	pointAlign = ki.pointAlign;

	std::cout << "registration finished." << endl;
	std::cout << "Measurement:" << endl;
	PCR_QM pq;
	pq.PCR_QM_init(pointAlign, pointTarget);
	vector<double> measure_i = pq.PCR_QM_ReturnResult();	
	std::cout << "Registration Measure" << ":" << "MSE: " 
		<< measure_i[0] << " RMSE: " << measure_i[1] << " MAE: " << measure_i[2] << endl;
	save_PointCloud(pointAlign, fileSaveSource);

}






































