/*********************************************************************************

					 Mesh Quality Mesurement Object

						Updating in 2020/08/13

						   By Dr. Chenlei Lv

			The functions includes:
			1. Achieving the angle statical infor for view (in Python);
			


**********************************************************************************/

#pragma once
#include <vector>
#include <iostream>
#include <fstream>
#include <GL/freeglut.h>
#include "GLM/glm.h"
#include "GLM/glmVector.h"
using namespace std;


class MeshMeasure {

public:	

	vector<double> MeshMeasure_CountAngle(GLMmodel * pModel) {

		vector<double> angleList;

		//1. compute point index trangulation
		for (int i = 0; i < pModel->numtriangles; i++) {
			int b1 = pModel->triangles[i].vindices[0];
			int b2 = pModel->triangles[i].vindices[1];
			int b3 = pModel->triangles[i].vindices[2];
			double xn = pModel->facetnorms[3 * (i + 1)];
			double yn = pModel->facetnorms[3 * (i + 1) + 1];
			double zn = pModel->facetnorms[3 * (i + 1) + 2];
			vector<double> pb1(3);
			pb1[0] = pModel->vertices[3 * b1];
			pb1[1] = pModel->vertices[3 * b1 + 1];
			pb1[2] = pModel->vertices[3 * b1 + 2];
			vector<double> pb2(3);
			pb2[0] = pModel->vertices[3 * b2];
			pb2[1] = pModel->vertices[3 * b2 + 1];
			pb2[2] = pModel->vertices[3 * b2 + 2];
			vector<double> pb3(3);
			pb3[0] = pModel->vertices[3 * b3];
			pb3[1] = pModel->vertices[3 * b3 + 1];
			pb3[2] = pModel->vertices[3 * b3 + 2];

			vector<double> v11(3);
			v11[0] = pb2[0] - pb1[0];
			v11[1] = pb2[1] - pb1[1];
			v11[2] = pb2[2] - pb1[2];

			vector<double> v12(3);
			v12[0] = pb3[0] - pb1[0];
			v12[1] = pb3[1] - pb1[1];
			v12[2] = pb3[2] - pb1[2];

			vector<double> v21(3);
			v21[0] = pb1[0] - pb2[0];
			v21[1] = pb1[1] - pb2[1];
			v21[2] = pb1[2] - pb2[2];

			vector<double> v22(3);
			v22[0] = pb3[0] - pb2[0];
			v22[1] = pb3[1] - pb2[1];
			v22[2] = pb3[2] - pb2[2];

			vector<double> v31(3);
			v31[0] = pb1[0] - pb3[0];
			v31[1] = pb1[1] - pb3[1];
			v31[2] = pb1[2] - pb3[2];

			vector<double> v32(3);
			v32[0] = pb2[0] - pb3[0];
			v32[1] = pb2[1] - pb3[1];
			v32[2] = pb2[2] - pb3[2];

			double angle1 = MeshMeasure_Angle(v11, v12);
			double angle2 = MeshMeasure_Angle(v21, v22);
			double angle3 = MeshMeasure_Angle(v31, v32);
			angleList.push_back(angle1);
			angleList.push_back(angle2);
			angleList.push_back(angle3);
		}

		return angleList;		
	}
	
	void MeshMeasure_QualityMeasure(GLMmodel* pModel) {

		double angleSmallest = 1000;
		double angleVLSum = 0;
		double Q_t_min = 10000;
		double Q_tVLSum = 0 ;
		//number triangles
		int tSum = 0;
		for (int i = 0; i < pModel->numtriangles; i++) {
			int b1 = pModel->triangles[i].vindices[0];
			int b2 = pModel->triangles[i].vindices[1];
			int b3 = pModel->triangles[i].vindices[2];
			double xn = pModel->facetnorms[3 * (i + 1)];
			double yn = pModel->facetnorms[3 * (i + 1) + 1];
			double zn = pModel->facetnorms[3 * (i + 1) + 2];
			vector<double> pb1(3);
			pb1[0] = pModel->vertices[3 * b1];
			pb1[1] = pModel->vertices[3 * b1 + 1];
			pb1[2] = pModel->vertices[3 * b1 + 2];
			vector<double> pb2(3);
			pb2[0] = pModel->vertices[3 * b2];
			pb2[1] = pModel->vertices[3 * b2 + 1];
			pb2[2] = pModel->vertices[3 * b2 + 2];
			vector<double> pb3(3);
			pb3[0] = pModel->vertices[3 * b3];
			pb3[1] = pModel->vertices[3 * b3 + 1];
			pb3[2] = pModel->vertices[3 * b3 + 2];

			vector<double> v11(3);
			v11[0] = pb2[0] - pb1[0];
			v11[1] = pb2[1] - pb1[1];
			v11[2] = pb2[2] - pb1[2];

			vector<double> v12(3);
			v12[0] = pb3[0] - pb1[0];
			v12[1] = pb3[1] - pb1[1];
			v12[2] = pb3[2] - pb1[2];

			vector<double> v21(3);
			v21[0] = pb1[0] - pb2[0];
			v21[1] = pb1[1] - pb2[1];
			v21[2] = pb1[2] - pb2[2];

			vector<double> v22(3);
			v22[0] = pb3[0] - pb2[0];
			v22[1] = pb3[1] - pb2[1];
			v22[2] = pb3[2] - pb2[2];

			vector<double> v31(3);
			v31[0] = pb1[0] - pb3[0];
			v31[1] = pb1[1] - pb3[1];
			v31[2] = pb1[2] - pb3[2];

			vector<double> v32(3);
			v32[0] = pb2[0] - pb3[0];
			v32[1] = pb2[1] - pb3[1];
			v32[2] = pb2[2] - pb3[2];

			double angle1 = MeshMeasure_Angle(v11, v12);
			double angle2 = MeshMeasure_Angle(v21, v22);
			double angle3 = MeshMeasure_Angle(v31, v32);
			angle1 = angle1 / 3.1415 * 180;
			angle2 = angle2 / 3.1415 * 180;
			angle3 = angle3 / 3.1415 * 180;
			double angleS = angle1;
			if (angle2 < angleS) {
				angleS = angle2;			
			}
			if (angle3 < angleS) {
				angleS = angle3;			
			}

			if (angleS<9999 && angleS>-9999) {
				tSum++;
				angleVLSum = angleVLSum + angleS;
				if (angleS < angleSmallest) {
					angleSmallest = angleS;
				}
				double a = sqrt(v11[0] * v11[0] + v11[1] * v11[1] + v11[2] * v11[2]);
				double b = sqrt(v12[0] * v12[0] + v12[1] * v12[1] + v12[2] * v12[2]);
				double c = sqrt(v22[0] * v22[0] + v22[1] * v22[1] + v22[2] * v22[2]);
				double p = (a + b + c) / 2;
				double Area = sqrt(p * (p - a) * (p - b) * (p - c));
				double lb = a;
				//double inRadius = Area / p;
				if (lb < b) {
					lb = b;
				}
				if (lb < c) {
					lb = c;
				}
				double Q_t = 6 / sqrt(3) * Area / (p * lb);
				if (Q_t < Q_t_min) {
					Q_t_min = Q_t;
				}
				Q_tVLSum = Q_tVLSum + Q_t;
			}			
			//Heron's formula
			//p = (a + b + c) / 2
			//S = sqrt[p(p - a)(p - b)(p - c)]
			
		}
		//mean angleVLSum and Q_tVLSum	
		angleVLSum = angleVLSum / tSum;
		Q_tVLSum = Q_tVLSum / tSum;
		cout <<"Mesh Measure Quality Result:" << endl;
		cout << "theta_min = " << angleSmallest << endl;
		cout << "theta_(min,ave) = " << angleVLSum << endl;
		cout << "Q_t_min = " << Q_t_min << endl;
		cout << "Q_t_ave = " << Q_tVLSum << endl;	
	}

	void MeshMeasure_mesh_Quality_Save(string fileName, vector<double> angle) {

		ofstream f1(fileName, ios::app);

		for (int i = 0; i < angle.size(); i++) {
			if (angle[i]<9999&& angle[i]>-9999) {
				f1 << angle[i] << endl;
			
			}
			else {
				f1 << 0 << endl;			
			}
			

		}

		f1.close();
	}
	
	vector<double>  MeshMeasure_CountMinAngleForEachPoint(GLMmodel* pModel) {

		vector<vector<int>> point_Neighbor(pModel->numvertices);
		vector<vector<double>> point_data(pModel->numvertices);
 		vector<double> point_MinAngle(pModel->numvertices);

		for (int i = 1; i <= pModel->numvertices; i++) {

			point_data[i - 1].push_back(pModel->vertices[3 * i]);
			point_data[i - 1].push_back(pModel->vertices[3 * i + 1]);
			point_data[i - 1].push_back(pModel->vertices[3 * i + 2]);
		
		}

		for (int i = 0; i < pModel->numtriangles; i++) {

			int b1 = pModel->triangles[i].vindices[0] - 1;
			int b2 = pModel->triangles[i].vindices[1] - 1;
			int b3 = pModel->triangles[i].vindices[2] - 1;

			point_Neighbor[b1].push_back(b2);
			point_Neighbor[b1].push_back(b3);
			point_Neighbor[b2].push_back(b3);
			point_Neighbor[b2].push_back(b1);
			point_Neighbor[b3].push_back(b1);
			point_Neighbor[b3].push_back(b2);

		}

		for (int i = 0; i < point_Neighbor.size(); i++) {

			vector<double> pb1(3);
			pb1[0] = point_data[i][0];
			pb1[1] = point_data[i][1];
			pb1[2] = point_data[i][2];

			double anglePiMin = 9999;

			for (int j = 0; j < point_Neighbor[i].size()/2; j++) {
				
				int pni2 = point_Neighbor[i][2 * j];
				int pni3 = point_Neighbor[i][2 * j + 1];

				vector<double> pb2(3);
				pb2[0] = point_data[pni2][0];
				pb2[1] = point_data[pni2][1];
				pb2[2] = point_data[pni2][2];
				vector<double> pb3(3);
				pb3[0] = point_data[pni3][0];
				pb3[1] = point_data[pni3][1];
				pb3[2] = point_data[pni3][2];

				vector<double> v11(3);
				v11[0] = pb2[0] - pb1[0];
				v11[1] = pb2[1] - pb1[1];
				v11[2] = pb2[2] - pb1[2];

				vector<double> v12(3);
				v12[0] = pb3[0] - pb1[0];
				v12[1] = pb3[1] - pb1[1];
				v12[2] = pb3[2] - pb1[2];			
			
				double anglePi = MeshMeasure_Angle(v11, v12);
				if (anglePi < anglePiMin) {
					anglePiMin = anglePi;				
				}				
			}

			anglePiMin = abs(anglePiMin - 1);
			point_MinAngle[i] = anglePiMin;
		}

		return point_MinAngle;
	
	}

private:
	
	double MeshMeasure_Angle(vector<double> v1, vector<double> v2) {

		double length1 = sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2]);
		double length2 = sqrt(v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2]);
		v1[0] = v1[0] / length1;
		v1[1] = v1[1] / length1;
		v1[2] = v1[2] / length1;

		v2[0] = v2[0] / length2;
		v2[1] = v2[1] / length2;
		v2[2] = v2[2] / length2;

		double aij = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
		if (aij > 1) {
			aij = 1;
		}
		if (aij < -1) {
			aij = -1;
		}
		double anglej = abs(acos(aij));
		return anglej;
	
	}

};
