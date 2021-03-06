/*********************************************************************************

				              Normal Update Method


							 Updating in 2020/11/17

							   By Dr. Chenlei Lv

			The functions includes:
			1. Load a point cloud and tranguation from mesh reconstrution
			2. Update normals for points and output 			

*********************************************************************************/

#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h> 
#include "GLM/glm.h"
#include "GLM/glmVector.h"
using namespace std;

class NormalUpdate {

public:

	vector<vector<double>> NormalUpdate_Return(GLMmodel* pModel){

		vector<vector<double>> normalVector;	
		if (pModel->numfacetnorms <= 2) {
			glmFacetNormals(pModel);
		}
		//achieve the trangular for different points.
		vector<vector<int>> neiborTrans;//neibor tragular index
		vector<vector<int>> neiborTrans_index;//neibor tragular index
		vector<vector<double>> neiborTrans_Norm;//neibor trangular norm
		vector<vector<double>> neiborTrans_Area;//bneibor tragular area

		//add first item to regular data of norm and area
		vector<double> empty_int;
		vector<double> empty_double;
		neiborTrans_Norm.push_back(empty_int);
		neiborTrans_Area.push_back(empty_double);

		for (int i = 0; i <= pModel->numvertices; i++) {
			vector<int> t_i;
			neiborTrans.push_back(t_i);
			neiborTrans_index.push_back(t_i);
		}

		for (int i = 0; i < pModel->numtriangles; i++) {
			int b1 = pModel->triangles[i].vindices[0];
			int b2 = pModel->triangles[i].vindices[1];
			int b3 = pModel->triangles[i].vindices[2];
			neiborTrans[b1].push_back(b1);
			neiborTrans[b1].push_back(b2);
			neiborTrans[b1].push_back(b3);
			neiborTrans_index[b1].push_back(i);
			neiborTrans[b2].push_back(b1);
			neiborTrans[b2].push_back(b2);
			neiborTrans[b2].push_back(b3);
			neiborTrans_index[b2].push_back(i);
			neiborTrans[b3].push_back(b1);
			neiborTrans[b3].push_back(b2);
			neiborTrans[b3].push_back(b3);
			neiborTrans_index[b3].push_back(i);
		}

		for (int i = 1; i < neiborTrans.size(); i++) {
			vector<int> tragular_index = neiborTrans[i];
			vector<int> tragular_index_index = neiborTrans_index[i];
			vector<double> norm_i;
			vector<double> area_i;
			for (int j = 0; j < tragular_index.size() / 3; j++) {
				int b1 = tragular_index[3 * j];
				int b2 = tragular_index[3 * j + 1];
				int b3 = tragular_index[3 * j + 2];
				int trans_i = tragular_index_index[j];
				double xb1 = pModel->vertices[3 * b1];
				double yb1 = pModel->vertices[3 * b1 + 1];
				double zb1 = pModel->vertices[3 * b1 + 2];
				double xb2 = pModel->vertices[3 * b2];
				double yb2 = pModel->vertices[3 * b2 + 1];
				double zb2 = pModel->vertices[3 * b2 + 2];
				double xb3 = pModel->vertices[3 * b3];
				double yb3 = pModel->vertices[3 * b3 + 1];
				double zb3 = pModel->vertices[3 * b3 + 2];
				double v12x = xb2 - xb1;//ax
				double v12y = yb2 - yb1;//ay
				double v12z = zb2 - zb1;//az
				double v23x = xb3 - xb2;//bx
				double v23y = yb3 - yb2;//by
				double v23z = zb3 - zb2;//bz
				double v31x = xb1 - xb3;
				double v31y = yb1 - yb3;
				double v31z = zb1 - zb3;
				double nx = v12y * v23z - v12z * v23y;
				double ny = v12z * v23x - v12x * v23z;
				double nz = v12x * v23y - v12y * v23x;				
				double unitN = sqrt(nx * nx + ny * ny + nz * nz);
				nx = nx / unitN;
				ny = ny / unitN;
				nz = nz / unitN;
				norm_i.push_back(nx);
				norm_i.push_back(ny);
				norm_i.push_back(nz);
				double v12_length = sqrt(v12x * v12x + v12y * v12y + v12z * v12z);
				double v23_length = sqrt(v23x * v23x + v23y * v23y + v23z * v23z);
				double v31_length = sqrt(v31x * v31x + v31y * v31y + v31z * v31z);
				double p = (v12_length + v23_length + v31_length) / 2;
				double area_ij = sqrt(p * (p - v12_length) * (p - v23_length) * (p - v31_length));
				area_i.push_back(area_ij);
			}

			neiborTrans_Norm.push_back(norm_i);
			neiborTrans_Area.push_back(area_i);
		}

		for (int i = 1; i <= pModel->numvertices; i++) {
			vector<double> norm_vi;
			norm_vi.push_back(0);
			norm_vi.push_back(0);
			norm_vi.push_back(0);
			vector<double> neiborTrans_Norm_i = neiborTrans_Norm[i];
			vector<double> neiborTrans_Area_i = neiborTrans_Area[i];
			double sum_area_i = 0;
			if (neiborTrans_Norm_i.size() == 0 ||
				neiborTrans_Area_i.size() == 0) {
				vector<double> empty_i;
				empty_i.push_back(0);
				empty_i.push_back(0);
				empty_i.push_back(0);
				normalVector.push_back(empty_i);
			}
			else {
				for (int j = 0; j < neiborTrans_Area_i.size(); j++) {
					sum_area_i = sum_area_i + neiborTrans_Area_i[j];
				}
				for (int j = 0; j < neiborTrans_Norm_i.size() / 3; j++) {
					double x_n = neiborTrans_Norm_i[3 * j];
					double y_n = neiborTrans_Norm_i[3 * j + 1];
					double z_n = neiborTrans_Norm_i[3 * j + 2];
					double weight_j = 1 / (double)neiborTrans_Area_i.size();					
					norm_vi[0] = norm_vi[0] + x_n * weight_j;
					norm_vi[1] = norm_vi[1] + y_n * weight_j;
					norm_vi[2] = norm_vi[2] + z_n * weight_j;
				}
				double n_length = sqrt(norm_vi[0] * norm_vi[0] + norm_vi[1] * norm_vi[1] + norm_vi[2] * norm_vi[2]);
				norm_vi[0] = norm_vi[0] / n_length;
				norm_vi[1] = norm_vi[1] / n_length;
				norm_vi[2] = norm_vi[2] / n_length;
				normalVector.push_back(norm_vi);
			}
		}
		return normalVector;	
	};

	vector<vector<double>> NormalUpdate_Return(vector<vector<double>> pointSet, vector<vector<int>> faceInfor) {
		
		vector<vector<double>> normalVector;
		vector<vector<int>> neighbor;
		neighbor.resize(pointSet.size());
		for (int i = 0; i < faceInfor.size(); i++) {
			int b1 = faceInfor[i][0];
			int b2 = faceInfor[i][1];
			int b3 = faceInfor[i][2];
			neighbor[b1].push_back(b2);
			neighbor[b1].push_back(b3);
			neighbor[b2].push_back(b3);
			neighbor[b2].push_back(b1);
			neighbor[b3].push_back(b1);
			neighbor[b3].push_back(b2);
		}

		//achieve the trangular for different points.
		
		vector<vector<double>> neiborTrans_Norm;//neibor trangular norm
		vector<vector<double>> neiborTrans_Area;//bneibor tragular area

		//add first item to regular data of norm and area
		vector<double> empty_int;
		vector<double> empty_double;
		neiborTrans_Norm.push_back(empty_int);
		neiborTrans_Area.push_back(empty_double);		

		for (int i = 0; i < neighbor.size(); i++) {
			vector<int> tragular_index = neighbor[i];			
			vector<double> norm_i;
			vector<double> area_i;
			for (int j = 0; j < tragular_index.size() / 3; j++) {
				int b1 = tragular_index[3 * j];
				int b2 = tragular_index[3 * j + 1];
				int b3 = tragular_index[3 * j + 2];				
				double xb1 = pointSet[b1][0];
				double yb1 = pointSet[b1][1];
				double zb1 = pointSet[b1][2];
				double xb2 = pointSet[b2][0];
				double yb2 = pointSet[b2][1];
				double zb2 = pointSet[b2][2];
				double xb3 = pointSet[b3][0];
				double yb3 = pointSet[b3][1];
				double zb3 = pointSet[b3][2];
				double v12x = xb2 - xb1;//ax
				double v12y = yb2 - yb1;//ay
				double v12z = zb2 - zb1;//az
				double v23x = xb3 - xb2;//bx
				double v23y = yb3 - yb2;//by
				double v23z = zb3 - zb2;//bz
				double v31x = xb1 - xb3;
				double v31y = yb1 - yb3;
				double v31z = zb1 - zb3;
				double nx = v12y * v23z - v12z * v23y;
				double ny = v12z * v23x - v12x * v23z;
				double nz = v12x * v23y - v12y * v23x;				
				double unitN = sqrt(nx * nx + ny * ny + nz * nz);
				nx = nx / unitN;
				ny = ny / unitN;
				nz = nz / unitN;
				norm_i.push_back(nx);
				norm_i.push_back(ny);
				norm_i.push_back(nz);
				double v12_length = sqrt(v12x * v12x + v12y * v12y + v12z * v12z);
				double v23_length = sqrt(v23x * v23x + v23y * v23y + v23z * v23z);
				double v31_length = sqrt(v31x * v31x + v31y * v31y + v31z * v31z);
				double p = (v12_length + v23_length + v31_length) / 2;
				double area_ij = sqrt(p * (p - v12_length) * (p - v23_length) * (p - v31_length));
				area_i.push_back(area_ij);
			}

			neiborTrans_Norm.push_back(norm_i);
			neiborTrans_Area.push_back(area_i);
		}

		for (int i = 0; i < pointSet.size(); i++) {
			vector<double> norm_vi;
			norm_vi.push_back(0);
			norm_vi.push_back(0);
			norm_vi.push_back(0);
			vector<double> neiborTrans_Norm_i = neiborTrans_Norm[i];
			vector<double> neiborTrans_Area_i = neiborTrans_Area[i];
			double sum_area_i = 0;
			if (neiborTrans_Norm_i.size() == 0 ||
				neiborTrans_Area_i.size() == 0) {
				vector<double> empty_i;
				empty_i.push_back(0);
				empty_i.push_back(0);
				empty_i.push_back(0);
				normalVector.push_back(empty_i);
			}
			else {
				for (int j = 0; j < neiborTrans_Area_i.size(); j++) {
					sum_area_i = sum_area_i + neiborTrans_Area_i[j];
				}
				for (int j = 0; j < neiborTrans_Norm_i.size() / 3; j++) {
					double x_n = neiborTrans_Norm_i[3 * j];
					double y_n = neiborTrans_Norm_i[3 * j + 1];
					double z_n = neiborTrans_Norm_i[3 * j + 2];
					double weight_j = 1 / (double)neiborTrans_Area_i.size();
					norm_vi[0] = norm_vi[0] + x_n * weight_j;
					norm_vi[1] = norm_vi[1] + y_n * weight_j;
					norm_vi[2] = norm_vi[2] + z_n * weight_j;
				}
				double n_length = sqrt(norm_vi[0] * norm_vi[0] + norm_vi[1] * norm_vi[1] + norm_vi[2] * norm_vi[2]);
				norm_vi[0] = norm_vi[0] / n_length;
				norm_vi[1] = norm_vi[1] / n_length;
				norm_vi[2] = norm_vi[2] / n_length;
				normalVector.push_back(norm_vi);
			}
		}
		return normalVector;
	
	}
};