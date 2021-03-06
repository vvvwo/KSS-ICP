/*********************************************************************************

			 AIVS-based simplification with curvature sensitive sampling
			 and shape feature keeping			


						Updating in 2020/09/09

						   By Dr. Chenlei Lv

			The functions includes:
			Isotropic simplification.
			Anisotropic simplification
			(Curvature sensitive sampling and shape feature keeping)

			Function:
			AIVS_simplification(int pointNum); //achieve isotropic simplification 
			result with pointNum points.
			AIVS_AIVS_simplification_Curvature(int pointNum, vector<double> prate)
			//achieve isotropic simplification 
			result with pointNum points.
			vector<int> AIVS_Pro_ReturnEdges(vector<vector<double>> seedPoints)
			//label the edge points from simplification result 

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
#include <time.h>
#include "ballRegionCompute.hpp"
#include "Method_Octree.hpp"

using namespace std;

class AIVS_Simplification_Pro {

public:

	BallRegion br;

private:

	string fileName;
	//string fileNameNormal;
	//vector<vector<double>> pointD;
	//vector<vector<double>> pointV;
	double shapeEdgeSensitive = 0.02;//For shape edged extraction
	vector<vector<int>> boxIndexNumberCu;//parallel structure
	vector<int> labelG; //record simplification sample;	
	vector<int> borderFeature;//sharp feature points
	vector<double> pointNormalValue;//curvature sensative
	vector<double> pointShapeValue;//curvature sensative
	vector<int> pointClassification;//classification number
	vector<int> pointCSum;//classification sum
	vector<double> pointCRate;//classification rate
	//vector<vector<double>> boxRange;//the max and min coordinate of the model
	int iter = 10;

public:

	void AIVS_Pro_init(BallRegion brInput, string fileNameInput) {//fileNameInput is the global path which has removed the tail:Remesh\Bunny
		
		cout << "AIVS_Pro_init Start!" << endl;
		fileName = fileNameInput;
		//init BallRegion
		br = brInput;

		//init parallel computation steps for different boxes 
		AIVS_initBoxIndexNumber();

		//init curvature sensative
		cout << "AIVS_Pro_init Curvature estimaition..." << endl;
		//AIVS_NormalValue();		
		//AIVS_NormalValue_Meshbased();
		AIVS_NormalValue();
		//AIVS_NormalValue_Meshbased_HaveObj();
		//init edge extraction
		//cout << "AIVS_Pro_init Edge points estimaition..." << endl;
		//AIVS_BorderFeature();

		cout << "AIVS_Pro_init Finished!" << endl;
	
	}
	
	vector<vector<double>> AIVS_simplification(int pointNum) {
		clock_t t1 = clock();
		//achieve the point rate
		double rate = (double)pointNum / (double)br.pointCloudData.size();

		//1. 8 box list (vector<vector<int>>)
		boxIndexNumberCu;

		//2. box center (vector<vector<double>>)
		vector<vector<double>> boxcenter = br.squareBoxesCReal;
		vector<int> boxcenterIndex = br.squareBoxesCenter;
		//3. box scale (double)
		double boxScale = br.unitSize;

		//4. neibor box link (vector<vector<int>>)
		vector<vector<int>> neiborBox(br.squareBoxes.size());
		for (int i = 1; i < br.squareBoxes.size(); i++) {
			vector<int> neiborBox_i = br.BallRegion_ReturnNeiborBox_Box(i);
			neiborBox[i] = neiborBox_i;
		}

		vector<double> prate;
		prate.push_back(3);
		prate.push_back(7);
		AIVS_Classification_Edge(prate);

		//5. the simplification number for each box
		int boxscale = 4;
		//vector<int> boxSimiNumer = AIVS_BoxSimplification(pointNum, boxscale);//box based point density estimation
		vector<int> boxSimiNumer = AIVS_BoxSimplification_Points(pointNum);
		//6. the points infor in each box
		br.squareBoxes;

		//7. global label (vector<int>)		
		labelG.resize(br.pointCloudData.size(), 1);

		//8. point cloud data (vector<vector<double>>)
		br.pointCloudData;

		//Voroni_OpenMP Start!

		vector<vector<int>> SimResult = AIVS_Voroni_OpenMP_KNN(//AIVS_Voroni_OpenMP( ; AIVS_Voroni_Cuda(;AIVS_Voroni_OpenMP_KNN.
			boxIndexNumberCu,
			boxcenter,
			boxcenterIndex,
			boxScale,
			neiborBox,
			boxSimiNumer,
			br.squareBoxes,
			labelG,
			br.pointCloudData
		);//store the data.

		//Final Step, to achieve the accurate points number.
		vector<vector<double>> finalResult = AIVS_AccurateCut_Optimization(SimResult, pointNum);//final simplification result.
		

		clock_t t2 = clock();
		std::cout << "AIVS_simplification running time:" << (t2 - t1) / 1000.0 << "s" << endl;
		return finalResult;
	}
	
	vector<int> AIVS_Pro_ReturnEdges(vector<vector<double>> seedPoints) {

		vector<int> edgeList;
		for (int i = 0; i < seedPoints.size(); i++) {

			std::vector<int> pointIdxNKNSearch(1);
			std::vector<float> pointNKNSquaredDistance(1);
			pcl::PointXYZ searchPoint;
			searchPoint.x = seedPoints[i][0];
			searchPoint.y = seedPoints[i][1];
			searchPoint.z = seedPoints[i][2];
			br.kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
			int pindex = pointIdxNKNSearch[0];
			int c = pointClassification[pindex];
			if (c == 1) {
				edgeList.push_back(i);
			}		
		}
		return edgeList;	
	}

private:
	
	//For curvature estimation
	void AIVS_NormalValue() {//average angle estimation

		if (pointNormalValue.size() > 0) {
			pointNormalValue.clear();
		}
		pointNormalValue.resize(br.pointCloudData.size());

		clock_t t1 = clock();

		int K = 7;
#pragma omp parallel for
		for (int i = 0; i < br.pointCloudData.size(); i++) {
			if (i % 10000 == 0) {
				cout << (br.pointCloudData.size() - i) / 10000 << ",";
			}
			vector<double> pTemp = br.pointCloudData[i];
			vector<double> pTempN = br.pointNormal[i];
			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);
			pcl::PointXYZ searchPoint;
			searchPoint.x = pTemp[0];
			searchPoint.y = pTemp[1];
			searchPoint.z = pTemp[2];
			br.kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
			double suma = 0;
			for (int j = 1; j < pointIdxNKNSearch.size(); j++) {
				vector<double> ps = br.pointNormal[pointIdxNKNSearch[j]];
				double a1 = AIVS_Angle(ps, pTempN);
				if (suma < a1) {
					suma = a1;				
				}
				
			}
			//suma = suma / (double)(K - 1);
			pointNormalValue[i] = suma;
		}
		std::cout << endl;
		clock_t t2 = clock();
		std::cout << "AIVS_NormalValue running time:" << (t2 - t1) / 1000.0 << "s" << endl;
	}	

#pragma region Simplification
	vector<vector<int>> AIVS_Voroni_OpenMP_KNN(
		vector<vector<int>> boxList_1,//8 box list for parallel computation
		vector<vector<double>> boxCenter_2,//the center points for each box 
		vector<int> boxCenterIndex_2,//the center index point in the box
		double boxScale_3,//the scle of the box
		vector<vector<int>> boxNeibor_4,
		vector<int> boxSimNum_5,
		vector<vector<int>> boxPoints_6,//the point index in the box
		vector<int> labelG_7,
		vector<vector<double>> pointData_8
	) {
		//init simiT for simplification result store
		vector<vector<int>> simiT(boxPoints_6.size());//store the raw simplification result
		for (int i = 0; i < simiT.size(); i++) {
			vector<int> simiT_i(boxPoints_6[i].size(), -1);
			simiT[i] = simiT_i;
		}
		double searchBoxRadius = boxScale_3 * 3.0 / 4.0;
		for (int i = 0; i < boxList_1.size(); i++) {
			cout << "blockList:" << i << endl;
#pragma omp parallel for
			for (int j = 0; j < boxList_1[i].size(); j++) {
				int boxIndex = boxList_1[i][j];
				int simNum = boxSimNum_5[boxIndex];
				if (simNum == 0) {
					continue;
				}
				//int boxNeibirIndex = boxNeibor_4[boxIndex]
				vector<double> pCenter = boxCenter_2[boxIndex];
				//achieve center point infor
				vector<int> pointTemp = boxPoints_6[boxIndex];
				vector<int> labelTemp(pointTemp.size(), 1);

				//search the neibor points into the block
				bool addJ = true;
				for (int k = 0; k < boxNeibor_4[boxIndex].size(); k++) {
					vector<int> pN = boxPoints_6[boxNeibor_4[boxIndex][k]];// achieve the points from neibor box
					for (int l = 0; l < pN.size(); l++) {
						vector<double> pn_l = pointData_8[pN[l]];
						if (pn_l[0] <= pCenter[0] + searchBoxRadius && pn_l[0] >= pCenter[0] - searchBoxRadius
							&& pn_l[1] <= pCenter[1] + searchBoxRadius && pn_l[1] >= pCenter[1] - searchBoxRadius
							&& pn_l[2] <= pCenter[2] + searchBoxRadius && pn_l[2] >= pCenter[2] - searchBoxRadius
							&& labelG_7[pN[l]] == 0) {
							pointTemp.push_back(pN[l]);
							labelTemp.push_back(2);
							addJ = false;
						}
					}
				}
				if (addJ) {
					if (boxCenterIndex_2[boxIndex] >= -1 && boxCenterIndex_2[boxIndex] < pointTemp.size()) {
						labelTemp[boxCenterIndex_2[boxIndex]] = 0;
					}
				}
				//construct KD-tree;
				pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSeedlocal;
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
				cloud->width = pointTemp.size();
				cloud->height = 1;
				cloud->points.resize(cloud->width * cloud->height);
				// fills a PointCloud with random data
				for (int i = 0; i < pointTemp.size(); i++)
				{
					pcl::PointXYZ pxyz;
					cloud->points[i].x = pointData_8[pointTemp[i]][0];
					cloud->points[i].y = pointData_8[pointTemp[i]][1];
					cloud->points[i].z = pointData_8[pointTemp[i]][2];

				}
				kdtreeSeedlocal.setInputCloud(cloud);

				int K = pointTemp.size();
				//start Voroni: //1. pointTemp  2. labelTemp 3. simNum
				int samplieIndex = 0;//record the  
				vector<double> mindistance(pointTemp.size(), -1);
				for (int k = 0; k < mindistance.size(); k++) {
					if (labelTemp[k] == 0) {
						mindistance[k] = 0;
						simiT[boxIndex][samplieIndex] = pointTemp[k];
						labelG_7[pointTemp[k]] = 0;
						samplieIndex++;
					}
					else if (labelTemp[k] == 2) {
						mindistance[k] = 0;
					}
					else {
						double minTemp = 9999;
						//using kdtree to detect the minDistance
						vector<double> pTemp = pointData_8[pointTemp[k]];
						std::vector<int> pointIdxNKNSearch(K);
						std::vector<float> pointNKNSquaredDistance(K);
						pcl::PointXYZ searchPoint;
						searchPoint.x = pTemp[0];
						searchPoint.y = pTemp[1];
						searchPoint.z = pTemp[2];
						kdtreeSeedlocal.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
						for (int l = 0; l < pointIdxNKNSearch.size(); l++) {
							int indexMin = pointIdxNKNSearch[l];
							if (labelTemp[indexMin] == 0 || labelTemp[indexMin] == 2) {
								minTemp = sqrt(pointNKNSquaredDistance[l]);
								break;
							}
						}
						mindistance[k] = minTemp;
					}
				}
				while (1) {
					if (samplieIndex >= simNum) {
						break;
					}
					else {
						//select the point with the longest mindis
						int indexSelect = -1;
						double max_mindistance = 0;
						for (int k = 0; k < mindistance.size(); k++) {
							if (labelTemp[k] == 1 && mindistance[k] > max_mindistance) {
								indexSelect = k;
								max_mindistance = mindistance[k];
							}
						}
						if (indexSelect == -1) {
							break;
						}
						//achieve new simplification point.
						mindistance[indexSelect] = 0;
						labelG_7[pointTemp[indexSelect]] = 0;
						simiT[boxIndex][samplieIndex] = pointTemp[indexSelect];
						vector<double> pTemp = pointData_8[pointTemp[indexSelect]];
						samplieIndex++;

						//updata mindistance.
						std::vector<int> pointIdxNKNSearch(K);
						std::vector<float> pointNKNSquaredDistance(K);
						pcl::PointXYZ searchPoint;
						searchPoint.x = pTemp[0];
						searchPoint.y = pTemp[1];
						searchPoint.z = pTemp[2];
						kdtreeSeedlocal.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

						for (int k = 0; k < pointIdxNKNSearch.size(); k++) {
							int indexLabel = pointIdxNKNSearch[k];
							if (labelTemp[indexLabel] == 1) {
								double dis_Temp = sqrt(pointNKNSquaredDistance[k]);
								if (dis_Temp < mindistance[indexLabel]) {
									mindistance[indexLabel] = dis_Temp;
								}
							}
						}
					}
				}
				//End Voroni
			}//block end		
		}
		return simiT;
	}

	//User define simplification
	void AIVS_Pipeline_UserDefine_OpenMP(
		vector<vector<int>> boxList_1,//8 box list for parallel computation
		vector<vector<double>> boxCenter_2,//the center points for each box 
		vector<int> boxCenterIndex_2,//the center index point in the box
		double boxScale_3,//the scle of the box
		vector<vector<int>> boxNeibor_4,
		vector<vector<int>> boxSimNum_5,
		vector<vector<int>> boxPoints_6,//the point index in the box
		vector<int> labelG_7,
		vector<vector<double>> pointData_8
	) {

		//1. init simiT for simplification result store


		//2. set searching neighbor box
		double searchBoxRadius = boxScale_3 * 3.0 / 4.0;

		//3. simplification in different boxes
		for (int i = 0; i < boxList_1.size(); i++) {
			cout << "blockList:" << i << endl;

			//3.1 parallel computation

#pragma omp parallel for
			for (int j = 0; j < boxList_1[i].size(); j++) {

				//3.2 achieve necessary infor
				int boxIndex = boxList_1[i][j];//box index
				//cout << boxIndex << ",";
				vector<int> simNum = boxSimNum_5[boxIndex];
				if (simNum.size() == 0) {
					continue;
				}
				//compute sum
				int sumNumSum = 0;
				for (int k = 0; k < simNum.size(); k++) {
					sumNumSum = sumNumSum + simNum[k];
				}
				if (sumNumSum >= boxPoints_6[boxIndex].size()) {
					for (int j = 0; j < boxPoints_6[boxIndex].size(); j++) {
						labelG_7[boxPoints_6[boxIndex][j]] = 0;
					}
					continue;
				}
				//int boxNeibirIndex = boxNeibor_4[boxIndex]
				vector<double> pCenter = boxCenter_2[boxIndex];
				//achieve center point infor
				vector<int> pointTemp = boxPoints_6[boxIndex];
				vector<int> labelTemp(pointTemp.size(), 1);

				//3.3 search the neibor points into the block
				for (int k = 0; k < boxNeibor_4[boxIndex].size(); k++) {
					vector<int> pN = boxPoints_6[boxNeibor_4[boxIndex][k]];// achieve the points from neibor box
					for (int l = 0; l < pN.size(); l++) {
						vector<double> pn_l = pointData_8[pN[l]];
						if (pn_l[0] <= pCenter[0] + searchBoxRadius && pn_l[0] >= pCenter[0] - searchBoxRadius
							&& pn_l[1] <= pCenter[1] + searchBoxRadius && pn_l[1] >= pCenter[1] - searchBoxRadius
							&& pn_l[2] <= pCenter[2] + searchBoxRadius && pn_l[2] >= pCenter[2] - searchBoxRadius
							&& labelG_7[pN[l]] == 0) {
							pointTemp.push_back(pN[l]);
							labelTemp.push_back(2);
						}
					}
				}
				if (pointTemp.size() == boxPoints_6[boxIndex].size() && boxCenterIndex_2[boxIndex] >= -1
					&& boxCenterIndex_2[boxIndex] < pointTemp.size()) {
					labelTemp[boxCenterIndex_2[boxIndex]] = 0;
				}

				//start Voroni: //1. pointTemp  2. labelTemp 3. simNum
				for (int k = 0; k < pointCRate.size(); k++) {
					int classIndex = k;
					int classsimNum = simNum[k];
					vector<int> labelTempk = AIVS_Pipeline_UserDefine_OpenMP_Sim(pointTemp, labelTemp, classIndex, classsimNum);
					labelTemp.clear();
					labelTemp = labelTempk;
				}

				for (int i = 0; i < boxPoints_6[boxIndex].size(); i++) {
					int pIndexi = pointTemp[i];
					int pointSign = labelTemp[i];
					if (pointSign == 0 || pointSign == 3) {
						labelG_7[pIndexi] = 0;
					}
				}
				//End Voroni
			}//block end		
		}

		labelG.clear();
		labelG = labelG_7;

	}

	vector<int> AIVS_Pipeline_UserDefine_OpenMP_Sim(vector<int> pointTemp, vector<int> labelTemp, int classIndex, int simNum) {

		//simNum is the point number for simplification

		int samplieIndex = 0;//record the simplification number
		vector<double> mindistance(pointTemp.size(), -1);
		for (int k = 0; k < mindistance.size(); k++) {
			int classIndexk = pointClassification[pointTemp[k]];
			if (labelTemp[k] == 0) {//init sampling point
				mindistance[k] = 0;
				samplieIndex++;
			}
			else if (labelTemp[k] == 2 || labelTemp[k] == 3) {//2: neighbor point && 3: sampling point have been seleted
				mindistance[k] = 0;
			}
			else {

				if (classIndexk == classIndex) {
					double minTemp = 9999;
					vector<double> pTemp = br.pointCloudData[pointTemp[k]];
					for (int l = 0; l < mindistance.size(); l++) {
						if (labelTemp[l] == 0 || labelTemp[l] == 2 || labelTemp[l] == 3) {
							vector<double> pTemp_l = br.pointCloudData[pointTemp[l]];
							double dis_Temp = sqrt(
								(pTemp[0] - pTemp_l[0]) * (pTemp[0] - pTemp_l[0]) +
								(pTemp[1] - pTemp_l[1]) * (pTemp[1] - pTemp_l[1]) +
								(pTemp[2] - pTemp_l[2]) * (pTemp[2] - pTemp_l[2])
							);
							if (dis_Temp < minTemp) {
								minTemp = dis_Temp;
							}
						}
					}
					mindistance[k] = minTemp;
				}
				else {
					mindistance[k] = 0;
				}
			}
		}
		while (1) {
			if (samplieIndex >= simNum) {
				break;
			}
			else {
				//select the point with the longest mindis
				int indexSelect = -1;
				double max_mindistance = 0;
				for (int k = 0; k < mindistance.size(); k++) {
					int classIndexk = pointClassification[pointTemp[k]];
					if (labelTemp[k] == 1 && mindistance[k] > max_mindistance && classIndexk == classIndex) {
						indexSelect = k;
						max_mindistance = mindistance[k];
					}
				}
				if (indexSelect == -1) {
					break;
				}
				//achieve new simplification point.
				mindistance[indexSelect] = 0;
				labelTemp[indexSelect] = 3;
				vector<double> pTemp = br.pointCloudData[pointTemp[indexSelect]];
				samplieIndex++;

				//updata mindistance.
				for (int k = 0; k < mindistance.size(); k++) {
					int classIndexk = pointClassification[pointTemp[k]];
					if (labelTemp[k] == 1 && classIndexk == classIndex) {
						vector<double> pTemp_k = br.pointCloudData[pointTemp[k]];
						double dis_Temp = sqrt((pTemp_k[0] - pTemp[0]) * (pTemp_k[0] - pTemp[0]) +
							(pTemp_k[1] - pTemp[1]) * (pTemp_k[1] - pTemp[1]) +
							(pTemp_k[2] - pTemp[2]) * (pTemp_k[2] - pTemp[2]));
						if (dis_Temp < mindistance[k]) {
							mindistance[k] = dis_Temp;
						}
					}
				}
			}
		}
		return labelTemp;
	}

#pragma endregion

#pragma region Other Functions
	//Return a face normal
	vector<double> AIVS_NormalValue_Face(vector<double> p1, vector<double> p2, vector<double> p3) {

		vector<double> v1(3, 0);
		v1[0] = p2[0] - p1[0];
		v1[1] = p2[1] - p1[1];
		v1[2] = p2[2] - p1[2];

		vector<double> v2(3, 0);
		v2[0] = p3[0] - p2[0];
		v2[1] = p3[1] - p2[1];
		v2[2] = p3[2] - p2[2];

		vector<double> n(3, 0);
		n[0] = v2[2] * v1[1] - v2[1] * v1[2];
		n[1] = -v2[2] * v1[0] + v2[0] * v1[2];
		n[2] = v2[1] * v1[0] - v2[0] * v1[1];

		double length = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
		n[0] = n[0] / length;
		n[1] = n[1] / length;
		n[2] = n[2] / length;

		return n;

	}

	//Achieve parallel computation steps
	void AIVS_initBoxIndexNumber() {

		int boxNum = br.squareBoxes.size();
		vector<int> xyzNum = br.XYZNumber;

		int simin = 9999;
		int simax = 0;


		vector<vector<int>> box(8);
		for (int i = 1; i <= xyzNum[0]; i++) {
			for (int j = 1; j <= xyzNum[1]; j++) {
				for (int k = 1; k <= xyzNum[2]; k++) {
					int index_Num = i + xyzNum[0] * (j - 1) + (xyzNum[0] * xyzNum[1] * (k - 1));
					if (br.squareBoxes[index_Num].size() == 0) {
						continue;
					}
					//if (index_Num == 325) {
						//cout << "i:" << i << "j:" << j << "k:" << k << endl;
					//}
					//if (index_Num < simin) {
						//simin = index_Num;
					//}
					//if (index_Num > simax) {
						//simax = index_Num;
					//}
					if (i % 2 == 1 && j % 2 == 1 && k % 2 == 1) {
						box[0].push_back(index_Num);
					}
					else if (i % 2 == 0 && j % 2 == 1 && k % 2 == 1) {
						box[1].push_back(index_Num);
					}
					else if (i % 2 == 0 && j % 2 == 0 && k % 2 == 1) {
						box[2].push_back(index_Num);
					}
					else if (i % 2 == 1 && j % 2 == 0 && k % 2 == 1) {
						box[3].push_back(index_Num);
					}
					else if (i % 2 == 1 && j % 2 == 1 && k % 2 == 0) {
						box[4].push_back(index_Num);
					}
					else if (i % 2 == 0 && j % 2 == 1 && k % 2 == 0) {
						box[5].push_back(index_Num);
					}
					else if (i % 2 == 0 && j % 2 == 0 && k % 2 == 0) {
						box[6].push_back(index_Num);
					}
					else {
						box[7].push_back(index_Num);
					}
				}
			}
		}

		boxIndexNumberCu = box;

	}

	//Inner angle from two vectors
	double AIVS_Angle(vector<double> ps, vector<double> pt) {

		double value = ps[0] * pt[0] + ps[1] * pt[1] + ps[2] * pt[2];
		if (value > 1) {
			value = 1;
		}
		else if (value < -1) {
			value = -1;
		}

		double a1 = acos(value);
		double a2 = acos(-value);
		if (a1 > a2) {
			return a2;
		}
		else {
			return a1;
		}
	}

	//Load edge points which have been estimated before
	bool AIVS_Sharp_Load(string fileSharp) {

		ifstream fin(fileSharp);
		if (fin)
		{
			if (borderFeature.size() > 0) {
				borderFeature.clear();
			}
			int numSum;
			fin >> numSum;
			for (int i = 0; i < numSum; i++) {
				int index;
				fin >> index;
				borderFeature.push_back(index);
			}
			fin.close();
			return true;
		}
		else {
			fin.close();
			return false;
		}

	}

	//save edge points
	void AIVS_Sharp_Save(vector<int> n, string fileSharp) {

		if (n.size() <= 0) {
			cout << "normal file name is empty!" << endl;
		}
		else {
			ofstream fout(fileSharp, ios::app);
			fout << n.size() << endl;
			for (int i = 0; i < n.size(); i++) {
				fout << n[i] << endl;
			}
			fout << endl;
			fout.close();
		}

	}
		
	//point classification based on curvature
	void AIVS_Classification_Curvature(vector<double> rate) {

		if (pointCRate.size() > 0) {
			pointCRate.clear();
		}
		pointCRate = rate;
		pointClassification.resize(pointNormalValue.size(), 0);
		vector<double> pNVS = pointNormalValue;
		sort(pNVS.begin(), pNVS.end());
		vector<double> tr(rate.size() + 1);
		tr[0] = 0;
		int unityScale = (double)pNVS.size() / (double)rate.size();
		for (int i = 1; i < tr.size() - 1; i++) {
			tr[i] = pNVS[unityScale * i];
		}
		tr[rate.size()] = pNVS[pNVS.size() - 1] + 0.1;
		vector<int> aSum(rate.size(), 0);//point number in different classes 
		for (int i = 0; i < pointClassification.size(); i++) {
			double pointNormalValue_i = pointNormalValue[i];
			for (int j = 0; j < tr.size() - 1; j++) {
				double tdj = tr[j];
				double tuj = tr[j + 1];
				if (pointNormalValue_i >= tdj && pointNormalValue_i < tuj) {
					aSum[j]++;
					pointClassification[i] = j;
					break;
				}
			}
		}
		pointCSum.clear();
		for (int i = 0; i < aSum.size(); i++) {
			pointCSum.push_back(aSum[i]);
		}
	}	

	//point classification based on edge curvature
	void AIVS_Classification_Edge(vector<double> rate) {

		if (pointCRate.size() > 0) {
			pointCRate.clear();
		}
		pointCRate = rate;
		pointClassification.resize(br.pointCloudData.size(), 0);

		vector<double> pNVS = pointNormalValue;
		sort(pNVS.begin(), pNVS.end());

		double edgeTh = 0.3;

		vector<int> aSum(2, 0);//point number in different classes 
		for (int i = 0; i < pointNormalValue.size(); i++) {
			double bcei = pointNormalValue[i];
			if (bcei > edgeTh) {
				pointClassification[i] = 1;	
				aSum[1]++;
			}			
		}
		aSum[0] = pointClassification.size() - aSum[1];
		pointCSum.clear();
		for (int i = 0; i < aSum.size(); i++) {
			pointCSum.push_back(aSum[i]);
		}
	}

	//achieve accurate simplification points number for parallel computation
	vector<int> AIVS_BoxSimplification_Points(int pointNum) {

		vector<int> boxEstimate;//box simplification estimination
		int pointSum = br.pointCloudData.size();
		double rate = (double)pointNum / (double)pointSum;
		boxEstimate.resize(br.squareBoxes.size(), 0);
		for (int i = 0; i < br.squareBoxes.size(); i++) {
			vector<int> pointInBox = br.squareBoxes[i];
			double simBox = (double)pointInBox.size() * rate;
			int simBoxT = simBox;
			if (simBox - simBoxT > 0.2) {
				boxEstimate[i] = simBoxT + 1;
			}
			else {
				boxEstimate[i] = simBoxT;
			}
		}
		return boxEstimate;
	}

	//achieve accurate simplification points number for parallel computation with different sampling rates
	vector<vector<int>> AIVS_BoxSimplification_Points_UserDefined(int pointNum) {

		vector<vector<int>> boxEstimate;//box simplification estimination
		int pointSum = br.pointCloudData.size();
		boxEstimate.resize(br.squareBoxes.size());

		//pointClassification;//classification number
		//pointCSum;//classification sum
		//pointCRate;//classification rate

		//achieve regular r
		double rr = 0;
		double fenmu = 0;
		for (int i = 0; i < pointCSum.size(); i++) {
			fenmu = fenmu + pointCSum[i] * pointCRate[i];
		}
		rr = (double)pointNum / fenmu;

		for (int i = 0; i < pointCRate.size(); i++) {
			pointCRate[i] = pointCRate[i] * rr;
			if (pointCRate[i] >= 1) {
				pointCRate[i] = 1;
			}
		}

		for (int i = 0; i < br.squareBoxes.size(); i++) {
			vector<int> pointInBox = br.squareBoxes[i];
			vector<int> pointInBoxSum(pointCRate.size(), 0);//clasification rate 
			vector<int> pointInSim(pointCRate.size(), 0);
			for (int j = 0; j < pointInBox.size(); j++) {
				int indexij = pointInBox[j];
				int classIndex = pointClassification[indexij];
				pointInBoxSum[classIndex]++;
			}

			for (int j = 0; j < pointInBoxSum.size(); j++) {
				double sumClass_j = pointInBoxSum[j] * pointCRate[j];
				int sumClass_j_Int = sumClass_j;
				if (sumClass_j - (double)sumClass_j_Int > 0.2) {
					sumClass_j_Int = sumClass_j_Int + 1;
				}
				pointInSim[j] = sumClass_j_Int;
			}
			boxEstimate[i] = pointInSim;
		}

		return boxEstimate;

	}

	//cropping point cloud with certain point number
	vector<vector<double>> AIVS_AccurateCut_Optimization(vector<vector<int>> t, int pointNum) {

		cout << "AIVS_AccurateCut_Optimization start:" << endl;

		vector<vector<double>> finalResult;

		int numberSum = 0;

		vector<int> sampleResult;

		for (int i = 0; i < br.squareBoxes.size(); i++) {
			vector<int> blockIndexi;
			for (int j = 0; j < t[i].size(); j++) {
				int index = t[i][j];
				if (index != -1) {
					blockIndexi.push_back(index);
				}
				else {
					break;
				}
			}
			numberSum = numberSum + blockIndexi.size();
			sampleResult.insert(sampleResult.end(), blockIndexi.begin(), blockIndexi.end());
		}

		//cut the points from the point cloud		
		int dTiff = numberSum - pointNum;
		cout << "AIVS_AccurateCut_Optimization kd-tree construct." << endl;
		//construct kd-tree
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSeed;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->width = sampleResult.size();
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);
		// fills a PointCloud with random data
		for (int i = 0; i < sampleResult.size(); i++)
		{
			pcl::PointXYZ pxyz;
			cloud->points[i].x = br.pointCloudData[sampleResult[i]][0];
			cloud->points[i].y = br.pointCloudData[sampleResult[i]][1];
			cloud->points[i].z = br.pointCloudData[sampleResult[i]][2];
		}
		kdtreeSeed.setInputCloud(cloud);
		vector<vector<float>> pNdis(sampleResult.size());
		vector<vector<int>> pNIndex(sampleResult.size());
		int K = 3;
		for (int i = 0; i < sampleResult.size(); i++) {
			vector<int> pointIdxNKNSearch(K);
			vector<float> pointNKNSquaredDistance(K);
			std::vector<int> pointNeibor_i;
			pcl::PointXYZ searchPoint;
			searchPoint.x = br.pointCloudData[sampleResult[i]][0];
			searchPoint.y = br.pointCloudData[sampleResult[i]][1];
			searchPoint.z = br.pointCloudData[sampleResult[i]][2];
			kdtreeSeed.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
			pNIndex[i] = pointIdxNKNSearch;
			pNdis[i] = pointNKNSquaredDistance;
			for (int j = 0; j < pNdis[i].size(); j++) {
				pNdis[i][j] = sqrt(pNdis[i][j]);
			}
		}

		vector<bool> labelSample(sampleResult.size(), true);

		cout << "AIVS_AccurateCut_Optimization delete point." << endl;

		while (dTiff > 0) {
			double min = 9999;
			int b1 = -1;
			int b2 = -1;
			//achieve the minmum value of the knn
			for (int i = 0; i < pNdis.size(); i++) {
				int b2t = pNIndex[i][1];
				double disTemp = pNdis[i][1];
				if (disTemp < min && labelSample[i] && labelSample[b2t]) {
					min = disTemp;
					b1 = i;
					b2 = b2t;
				}
			}

			if (min == 9999 || b1 == -1 || b2 == -1) {
				break;
			}
			//judge which point should be deleted
			double b1d2 = pNdis[b1][2];
			double b2d2 = pNdis[b2][2];
			int deleteIndex = b1;
			if (b1d2 > b2d2) {
				//delete b2
				deleteIndex = b2;
			}
			//updata the pNdis & pNIndex
			labelSample[deleteIndex] = false;
			dTiff--;
		}
		for (int i = 0; i < labelSample.size(); i++) {

			if (labelSample[i]) {
				int pIndex = sampleResult[i];
				vector<double> pni;
				pni = br.pointCloudData[pIndex];
				pni.insert(pni.end(), br.pointNormal[pIndex].begin(), br.pointNormal[pIndex].end());				
				finalResult.push_back(pni);
			}
		}

		cout << "AIVS_AccurateCut_Optimization finish." << endl;
		return finalResult;
	}

	vector<vector<double>> AIVS_AccurateCut_Optimization(int pointNum, bool cute) {

		cout << "AIVS_AccurateCut_Optimization start:" << endl;

		vector<vector<double>> finalResult;

		int numberSum = 0;

		vector<int> sampleResult;
		vector<int> sampleResultClass;
		for (int i = 0; i < labelG.size(); i++) {
			if (labelG[i] == 0) {
				sampleResult.push_back(i);
				sampleResultClass.push_back(pointClassification[i]);
				numberSum++;
			}
		}

		if (!cute) {

			for (int i = 0; i < sampleResult.size(); i++) {
				vector<double> pni;
				pni = br.pointCloudData[sampleResult[i]];
				pni.insert(pni.end(), br.pointNormal[sampleResult[i]].begin(), br.pointNormal[sampleResult[i]].end());
				finalResult.push_back(pni);
				//finalResult.push_back(br.pointCloudData[sampleResult[i]]);

			}
			return finalResult;
		}

		//cut the points from the point cloud		
		int dTiff = numberSum - pointNum;
		cout << "AIVS_AccurateCut_Optimization kd-tree construct." << endl;
		//construct kd-tree
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSeed;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->width = sampleResult.size();
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);
		// fills a PointCloud with random data
		for (int i = 0; i < sampleResult.size(); i++)
		{
			pcl::PointXYZ pxyz;
			cloud->points[i].x = br.pointCloudData[sampleResult[i]][0];
			cloud->points[i].y = br.pointCloudData[sampleResult[i]][1];
			cloud->points[i].z = br.pointCloudData[sampleResult[i]][2];
		}
		kdtreeSeed.setInputCloud(cloud);
		vector<vector<float>> pNdis(sampleResult.size());
		vector<vector<int>> pNIndex(sampleResult.size());
		int K = 3;
		for (int i = 0; i < sampleResult.size(); i++) {
			vector<int> pointIdxNKNSearch(K);
			vector<float> pointNKNSquaredDistance(K);
			std::vector<int> pointNeibor_i;
			pcl::PointXYZ searchPoint;
			searchPoint.x = br.pointCloudData[sampleResult[i]][0];
			searchPoint.y = br.pointCloudData[sampleResult[i]][1];
			searchPoint.z = br.pointCloudData[sampleResult[i]][2];
			kdtreeSeed.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
			pNIndex[i] = pointIdxNKNSearch;
			pNdis[i] = pointNKNSquaredDistance;
			for (int j = 0; j < pNdis[i].size(); j++) {
				pNdis[i][j] = sqrt(pNdis[i][j]);
			}
		}

		vector<bool> labelSample(sampleResult.size(), true);

		cout << "AIVS_AccurateCut_Optimization delete point." << endl;

		while (dTiff > 0) {
			double min = 9999;
			int b1 = -1;
			int b2 = -1;
			//achieve the minmum value of the knn
			for (int i = 0; i < pNdis.size(); i++) {
				int b2t = pNIndex[i][1];
				double disTemp = pNdis[i][1];
				if (disTemp < min && labelSample[i] && labelSample[b2t]) {
					min = disTemp;
					b1 = i;
					b2 = b2t;
				}
			}

			if (min == 9999 || b1 == -1 || b2 == -1) {
				break;
			}
			//judge which point should be deleted
			double b1d2 = pNdis[b1][2];
			double b2d2 = pNdis[b2][2];
			int deleteIndex = b1;
			if (b1d2 > b2d2) {
				//delete b2
				deleteIndex = b2;
			}
			//updata the pNdis & pNIndex
			labelSample[deleteIndex] = false;
			dTiff--;
		}
		for (int i = 0; i < labelSample.size(); i++) {

			if (labelSample[i]) {
				int pIndex = sampleResult[i];
				vector<double> pni;
				pni = br.pointCloudData[pIndex];
				pni.insert(pni.end(), br.pointNormal[pIndex].begin(), br.pointNormal[pIndex].end());
				finalResult.push_back(pni);
			}
		}

		/*
		while (dTiff > 0) {
			double min = 9999;
			int b1 = -1;
			int b2 = -1;
			//achieve the minmum value of the knn
			for (int i = 0; i < pNdis.size(); i++) {
				int b2t = pNIndex[i][1];
				double disTemp = pNdis[i][1];
				if (disTemp < min && labelSample[i] && labelSample[b2t]&& (sampleResultClass[i]==0 || sampleResultClass[b2t] == 0)) {
					min = disTemp;
					b1 = i;
					b2 = b2t;
				}
			}

			if (min == 9999 || b1 == -1 || b2 == -1) {
				break;
			}
			int deleteIndex = b1;
			//judge which point should be deleted
			//if (sampleResultClass[b1] == 0 && sampleResultClass[b2 != 0]) {
				//deleteIndex = b1;
			//}
			//else if (sampleResultClass[b1] == 0 && sampleResultClass[b2 != 0]) {
				//deleteIndex = b2;

			//}
			//else {
			double b1d2 = pNdis[b1][2];
			double b2d2 = pNdis[b2][2];
			if (b1d2 > b2d2) {
					//delete b2
				deleteIndex = b2;
			}
			//}
			//updata the pNdis & pNIndex
			labelSample[deleteIndex] = false;
			dTiff--;
		}

		for (int i = 0; i < labelSample.size(); i++) {

			if (labelSample[i]) {
				int pIndex = sampleResult[i];
				finalResult.push_back(br.pointCloudData[pIndex]);
			}
		}*/

		cout << "AIVS_AccurateCut_Optimization finish." << endl;
		return finalResult;
	}

	vector<vector<double>> AIVS_AddEdgePoints(vector<vector<double>> seedP) {

		pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSeed;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->width = seedP.size();
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);
		// fills a PointCloud with random data
		for (int i = 0; i < seedP.size(); i++)
		{
			pcl::PointXYZ pxyz;
			cloud->points[i].x = seedP[i][0];
			cloud->points[i].y = seedP[i][1];
			cloud->points[i].z = seedP[i][2];
		}
		kdtreeSeed.setInputCloud(cloud);
		vector<vector<double>> seedPNew;

		double radius_t = 9999;//estimate nearest radius
		double radius_m = -1;
		int K = 2;
		for (int i = 0; i < seedP.size(); i++) {			
			vector<int> pointIdxNKNSearch(K);
			vector<float> pointNKNSquaredDistance(K);
			std::vector<int> pointNeibor_i;
			pcl::PointXYZ searchPoint;
			searchPoint.x = seedP[i][0];
			searchPoint.y = seedP[i][1];
			searchPoint.z = seedP[i][2];
			kdtreeSeed.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
			double dis_i = sqrt(pointNKNSquaredDistance[1]);			
			if (dis_i < radius_t) {
				radius_t = dis_i;
			}
			if (dis_i > radius_m) {
				radius_m = radius_t;			
			}
		}		

		K = 1;
		for (int i = 0; i < pointClassification.size(); i++) {
			if (pointClassification[i] != 1) {
				continue;			
			}
			vector<int> pointIdxNKNSearch(K);
			vector<float> pointNKNSquaredDistance(K);
			std::vector<int> pointNeibor_i;
			pcl::PointXYZ searchPoint;
			searchPoint.x = br.pointCloudData[i][0];
			searchPoint.y = br.pointCloudData[i][1];
			searchPoint.z = br.pointCloudData[i][2];
			kdtreeSeed.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);			
			double dis_i = sqrt(pointNKNSquaredDistance[0]);
			double dis_i2 = AIVS_AddEdgePoints_ReturnMinDis(br.pointCloudData[i], seedPNew);			
			if (dis_i2 < dis_i) {
				dis_i = dis_i2;			
			}
			//if (pointClassification[i] != 1) {				
				//if (dis_i > radius_m * 0.8) {
					//seedPNew.push_back(br.pointCloudData[i]);
				//}				
			//}
			//else {
			if (dis_i > (radius_t / 2)) {
				seedPNew.push_back(br.pointCloudData[i]);
			}
			//}			
		}
		seedP.insert(seedP.end(), seedPNew.begin(), seedPNew.end());
		return seedP;
	}

	double AIVS_AddEdgePoints_ReturnMinDis(vector<double> p, vector<vector<double>> seedNew) {

		double dis_min = 9999;
		for (int i = 0; i < seedNew.size(); i++) {

			vector<double> p2 = seedNew[i];
			double dis_i = sqrt((p[0] - p2[0]) * (p[0] - p2[0]) + 
				(p[1] - p2[1]) * (p[1] - p2[1]) + 
				(p[2] - p2[2]) * (p[2] - p2[2]));
			if (dis_i < dis_min) {
				dis_min = dis_i;			
			}		
		}
		return dis_min;	
	}

#pragma endregion
	
};