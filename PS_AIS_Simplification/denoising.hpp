/*********************************************************************************
                      
					  Point Cloud Denoising Object

					    Programing in 2020/08/11

						   By Dr. Chenlei Lv

			The functions includes: 
			1. Add Gaussian noise into a point cloud;
			2. Deoising by Gaussian kernel function for a point cloud;
			2. Deoising by Fast BilateralFilter function for a point cloud;

*********************************************************************************/

#pragma once
#include <iostream> 
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h> 
#include <pcl/point_cloud.h>
#include <boost/random.hpp>
#include <pcl/console/time.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/kdtree/kdtree_flann.h>
using namespace std;

class DenoisingPD {

public:

	vector<vector<double>> pointCloudOriginal;//point cloud
	vector<vector<double>> pointCloudONormal;//normal vectors	
	vector<vector<double>> pCDenosing;//remove noise from a point cloud
	vector<vector<double>> pCDenosingNormal;//new normal
	double K = 20;//K-neighbor
	double segma;
	double radius;//searching K-neighbor radius
	int iter_Global = 10;//point update iter

public:

	void DenoisingPD_init(vector<vector<double>> p, vector<vector<double>> n) {
		cout << "Denoising init start:" << endl;
		pointCloudOriginal = p;
		pointCloudONormal = n;			
		cout << "Searching the radius:" << endl;
		DenoisingPD_SearchingRadiusByKneighboor();
		cout << "Denoising init finished:" << endl;
	}

	void DenoisingPD_AddNoise() {	
		cout << "Add noising start:" << endl;
		boost::mt19937 rng;
		rng.seed(static_cast<unsigned int>(time(0)));
		boost::normal_distribution<> nd(0, radius/3);
		boost::variate_generator<boost::mt19937&, boost::normal_distribution<>> var_nor(rng, nd);
		for (int i = 0; i < pointCloudOriginal.size(); i++) {
			double randomi = static_cast<float> (var_nor());
			pointCloudOriginal[i][0] = pointCloudOriginal[i][0] + randomi * pointCloudONormal[i][0];
			pointCloudOriginal[i][1] = pointCloudOriginal[i][1] + randomi * pointCloudONormal[i][1];
			pointCloudOriginal[i][2] = pointCloudOriginal[i][2] + randomi * pointCloudONormal[i][2];
		}	
		cout << "Add noising finished!" << endl;
	}

	void DenoisingPD_AddNoise(GLMmodel* pModelM, string fileName) {
		cout << "Add noising start:" << endl;
		boost::mt19937 rng;
		rng.seed(static_cast<unsigned int>(time(0)));
		boost::normal_distribution<> nd(0, radius / 3);
		boost::variate_generator<boost::mt19937&, boost::normal_distribution<>> var_nor(rng, nd);
		for (int i = 0; i < pointCloudOriginal.size(); i++) {
			double randomi = static_cast<float> (var_nor());
			pointCloudOriginal[i][0] = pointCloudOriginal[i][0] + randomi * pointCloudONormal[i][0];
			pointCloudOriginal[i][1] = pointCloudOriginal[i][1] + randomi * pointCloudONormal[i][1];
			pointCloudOriginal[i][2] = pointCloudOriginal[i][2] + randomi * pointCloudONormal[i][2];
		}
		vector<vector<int>> faceInfor;
		for (int i = 0; i < pModelM->numtriangles; i++) {
			int b1 = pModelM->triangles[i].vindices[0];
			int b2 = pModelM->triangles[i].vindices[1];
			int b3 = pModelM->triangles[i].vindices[2];
			vector<int> face_i;
			face_i.push_back(b1);
			face_i.push_back(b2);
			face_i.push_back(b3);
			faceInfor.push_back(face_i);
		}

		DenoisingPD_SaveOBJ_Face(pointCloudOriginal, faceInfor, fileName);
		cout << "Add noising finished!" << endl;
	}

	void DenoisingPD_GaussianKernel(double segmai,double Ki) {

		K = Ki;
		segma = segmai;
		//update radius
		DenoisingPD_SearchingRadiusByKneighboor();

		pCDenosing.resize(pointCloudOriginal.size());

		cout << "Gaussian denoising start, the parameters: segma = "<<segma<<" K = "<<K<< endl;

		pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr outputcloud(new pcl::PointCloud<pcl::PointXYZ>);

		//Read point cloud in the input file
		for (int i = 0; i < pointCloudOriginal.size(); i++)
		{
			pcl::PointXYZ cloud_i;
			cloud_i.x = pointCloudOriginal[i][0];
			cloud_i.y = pointCloudOriginal[i][1];
			cloud_i.z = pointCloudOriginal[i][2];			
			inputcloud->push_back(cloud_i);
		}

		//Set up the Gaussian Kernel
		pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>::Ptr kernel(new pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>);
		(*kernel).setSigma(segma);
		(*kernel).setThreshold(radius);
		//(*kernel).setThresholdRelativeToSigma(segma);
		std::cout << "Kernel made" << std::endl;

		//Set up the KDTree
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		(*kdtree).setInputCloud(inputcloud);
		std::cout << "KdTree made" << std::endl;


		//Set up the Convolution Filter
		pcl::filters::Convolution3D <pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> > convolution;
		convolution.setKernel(*kernel);
		convolution.setInputCloud(inputcloud);
		convolution.setSearchMethod(kdtree);
		convolution.setRadiusSearch(radius);
		convolution.setNumberOfThreads(10);
		std::cout << "Convolution Start" << std::endl;
		convolution.convolve(*outputcloud);
		std::cout << "Convoluted" << std::endl;

		for (int i = 0; i < outputcloud->size(); i++) {
			vector<double> pi(3);
			pi[0] = outputcloud->at(i).x;
			pi[1] = outputcloud->at(i).y;
			pi[2] = outputcloud->at(i).z;
			pCDenosing[i] = pi;
		}
		cout << "Gaussian denoising finished!" << endl;
	}

	void DenoisingPD_BilateralFilter(double segmai, double Ki) {

		float sigma_s = segmai;
		K = Ki;
		DenoisingPD_SearchingRadiusByKneighboor();
		float sigma_r = radius;

		//sigma_s:
		//Set the standard deviation of the Gaussian used by the bilateral filter 
		//for the spatial neighborhood/window.	

		//sigma_r:
		//Set the standard deviation of the Gaussian used to control how much 
		//an adjacent pixel is downweighted because of the intensity difference

		pCDenosing.resize(pointCloudOriginal.size());

		cout << "BilateralFilter denoising start:" << endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud(new pcl::PointCloud<pcl::PointXYZ>);
		//fromPCLPointCloud2(*input, *inputcloud);

		//Read point cloud in the input file
		for (int i = 0; i < pointCloudOriginal.size(); i++)
		{
			pcl::PointXYZ cloud_i;
			cloud_i.x = pointCloudOriginal[i][0];
			cloud_i.y = pointCloudOriginal[i][1];
			cloud_i.z = pointCloudOriginal[i][2];			
			inputcloud->push_back(cloud_i);
		}		
		
		pcl::FastBilateralFilter<pcl::PointXYZ> fbf;
		fbf.setInputCloud(inputcloud);
		fbf.setSigmaS(sigma_s);
		fbf.setSigmaR(sigma_r);
		pcl::PointCloud<pcl::PointXYZ> xyz_filtered;
		fbf.filter(xyz_filtered);	

		for (int i = 0; i < xyz_filtered.size(); i++) {
			vector<double> pi(3);
			pi[0] = xyz_filtered.at(i).x;
			pi[1] = xyz_filtered.at(i).y;
			pi[2] = xyz_filtered.at(i).z;
			pCDenosing[i] = pi;
		}
		cout << "BilateralFilter denoising finished!" << endl;
	}
	
	void DenoisingPD_MLS_Smooth(int iterNum, int Ki) {

		K = Ki;
		DenoisingPD_SearchingRadiusByKneighboor();

		cout << "MLS_Smooth start. The parameter: Iter = " << iterNum << ",K = " << K << endl;
		vector<vector<double>> pS = pointCloudOriginal;
		vector<vector<double>> nS = pointCloudONormal;

		while (iterNum) {
			cout << "update iter:" << iterNum << endl;
			iterNum--;
			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;//kdtree
			std::cout << "Init kdtree" << std::endl;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			cloud->width = pointCloudOriginal.size();
			cloud->height = 1;
			cloud->points.resize(cloud->width * cloud->height);
			// fills a PointCloud with random data
			for (int i = 0; i < pS.size(); i++)
			{
				pcl::PointXYZ pxyz;
				cloud->points[i].x = pS[i][0];
				cloud->points[i].y = pS[i][1];
				cloud->points[i].z = pS[i][2];

			}
			kdtree.setInputCloud(cloud);
			vector<vector<vector<double>>> result = DenoisingPD_pointUpdate(pS, nS, kdtree);
			pS.clear();
			nS.clear();
			pS = result[0];	
			nS = result[1];
		}	

		pCDenosing.clear();
		pCDenosing = pS;
		pCDenosingNormal.clear();
		pCDenosingNormal = nS;
	}

	void DenoisingPD_MLS_Smooth(int iterNum, int Ki, GLMmodel* pModelM, string fileName) {

		K = Ki;
		DenoisingPD_SearchingRadiusByKneighboor();

		cout << "MLS_Smooth start. The parameter: Iter = " << iterNum << ",K = " << K << endl;
		vector<vector<double>> pS = pointCloudOriginal;
		vector<vector<double>> nS = pointCloudONormal;

		while (iterNum) {
			cout << "update iter:" << iterNum << endl;
			iterNum--;
			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;//kdtree
			std::cout << "Init kdtree" << std::endl;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			cloud->width = pointCloudOriginal.size();
			cloud->height = 1;
			cloud->points.resize(cloud->width * cloud->height);
			// fills a PointCloud with random data
			for (int i = 0; i < pS.size(); i++)
			{
				pcl::PointXYZ pxyz;
				cloud->points[i].x = pS[i][0];
				cloud->points[i].y = pS[i][1];
				cloud->points[i].z = pS[i][2];

			}
			kdtree.setInputCloud(cloud);
			vector<vector<vector<double>>> result = DenoisingPD_pointUpdate(pS, nS, kdtree);
			pS.clear();
			nS.clear();
			pS = result[0];
			nS = result[1];
		}
		vector<vector<int>> faceInfor;
		for (int i = 0; i < pModelM->numtriangles; i++) {
			int b1 = pModelM->triangles[i].vindices[0];
			int b2 = pModelM->triangles[i].vindices[1];
			int b3 = pModelM->triangles[i].vindices[2];
			vector<int> face_i;
			face_i.push_back(b1);
			face_i.push_back(b2);
			face_i.push_back(b3);
			faceInfor.push_back(face_i);
		}
		pCDenosing.clear();
		pCDenosing = pS;
		DenoisingPD_SaveOBJ_Face(pCDenosing, faceInfor, fileName + "_dnoise");
	}

private:

	void DenoisingPD_SearchingRadiusByKneighboor() {

		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;//kdtree
		std::cout << "Init kdtree" << std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->width = pointCloudOriginal.size();
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);
		// fills a PointCloud with random data
		for (int i = 0; i < pointCloudOriginal.size(); i++)
		{
			pcl::PointXYZ pxyz;
			cloud->points[i].x = pointCloudOriginal[i][0];
			cloud->points[i].y = pointCloudOriginal[i][1];
			cloud->points[i].z = pointCloudOriginal[i][2];

		}
		kdtree.setInputCloud(cloud);

		double rMax = -1;
	
		int step = pointCloudOriginal.size() / 1000;
		if (step < 1) {
			for (int i = 0; i < pointCloudOriginal.size(); i++) {				
				std::vector<int> pointIdxNKNSearch(K);
				std::vector<float> pointNKNSquaredDistance(K);
				std::vector<int> pointNeibor_i;
				pcl::PointXYZ searchPoint;
				searchPoint.x = pointCloudOriginal[i][0];
				searchPoint.y = pointCloudOriginal[i][1];
				searchPoint.z = pointCloudOriginal[i][2];
				kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
				double dis = sqrt(pointNKNSquaredDistance[pointNKNSquaredDistance.size()-1]);
				if (dis > rMax) {
					rMax = dis;				
				}
			}		
		}
		else {
			for (int i = 0; i < step; i++) {
				int index = i * 1000;
				std::vector<int> pointIdxNKNSearch(K);
				std::vector<float> pointNKNSquaredDistance(K);
				std::vector<int> pointNeibor_i;
				pcl::PointXYZ searchPoint;
				searchPoint.x = pointCloudOriginal[index][0];
				searchPoint.y = pointCloudOriginal[index][1];
				searchPoint.z = pointCloudOriginal[index][2];
				kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
				double dis = sqrt(pointNKNSquaredDistance[pointNKNSquaredDistance.size() - 1]);
				if (dis > rMax) {
					rMax = dis;
				}
			}		
		}
		radius = rMax;
	}
	
	vector<vector<vector<double>>> DenoisingPD_pointUpdate(vector<vector<double>> pointd, 
		vector<vector<double>> normald, pcl::KdTreeFLANN<pcl::PointXYZ> kdtree) {

		double h = radius;
		vector<vector<double>> pointS(pointd.size());
		vector<vector<double>> normalS(pointd.size());

#pragma omp parallel for
		for (int i = 0; i < pointd.size(); i++) {
			//cout << i << ",";
			if (i % 1000 == 0) {
				cout << (pointd.size() - i) / 1000 << ",";
			}
			vector<double> point_i = pointd[i];
			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);
			std::vector<int> pointNeior;
			//double r_i = pointNKNSquaredDistance[pointNKNSquaredDistance.size() - 1];
			pcl::PointXYZ searchPoint;
			searchPoint.x = point_i[0];
			searchPoint.y = point_i[1];
			searchPoint.z = point_i[2];
			kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);			
			pointNeior.insert(pointNeior.end(), pointIdxNKNSearch.begin(), pointIdxNKNSearch.end());			
			vector<vector<double>> pointNormal_i(pointNeior.size());

			for (int i = 0; i < pointIdxNKNSearch.size(); i++) {
				pointNormal_i[i] = normald[pointIdxNKNSearch[i]];			
			}

			//regular normal
			vector<double> normalUniform = pointNormal_i[0];
			for (int i = 0; i < pointNormal_i.size(); i++) {
				vector<double> normalNeibor_i = pointNormal_i[i];
				vector<double> normalNeibor_i2;
				normalNeibor_i2.push_back(-normalNeibor_i[0]);
				normalNeibor_i2.push_back(-normalNeibor_i[1]);
				normalNeibor_i2.push_back(-normalNeibor_i[2]);
				//manage different directions
				double a1 = normalUniform[0] * normalNeibor_i[0] +
					normalUniform[1] * normalNeibor_i[1] + normalUniform[2] * normalNeibor_i[2];
				double a2 = normalUniform[0] * normalNeibor_i2[0] +
					normalUniform[1] * normalNeibor_i2[1] + normalUniform[2] * normalNeibor_i2[2];
				double cosa1 = acos(a1);
				double cosa2 = acos(a2);
				if (cosa1 > cosa2) {
					pointNormal_i[i][0] = normalNeibor_i2[0];
					pointNormal_i[i][1] = normalNeibor_i2[1];
					pointNormal_i[i][2] = normalNeibor_i2[2];
				}
			}

#pragma endregion

#pragma region MLS error
			//++++++++++++++++++++interater start+++++++++++++++++++++++++++
			double errorExist = 0.0001;
			vector<double> px;
			px.insert(px.end(), point_i.begin(), point_i.end());
			//vector<int> p_neibor = br.pointNeibor[i];			
			//regularNoraml

			vector<double> px_store(3);
			vector<double> nx_store(3);
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
			int iter = iter_Global;
			while (iter) {
				//vector<double> ax = simMeasurement_cop_a(px, pointNeiborRegualrNum);
				//vector<double> nx = simMeasurement_cop_n(px, pointNeiborRegualrNum, pointNeiborNormalRegualrNum);
				double fenmu = 0;
				for (int j = 0; j < pointNeior.size(); j++) {
					double dis_i = sqrt((pointd[pointNeior[j]][0] - px[0]) * (pointd[pointNeior[j]][0] - px[0]) +
						(pointd[pointNeior[j]][1] - px[1]) * (pointd[pointNeior[j]][1] - px[1]) +
						(pointd[pointNeior[j]][2] - px[2]) * (pointd[pointNeior[j]][2] - px[2]));
					if (dis_i == 0) {
						continue;
					}
					double eData = -((dis_i / h) * (dis_i / h));
					eData = exp(eData);
					ax[0] = ax[0] + pointd[pointNeior[j]][0] * eData;
					ax[1] = ax[1] + pointd[pointNeior[j]][1] * eData;
					ax[2] = ax[2] + pointd[pointNeior[j]][2] * eData;
					nx[0] = nx[0] + normald[pointNeior[j]][0] * eData;
					nx[1] = nx[1] + normald[pointNeior[j]][1] * eData;
					nx[2] = nx[2] + normald[pointNeior[j]][2] * eData;
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
				weight = nx[0] * (point_i[0] - ax[0]) +
					nx[1] * (point_i[1] - ax[1]) + nx[2] * (point_i[2] - ax[2]);
				px_store[0] = px[0];
				px_store[1] = px[1];
				px_store[2] = px[2];
				nx_store[0] = nx[0];
				nx_store[1] = nx[1];
				nx_store[2] = nx[2];
				px[0] = point_i[0] - weight * nx[0];
				px[1] = point_i[1] - weight * nx[1];
				px[2] = point_i[2] - weight * nx[2];
				//5.4 ||n(x')T(a(x')-x)n(x')||>errorEndTem
				ax[0] = 0;
				ax[1] = 0;
				ax[2] = 0;
				nx[0] = 0;
				nx[1] = 0;
				nx[2] = 0;

				//vector<double> axnew = simMeasurement_cop_a(px, pointNeiborRegualrNum);
				//vector<double> nxnew = simMeasurement_cop_n(px, pointNeiborRegualrNum, pointNeiborNormalRegualrNum);

				for (int j = 0; j < pointNeior.size(); j++) {
					double dis_i = sqrt((pointd[pointNeior[j]][0] - px[0]) * (pointd[pointNeior[j]][0] - px[0]) +
						(pointd[pointNeior[j]][1] - px[1]) * (pointd[pointNeior[j]][1] - px[1]) +
						(pointd[pointNeior[j]][2] - px[2]) * (pointd[pointNeior[j]][2] - px[2]));
					if (dis_i == 0) {
						continue;
					}
					double eData = -((dis_i / h) * (dis_i / h));
					eData = exp(eData);
					ax[0] = ax[0] + pointd[pointNeior[j]][0] * eData;
					ax[1] = ax[1] + pointd[pointNeior[j]][1] * eData;
					ax[2] = ax[2] + pointd[pointNeior[j]][2] * eData;
					nx[0] = nx[0] + normald[pointNeior[j]][0] * eData;
					nx[1] = nx[1] + normald[pointNeior[j]][1] * eData;
					nx[2] = nx[2] + normald[pointNeior[j]][2] * eData;
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

				weight = nx[0] * (point_i[0] - ax[0]) +
					nx[1] * (point_i[1] - ax[1]) + nx[2] * (point_i[2] - ax[2]);

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
#pragma endregion 

			vector<double> finalResult(3);
			finalResult[0] = px[0];
			finalResult[1] = px[1];
			finalResult[2] = px[2];
			vector<double> finalResult2(3);
			finalResult2[0] = nx_store[0];
			finalResult2[1] = nx_store[1];
			finalResult2[2] = nx_store[2];
			pointS[i] = finalResult;
			normalS[i] = finalResult2;
		}
		cout << endl;
		vector<vector<vector<double>>> result;
		result.push_back(pointS);
		result.push_back(normalS);

		cout << pointd[0][0] << "," << pointd[0][1] << "," << pointd[0][2] << "," << endl;
		cout << pointS[0][0] << "," << pointS[0][1] << "," << pointS[0][2] << "," << endl;
		

		return result;
	}
	
	void DenoisingPD_SaveOBJ_Face(vector<vector<double>> points, vector<vector<int>> faceInfor, string fileName) {

		string fin = "Noise\\"+ fileName+"_noise.obj";

		ofstream f1(fin);

		//f1 << points.size() << " " << facet.size() << " " << 0 << endl;

		for (int i = 0; i < points.size(); i++) {
			f1 << "v " << points[i][0] << " " << points[i][1] << " " << points[i][2] << " " << endl;
		}

		for (int i = 0; i < faceInfor.size(); i++) {
			f1 << "f " << faceInfor[i][0] << " " << faceInfor[i][1] << " " << faceInfor[i][2] << endl;
		}

		f1.close();


	}
};

