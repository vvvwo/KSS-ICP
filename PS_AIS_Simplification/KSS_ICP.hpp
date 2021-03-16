/*********************************************************************************

		          KSS ICP for Point Cloud Registration

						Updating in 2021/03/02

						   By Dr. Chenlei Lv

			The functions includes:
			1. KSS_ICP point cloud registration
			2. Transfer point cloud by transfer matrix

*********************************************************************************/

#pragma once
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/3dsc.h>
#include "pointPipeline.hpp"
#include "initRegistrationKSS.hpp"
#include "transferPC.hpp"
#include "Method_AIVS_SimPro.hpp"
#include <pcl/common/transforms.h>

using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class KSSICP {

private:

	vector<vector<double>> pointSource;
	vector<vector<double>> pointTarget;
	int pNumber;
	double accurateG;

public:
	
	vector<vector<double>> pointAlign;

public:

	void KSSICP_init(vector<vector<double>> ps, vector<vector<double>> pt, double accurate) {
		accurateG = accurate;
		pointSource = ps;
		pointTarget = pt;
		if (pointSource.size() > pointTarget.size()) {
			pNumber = pointTarget.size();
		}
		else {
			pNumber = pointSource.size();
		}
		pNumber = pNumber / 2;
		if (pNumber > 2000) {
			pNumber = 2000;
		}
	}

	void KSSICP_Registration(int iter) {

		pointPipeline ppt;
		ppt.pointPipeline_init_point_withoutUniform(pointTarget);
		AIVS_Simplification_Pro asp;
		asp.AIVS_Pro_init(ppt.br, "target");
		vector<vector<double>> pointCloudT = asp.AIVS_simplification(pNumber);

		pointPipeline pps;
		pps.pointPipeline_init_point_withoutUniform(pointSource);
		AIVS_Simplification_Pro asps;
		asps.AIVS_Pro_init(pps.br, "source");
		vector<vector<double>> pointCloudS = asps.AIVS_simplification(pNumber);

		//save_PointCloud(pointCloudT, "E://simT.xyz");
		//save_PointCloud(pointCloudS, "E://simS.xyz");

		initRegistration_KSS ir;
		ir.initRegistration_init(pointCloudS, pointCloudT, accurateG);
		//pointAlign = ir.initRegistration_Rotation(pointSource);

		vector<vector<double>> angleListLocal = ir.angleList;
		vector<vector<double>> pointAlignSSS;
		if (angleListLocal.size() > 0) {
			double Q = 9999;
			int angleIndex = 0;
			for (int i = 0; i < angleListLocal.size(); i++) {
				vector<vector<double>> pointAlignSSSi = ir.initRegistration_Rotation_Angle(pointCloudS, angleListLocal[i]);//save_PointCloud(pointAlignSSS, "E://result.xyz");
				double ri = shapeRegistration_ICP_AngleList(iter, Q, pointAlignSSSi, pointCloudT);
				cout << "kernel" << i << ":" << ri << endl;
				if (ri < Q && ri >= 0) {
					Q = ri;
					angleIndex = i;
				}
			}
			pointAlignSSS = ir.initRegistration_Rotation_Angle(pointCloudS, angleListLocal[angleIndex]);
			pointAlign = ir.initRegistration_Rotation_Angle(pointSource, angleListLocal[angleIndex]);
		}
		else {
			pointAlignSSS = ir.initRegistration_Rotation(pointCloudS);//save_PointCloud(pointAlignSSS, "E://result.xyz");
			pointAlign = ir.initRegistration_Rotation(pointSource);
		}

		//save_PointCloud(pointAlignSSS, "E://simA.xyz");

		pointSource.clear();
		pointSource = pointAlign;
		pointAlign.clear();
		double resultFitness = shapeRegistration_ICP(iter, pointAlignSSS, pointCloudT);
	}
	
	double shapeRegistration_ICP(int iter) {

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_s(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_t(new pcl::PointCloud<pcl::PointXYZ>);
		for (int i = 0; i < pointSource.size(); i++)
		{
			pcl::PointXYZ cloud_i;
			cloud_i.x = pointSource[i][0];
			cloud_i.y = pointSource[i][1];
			cloud_i.z = pointSource[i][2];
			cloud_s->push_back(cloud_i);
		}
		for (int i = 0; i < pointTarget.size(); i++)
		{
			pcl::PointXYZ cloud_i;
			cloud_i.x = pointTarget[i][0];
			cloud_i.y = pointTarget[i][1];
			cloud_i.z = pointTarget[i][2];
			cloud_t->push_back(cloud_i);
		}

		pcl::PointCloud<pcl::PointXYZ> output;
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setMaxCorrespondenceDistance(1);
		icp.setTransformationEpsilon(1e-10);
		icp.setEuclideanFitnessEpsilon(0.001);
		icp.setMaximumIterations(iter);
		icp.setInputSource(cloud_s);
		icp.setInputTarget(cloud_t);
		icp.align(output);

		double result = icp.getFitnessScore();
		std::cout << "has converged: " << icp.hasConverged() << std::endl;
		std::cout << "score: " << icp.getFitnessScore() << std::endl;
		std::cout << icp.getFinalTransformation() << std::endl;
		pointAlign.clear();

		for (int i = 0; i < output.size(); i++) {
			pcl::PointXYZ pi = output[i];
			double xi = pi.x;
			double yi = pi.y;
			double zi = pi.z;
			vector<double> ppi;
			ppi.push_back(xi);
			ppi.push_back(yi);
			ppi.push_back(zi);
			pointAlign.push_back(ppi);
		}

		return result;
	}

	double shapeRegistration_ICP(int iter, vector<vector<double>> ps, vector<vector<double>> pt) {

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_s(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_t(new pcl::PointCloud<pcl::PointXYZ>);
		for (int i = 0; i < ps.size(); i++)
		{
			pcl::PointXYZ cloud_i;
			cloud_i.x = ps[i][0];
			cloud_i.y = ps[i][1];
			cloud_i.z = ps[i][2];
			cloud_s->push_back(cloud_i);
		}
		for (int i = 0; i < pt.size(); i++)
		{
			pcl::PointXYZ cloud_i;
			cloud_i.x = pt[i][0];
			cloud_i.y = pt[i][1];
			cloud_i.z = pt[i][2];
			cloud_t->push_back(cloud_i);
		}

		pcl::PointCloud<pcl::PointXYZ> output;
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setMaxCorrespondenceDistance(1);
		icp.setTransformationEpsilon(1e-10);
		icp.setEuclideanFitnessEpsilon(0.001);
		icp.setMaximumIterations(iter);
		icp.setInputSource(cloud_s);
		icp.setInputTarget(cloud_t);
		icp.align(output);

		double result = icp.getFitnessScore();
		std::cout << "has converged: " << icp.hasConverged() << std::endl;
		std::cout << "score: " << icp.getFitnessScore() << std::endl;
		std::cout << icp.getFinalTransformation() << std::endl;
		pointAlign.clear();		

		Eigen::Matrix<float, 4, 4> rt = icp.getFinalTransformation();

		for (int i = 0; i < pointSource.size(); i++) {
			vector<double> pi(3);
			pi[0] = rt(0, 0) * pointSource[i][0] + rt(0, 1) * pointSource[i][1] + rt(0, 2) * pointSource[i][2] + rt(0, 3);
			pi[1] = rt(1, 0) * pointSource[i][0] + rt(1, 1) * pointSource[i][1] + rt(1, 2) * pointSource[i][2] + rt(1, 3);
			pi[2] = rt(2, 0) * pointSource[i][0] + rt(2, 1) * pointSource[i][1] + rt(2, 2) * pointSource[i][2] + rt(2, 3);
			pointAlign.push_back(pi);
		}	

		return result;
	}

	//additional process
	double shapeRegistration_ICP_AngleList(int iter, double Q, vector<vector<double>> ps, vector<vector<double>> pt) {

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_s(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_t(new pcl::PointCloud<pcl::PointXYZ>);
		for (int i = 0; i < ps.size(); i++)
		{
			pcl::PointXYZ cloud_i;
			cloud_i.x = ps[i][0];
			cloud_i.y = ps[i][1];
			cloud_i.z = ps[i][2];
			cloud_s->push_back(cloud_i);
		}
		for (int i = 0; i < pt.size(); i++)
		{
			pcl::PointXYZ cloud_i;
			cloud_i.x = pt[i][0];
			cloud_i.y = pt[i][1];
			cloud_i.z = pt[i][2];
			cloud_t->push_back(cloud_i);
		}

		pcl::PointCloud<pcl::PointXYZ> output;
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setMaxCorrespondenceDistance(1);
		icp.setTransformationEpsilon(1e-10);
		icp.setEuclideanFitnessEpsilon(0.001);
		icp.setMaximumIterations(iter);
		icp.setInputSource(cloud_s);
		icp.setInputTarget(cloud_t);
		icp.align(output);

		double result = icp.getFitnessScore();
		//std::cout << "has converged: " << icp.hasConverged() << std::endl;
		//std::cout << "score: " << icp.getFitnessScore() << std::endl;
		//std::cout << icp.getFinalTransformation() << std::endl;
		//pointAlign.clear();
		return result;	

	}

	

	double shapeRegistration_ICP_Input(int iter, double Q, vector<vector<double>> ps, vector<vector<double>> pt) {

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_s(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_t(new pcl::PointCloud<pcl::PointXYZ>);
		for (int i = 0; i < ps.size(); i++)
		{
			pcl::PointXYZ cloud_i;
			cloud_i.x = ps[i][0];
			cloud_i.y = ps[i][1];
			cloud_i.z = ps[i][2];
			cloud_s->push_back(cloud_i);
		}
		for (int i = 0; i < pt.size(); i++)
		{
			pcl::PointXYZ cloud_i;
			cloud_i.x = pt[i][0];
			cloud_i.y = pt[i][1];
			cloud_i.z = pt[i][2];
			cloud_t->push_back(cloud_i);
		}

		pcl::PointCloud<pcl::PointXYZ> output;
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setMaxCorrespondenceDistance(1);
		icp.setTransformationEpsilon(1e-10);
		icp.setEuclideanFitnessEpsilon(0.001);
		icp.setMaximumIterations(iter);
		icp.setInputSource(cloud_s);
		icp.setInputTarget(cloud_t);
		icp.align(output);

		double result = icp.getFitnessScore();
		std::cout << "has converged: " << icp.hasConverged() << std::endl;
		std::cout << "score: " << icp.getFitnessScore() << std::endl;
		std::cout << icp.getFinalTransformation() << std::endl;

		if (result < Q) {			
			pointAlign.clear();
			for (int i = 0; i < output.size(); i++) {
				pcl::PointXYZ pi = output[i];
				double xi = pi.x;
				double yi = pi.y;
				double zi = pi.z;
				vector<double> ppi;
				ppi.push_back(xi);
				ppi.push_back(yi);
				ppi.push_back(zi);
				pointAlign.push_back(ppi);
			}		
		}
		return result;
	}

	void IntrinsicICP_Evaluate(int Number) {

		vector<double> errorVector;

		IntrinsicICP_P();

		for (int i = 0; i < Number; i++) {
			double errori = shapeRegistration_ICP(i);
			errorVector.push_back(errori);
		}

		for (int i = 0; i < errorVector.size(); i++) {
			cout << "iter " << i << ":" << errorVector[i] << endl;
		}

	}
	
	vector<vector<double>> IntrinsicICP_pointSource() {

		return pointSource;

	}
	vector<vector<double>> IntrinsicICP_pointTarget() {

		return pointTarget;

	}

private:

	void KSSICP_Registration_Additional(int iter, vector<vector<double>> pointCloudAlignS, vector<vector<double>> pointCloudT) {

		initRegistration_KSS ir;
		ir.initRegistration_init(pointCloudAlignS, pointCloudT, 12);
		pointSource.clear();
		pointSource = ir.initRegistration_Rotation(pointAlign);
		pointAlign.clear();

	}

	void IntrinsicICP_P() {

		pointPipeline ppt;
		ppt.pointPipeline_init_point(pointTarget);
		AIVS_Simplification_Pro asp;
		asp.AIVS_Pro_init(ppt.br, "target");
		vector<vector<double>> pointCloudT = asp.AIVS_simplification(5000);

		pointPipeline pps;
		pps.pointPipeline_init_point(pointSource);
		AIVS_Simplification_Pro asps;
		asps.AIVS_Pro_init(pps.br, "source");
		vector<vector<double>> pointCloudS = asps.AIVS_simplification(5000);

		initRegistration ir;
		ir.initRegistration_init(pointCloudS, pointCloudT);
		pointSource.clear();
		pointSource = ir.pointSource;
		pointTarget.clear();
		pointTarget = ir.pointTarget;

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

};


