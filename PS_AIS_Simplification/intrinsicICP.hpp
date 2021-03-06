/*********************************************************************************

				 Intrinsic ICP for Point Cloud based Shape 
				        Registration

						Updating in 2021/01/22

						   By Dr. Chenlei Lv

			The functions includes:
			1. Intrinsic ICP shape registration
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
#include "initRegistration.hpp"
#include "transferPC.hpp"
#include "Method_AIVS_SimPro.hpp"

using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class IntrinsicICP {

private:

	vector<vector<double>> pointSource;
	vector<vector<double>> pointTarget;
	int pNumber;

public:

	vector<vector<double>> pointAlign;

public:

	void IntrinsicICP_init(vector<vector<double>> ps, vector<vector<double>> pt) {

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
		
	void IntrinsicICP_Registration(int iter) {

		pointPipeline ppt;
		ppt.pointPipeline_init_point(pointTarget);
		AIVS_Simplification_Pro asp;
		asp.AIVS_Pro_init(ppt.br, "target");
		vector<vector<double>> pointCloudT = asp.AIVS_simplification(pNumber);

		pointPipeline pps;
		pps.pointPipeline_init_point(pointSource);
		AIVS_Simplification_Pro asps;
		asps.AIVS_Pro_init(pps.br, "source");
		vector<vector<double>> pointCloudS = asps.AIVS_simplification(pNumber);

		initRegistration ir;
		ir.initRegistration_init(pointCloudS, pointCloudT);
		pointSource.clear();
		pointSource = ir.pointSource;
		pointTarget.clear();
		pointTarget = ir.pointTarget;
		shapeRegistration_ICP(iter);

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

};

