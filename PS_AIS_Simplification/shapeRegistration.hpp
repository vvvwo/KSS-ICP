/*********************************************************************************

			     Point Cloud based Shape Registration

						Updating in 2021/01/05

						   By Dr. Chenlei Lv

			The functions includes:
			1. ICP shape registration
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

using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class shapeRegistration {

private:

	vector<vector<double>> pointSource;
	vector<vector<double>> pointTarget;


public:

	vector<vector<double>> pointAlign;
	

public:

	void shapeRegistration_init(vector<vector<double>> ps, vector<vector<double>> pt) {

		pointSource = ps;
		pointTarget = pt;		

	}
	
	void shapeRegistration_init_scale(){		

		pointPipeline ppt;
		ppt.pointPipeline_init_point(pointTarget);
		pointTarget.clear();
		pointTarget = ppt.br.pointCloudData;

		pointPipeline pps;
		pps.pointPipeline_init_point(pointSource);
		pointSource.clear();
		pointSource = pps.br.pointCloudData;
		

	}

	void shapeRegistration_IIR(int iter) {

		initRegistration ir;
		ir.initRegistration_init(pointSource, pointTarget);
		vector<vector<double>> p1 = ir.pointSource;
		vector<vector<double>> p2 = ir.pointTarget;

		pointSource.clear();
		pointSource = p1;
		pointTarget.clear();
		pointTarget = p2;
		shapeRegistration_ICP(iter);

	}

	void shapeRegistration_IIR_Sim(int iter) {

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
		shapeRegistration_ICP(iter);
	
	}


	void shapeRegistration_ICP(int iter) {

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
	
	}
	
	void shapeRegistration_3DSC(int iter) {

		vector<vector<double>> pointSourceNormal;
		vector<vector<double>> pointTargetNormal;
		NormalEstimation ne;
		pointSourceNormal = ne.estimateNormal_PCL_MP_return(pointSource);
		pointTargetNormal = ne.estimateNormal_PCL_MP_return(pointTarget);

		if (pointAlign.size() > 0) {
			pointAlign.clear();		
		}
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

		//down sampling
		pcl::RandomSample<PointT> rs_src;
		rs_src.setInputCloud(cloud_s);
		rs_src.setSample(500);
		PointCloud::Ptr cloud_src(new PointCloud);
		rs_src.filter(*cloud_src);
		std::cout << "down size *cloud_src_o from " << cloud_s->size() << "to" << cloud_src->size() << endl;

		pcl::RandomSample<PointT> rs_tgt;
		rs_tgt.setInputCloud(cloud_t);
		rs_tgt.setSample(500);
		PointCloud::Ptr cloud_tgt(new PointCloud);
		rs_tgt.filter(*cloud_tgt);
		std::cout << "down size *cloud_tgt_o from " << cloud_t->size() << "to" << cloud_tgt->size() << endl;

		//normal estimation
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeS;//kdtree
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeT;//kdtree
		kdtreeS.setInputCloud(cloud_s);
		kdtreeT.setInputCloud(cloud_t);
		
		pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);			
		pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);	

		for (int i = 0; i < cloud_src->size(); i++) {
			int K = 1;
			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);			
			pcl::PointXYZ searchPoint = cloud_src->at(i);
			kdtreeS.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
			vector<double> normal_i = pointSourceNormal[pointIdxNKNSearch[0]];
			pcl::Normal ni;
			ni.normal_x = normal_i[0];
			ni.normal_y = normal_i[1];
			ni.normal_z = normal_i[2];
			cloud_src_normals->push_back(ni);
		}

		for (int i = 0; i < cloud_tgt->size(); i++) {
			int K = 1;
			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);
			pcl::PointXYZ searchPoint = cloud_tgt->at(i);
			kdtreeT.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
			vector<double> normal_i = pointTargetNormal[pointIdxNKNSearch[0]];
			pcl::Normal ni;
			ni.normal_x = normal_i[0];
			ni.normal_y = normal_i[1];
			ni.normal_z = normal_i[2];
			cloud_tgt_normals->push_back(ni);
		}

		pcl::ShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal, pcl::ShapeContext1980> sp_tgt;
		sp_tgt.setInputCloud(cloud_tgt);
		sp_tgt.setInputNormals(cloud_tgt_normals);		
		pcl::search::KdTree<PointT>::Ptr tree_tgt_sp(new pcl::search::KdTree<PointT>);
		sp_tgt.setSearchMethod(tree_tgt_sp);
		pcl::PointCloud<pcl::ShapeContext1980>::Ptr sps_tgt(new pcl::PointCloud<pcl::ShapeContext1980>());
		sp_tgt.setRadiusSearch(0.5);
		sp_tgt.compute(*sps_tgt);
		cout << "compute *cloud_tgt_sps" << endl;

		pcl::ShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal, pcl::ShapeContext1980> sp_src;
		sp_src.setInputCloud(cloud_src);
		sp_src.setInputNormals(cloud_src_normals);		
		pcl::search::KdTree<PointT>::Ptr tree_src_sp(new pcl::search::KdTree<PointT>);
		sp_src.setSearchMethod(tree_src_sp);
		pcl::PointCloud<pcl::ShapeContext1980>::Ptr sps_src(new pcl::PointCloud<pcl::ShapeContext1980>());
		sp_src.setRadiusSearch(0.5);
		sp_src.compute(*sps_src);

		cout << "compute *cloud_src_sps" << endl;

		pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::ShapeContext1980> scia;
		scia.setInputSource(cloud_src);
		scia.setInputTarget(cloud_tgt);
		scia.setSourceFeatures(sps_src);
		scia.setTargetFeatures(sps_tgt);
		//scia.setMinSampleDistance(1);
		//scia.setNumberOfSamples(2);
		//scia.setCorrespondenceRandomness(20);
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
		scia.align(*output);
		std::cout << "sac has converged:" << scia.hasConverged() << endl;
		std::cout << "score:" << scia.getFitnessScore() << endl;	
		
		pcl::transformPointCloud(*cloud_s, *output, scia.getFinalTransformation());

		for (int i = 0; i < output->size(); i++) {
			pcl::PointXYZ pi = output->at(i);
			double xi = pi.x;
			double yi = pi.y;
			double zi = pi.z;
			vector<double> ppi;
			ppi.push_back(xi);
			ppi.push_back(yi);
			ppi.push_back(zi);
			pointAlign.push_back(ppi);
		}	

		pointSource.clear();
		pointSource = pointAlign;
		pointAlign.clear();
		shapeRegistration_ICP(iter);

	}

	void shapeRegistration_FPFH(int iter) {

		if (pointAlign.size() > 0) {
			pointAlign.clear();
		}

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

		pcl::RandomSample<PointT> rs_src;
		rs_src.setInputCloud(cloud_s);
		rs_src.setSample(2000);
		PointCloud::Ptr cloud_src(new PointCloud);
		rs_src.filter(*cloud_src);
		std::cout << "down size *cloud_src_o from " << cloud_s->size() << "to" << cloud_src->size() << endl;

		pcl::RandomSample<PointT> rs_tgt;
		rs_tgt.setInputCloud(cloud_t);
		rs_tgt.setSample(2000);
		PointCloud::Ptr cloud_tgt(new PointCloud);
		rs_tgt.filter(*cloud_tgt);
		std::cout << "down size *cloud_tgt_o from " << cloud_t->size() << "to" << cloud_tgt->size() << endl;

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_src;
		ne_src.setInputCloud(cloud_src);
		pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree< pcl::PointXYZ>());
		ne_src.setSearchMethod(tree_src);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
		ne_src.setRadiusSearch(0.02);
		ne_src.compute(*cloud_src_normals);

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_tgt;
		ne_tgt.setInputCloud(cloud_tgt);
		pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree< pcl::PointXYZ>());
		ne_tgt.setSearchMethod(tree_tgt);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
		//ne_tgt.setKSearch(20);
		ne_tgt.setRadiusSearch(0.02);
		ne_tgt.compute(*cloud_tgt_normals);

		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_src;
		fpfh_src.setInputCloud(cloud_src);
		fpfh_src.setInputNormals(cloud_src_normals);
		pcl::search::KdTree<PointT>::Ptr tree_src_fpfh(new pcl::search::KdTree<PointT>);
		fpfh_src.setSearchMethod(tree_src_fpfh);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
		fpfh_src.setRadiusSearch(0.05);
		fpfh_src.compute(*fpfhs_src);
		std::cout << "compute *cloud_src fpfh" << endl;

		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_tgt;
		fpfh_tgt.setInputCloud(cloud_tgt);
		fpfh_tgt.setInputNormals(cloud_tgt_normals);
		pcl::search::KdTree<PointT>::Ptr tree_tgt_fpfh(new pcl::search::KdTree<PointT>);
		fpfh_tgt.setSearchMethod(tree_tgt_fpfh);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
		fpfh_tgt.setRadiusSearch(0.05);
		fpfh_tgt.compute(*fpfhs_tgt);
		std::cout << "compute *cloud_tgt fpfh" << endl;

		pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
		scia.setInputSource(cloud_src);
		scia.setInputTarget(cloud_tgt);
		scia.setSourceFeatures(fpfhs_src);
		scia.setTargetFeatures(fpfhs_tgt);
		//scia.setMinSampleDistance(1);
		//scia.setNumberOfSamples(2);
		//scia.setCorrespondenceRandomness(20);
		pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
		scia.align(*output);
		std::cout << "sac has converged:" << scia.hasConverged() << endl;
		std::cout << "score:" << scia.getFitnessScore() << endl;

		pcl::transformPointCloud(*cloud_s, *output, scia.getFinalTransformation());

		for (int i = 0; i < output->size(); i++) {
			pcl::PointXYZ pi = output->at(i);
			double xi = pi.x;
			double yi = pi.y;
			double zi = pi.z;
			vector<double> ppi;
			ppi.push_back(xi);
			ppi.push_back(yi);
			ppi.push_back(zi);
			pointAlign.push_back(ppi);
		}
		pointSource.clear();
		pointSource = pointAlign;
		pointAlign.clear();
		shapeRegistration_ICP(iter);
	
	}

	void shapeRegistration_NDT(int iter) {

		if (pointAlign.size() > 0) {
			pointAlign.clear();
		}

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

		/*
		pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
		voxel_grid.setLeafSize(0.012, 0.012, 0.012);
		voxel_grid.setInputCloud(cloud_s);
		PointCloud::Ptr cloud_src(new PointCloud);
		voxel_grid.filter(*cloud_src);
		std::cout << "down size *cloud_src_o from " << cloud_s->size() << "to" << cloud_src->size() << endl;
		
		pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_2;
		voxel_grid_2.setLeafSize(0.012, 0.012, 0.012);
		voxel_grid_2.setInputCloud(cloud_t);
		PointCloud::Ptr cloud_tgt(new PointCloud);
		voxel_grid_2.filter(*cloud_tgt);
		std::cout << "down size *cloud_tgt_o.pcd from " << cloud_t->size() << "to" << cloud_tgt->size() << endl;
		*/

		pcl::RandomSample<PointT> rs_src;
		rs_src.setInputCloud(cloud_s);
		rs_src.setSample(1000);
		PointCloud::Ptr cloud_src(new PointCloud);
		rs_src.filter(*cloud_src);
		std::cout << "down size *cloud_src_o from " << cloud_s->size() << "to" << cloud_src->size() << endl;

		pcl::RandomSample<PointT> rs_tgt;
		rs_tgt.setInputCloud(cloud_t);
		rs_tgt.setSample(1000);
		PointCloud::Ptr cloud_tgt(new PointCloud);
		rs_tgt.filter(*cloud_tgt);
		std::cout << "down size *cloud_tgt_o from " << cloud_t->size() << "to" << cloud_tgt->size() << endl;


		pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
		PointCloud::Ptr cloud_ndt(new PointCloud);
		ndt.setTransformationEpsilon(0.01);
		ndt.setStepSize(0.05);
		ndt.setResolution(3);//Íø¸ñ·Ö±æÂÊ
		ndt.setMaximumIterations(100);
		ndt.setInputSource(cloud_src);
		ndt.setInputTarget(cloud_tgt);
		Eigen::AngleAxisf init_rotation(M_PI / 4, Eigen::Vector3f::UnitZ());
		Eigen::Translation3f init_transtion(0, 0, 0);
		Eigen::Matrix4f init_guess = (init_transtion * init_rotation).matrix();
		ndt.align(*cloud_ndt, init_guess);
		pcl::transformPointCloud(*cloud_s, *cloud_ndt, ndt.getFinalTransformation());
		
		for (int i = 0; i < cloud_ndt->size(); i++) {
			pcl::PointXYZ pi = cloud_ndt->at(i);
			double xi = pi.x;
			double yi = pi.y;
			double zi = pi.z;
			vector<double> ppi;
			ppi.push_back(xi);
			ppi.push_back(yi);
			ppi.push_back(zi);
			pointAlign.push_back(ppi);
		}
		pointSource.clear();
		pointSource = pointAlign;
		pointAlign.clear();
		shapeRegistration_ICP(iter);
	
	}

	void shapeRegistration_PFH(int iter) {

		if (pointAlign.size() > 0) {
			pointAlign.clear();
		}

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

		pcl::RandomSample<PointT> rs_src;
		rs_src.setInputCloud(cloud_s);
		rs_src.setSample(2000);
		PointCloud::Ptr cloud_src(new PointCloud);
		rs_src.filter(*cloud_src);
		std::cout << "down size *cloud_src_o from " << cloud_s->size() << "to" << cloud_src->size() << endl;

		pcl::RandomSample<PointT> rs_tgt;
		rs_tgt.setInputCloud(cloud_t);
		rs_tgt.setSample(2000);
		PointCloud::Ptr cloud_tgt(new PointCloud);
		rs_tgt.filter(*cloud_tgt);
		std::cout << "down size *cloud_tgt_o from " << cloud_t->size() << "to" << cloud_tgt->size() << endl;

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_src;
		ne_src.setInputCloud(cloud_src);
		pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree< pcl::PointXYZ>());
		ne_src.setSearchMethod(tree_src);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
		ne_src.setRadiusSearch(0.02);
		ne_src.compute(*cloud_src_normals);

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_tgt;
		ne_tgt.setInputCloud(cloud_tgt);
		pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree< pcl::PointXYZ>());
		ne_tgt.setSearchMethod(tree_tgt);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
		//ne_tgt.setKSearch(20);
		ne_tgt.setRadiusSearch(0.02);
		ne_tgt.compute(*cloud_tgt_normals);

		pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_src;
		pfh_src.setInputCloud(cloud_src);
		pfh_src.setInputNormals(cloud_src_normals);
		pcl::search::KdTree<PointT>::Ptr tree_src_fpfh(new pcl::search::KdTree<PointT>);
		pfh_src.setSearchMethod(tree_src_fpfh);
		pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs_src(new pcl::PointCloud<pcl::PFHSignature125>());
		pfh_src.setRadiusSearch(0.05);
		pfh_src.compute(*pfhs_src);
		std::cout << "compute *cloud_src pfh" << endl;

		pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_tgt;
		pfh_src.setInputCloud(cloud_tgt);
		pfh_src.setInputNormals(cloud_tgt_normals);
		pcl::search::KdTree<PointT>::Ptr tree_tgt_fpfh(new pcl::search::KdTree<PointT>);
		pfh_src.setSearchMethod(tree_tgt_fpfh);
		pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs_tgt(new pcl::PointCloud<pcl::PFHSignature125>());
		pfh_src.setRadiusSearch(0.05);
		pfh_src.compute(*pfhs_tgt);
		std::cout << "compute *cloud_tgt pfh" << endl;

		pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::PFHSignature125> scia;
		scia.setInputSource(cloud_src);
		scia.setInputTarget(cloud_tgt);
		scia.setSourceFeatures(pfhs_src);
		scia.setTargetFeatures(pfhs_tgt);
		//scia.setMinSampleDistance(1);
		//scia.setNumberOfSamples(2);
		//scia.setCorrespondenceRandomness(20);
		pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
		scia.align(*output);
		std::cout << "sac has converged:" << scia.hasConverged() << endl;
		std::cout << "score:" << scia.getFitnessScore() << endl;

		pcl::transformPointCloud(*cloud_s, *output, scia.getFinalTransformation());

		for (int i = 0; i < output->size(); i++) {
			pcl::PointXYZ pi = output->at(i);
			double xi = pi.x;
			double yi = pi.y;
			double zi = pi.z;
			vector<double> ppi;
			ppi.push_back(xi);
			ppi.push_back(yi);
			ppi.push_back(zi);
			pointAlign.push_back(ppi);
		}
		pointSource.clear();
		pointSource = pointAlign;
		pointAlign.clear();
		shapeRegistration_ICP(iter);

	}

	void shapeRegistration_Evaluate(int Number) {

		if (Number == 1) {//ICP
			cout << "iter:" << 1 << endl;
			shapeRegistration_ICP(1);
			cout << "iter:" << 2 << endl;
			shapeRegistration_ICP(2);
			cout << "iter:" << 3 << endl;
			shapeRegistration_ICP(3);
			cout << "iter:" << 4 << endl;
			shapeRegistration_ICP(4);
			cout << "iter:" << 5 << endl;
			shapeRegistration_ICP(5);
			cout << "iter:" << 7 << endl;
			shapeRegistration_ICP(7);
			cout << "iter:" << 10 << endl;
			shapeRegistration_ICP(10);
			cout << "iter:" << 15 << endl;
			shapeRegistration_ICP(15);
			cout << "iter:" << 20 << endl;
			shapeRegistration_ICP(20);
			cout << "iter:" << 40 << endl;
			shapeRegistration_ICP(40);

		
		}
		else if (Number == 2) {//NDT
			shapeRegistration_NDT_P();
			cout << "iter:" << 1 << endl;
			shapeRegistration_ICP(1);
			cout << "iter:" << 2 << endl;
			shapeRegistration_ICP(2);
			cout << "iter:" << 3 << endl;
			shapeRegistration_ICP(3);
			cout << "iter:" << 4 << endl;
			shapeRegistration_ICP(4);
			cout << "iter:" << 5 << endl;
			shapeRegistration_ICP(5);
			cout << "iter:" << 7 << endl;
			shapeRegistration_ICP(7);
			cout << "iter:" << 10 << endl;
			shapeRegistration_ICP(10);
			cout << "iter:" << 15 << endl;
			shapeRegistration_ICP(15);
			cout << "iter:" << 20 << endl;
			shapeRegistration_ICP(20);
			cout << "iter:" << 40 << endl;
			shapeRegistration_ICP(40);
		
		}
		else if (Number == 3) {//3DSC
			shapeRegistration_3DSC_P();
			cout << "iter:" << 1 << endl;
			shapeRegistration_ICP(1);
			cout << "iter:" << 2 << endl;
			shapeRegistration_ICP(2);
			cout << "iter:" << 3 << endl;
			shapeRegistration_ICP(3);
			cout << "iter:" << 4 << endl;
			shapeRegistration_ICP(4);
			cout << "iter:" << 5 << endl;
			shapeRegistration_ICP(5);
			cout << "iter:" << 7 << endl;
			shapeRegistration_ICP(7);
			cout << "iter:" << 10 << endl;
			shapeRegistration_ICP(10);
			cout << "iter:" << 15 << endl;
			shapeRegistration_ICP(15);
			cout << "iter:" << 20 << endl;
			shapeRegistration_ICP(20);
			cout << "iter:" << 40 << endl;
			shapeRegistration_ICP(40);
		
		}
		else if (Number == 4) {//PFH
			shapeRegistration_PFH_P();
			cout << "iter:" << 1 << endl;
			shapeRegistration_ICP(1);
			cout << "iter:" << 2 << endl;
			shapeRegistration_ICP(2);
			cout << "iter:" << 3 << endl;
			shapeRegistration_ICP(3);
			cout << "iter:" << 4 << endl;
			shapeRegistration_ICP(4);
			cout << "iter:" << 5 << endl;
			shapeRegistration_ICP(5);
			cout << "iter:" << 7 << endl;
			shapeRegistration_ICP(7);
			cout << "iter:" << 10 << endl;
			shapeRegistration_ICP(10);
			cout << "iter:" << 15 << endl;
			shapeRegistration_ICP(15);
			cout << "iter:" << 20 << endl;
			shapeRegistration_ICP(20);
			cout << "iter:" << 40 << endl;
			shapeRegistration_ICP(40);

		}
		else if (Number == 5) {//FPFH
			shapeRegistration_FPFH_P();
			cout << "iter:" << 1 << endl;
			shapeRegistration_ICP(1);
			cout << "iter:" << 2 << endl;
			shapeRegistration_ICP(2);
			cout << "iter:" << 3 << endl;
			shapeRegistration_ICP(3);
			cout << "iter:" << 4 << endl;
			shapeRegistration_ICP(4);
			cout << "iter:" << 5 << endl;
			shapeRegistration_ICP(5);
			cout << "iter:" << 7 << endl;
			shapeRegistration_ICP(7);
			cout << "iter:" << 10 << endl;
			shapeRegistration_ICP(10);
			cout << "iter:" << 15 << endl;
			shapeRegistration_ICP(15);
			cout << "iter:" << 20 << endl;
			shapeRegistration_ICP(20);
			cout << "iter:" << 40 << endl;
			shapeRegistration_ICP(40);

		}
		else if (Number == 6) {//Our
		    shapeRegistration_IIR_Sim_P();
		    cout << "iter:" << 1 << endl;
			shapeRegistration_ICP(1);
			cout << "iter:" << 2 << endl;
			shapeRegistration_ICP(2);
			cout << "iter:" << 3 << endl;
			shapeRegistration_ICP(3);
			cout << "iter:" << 4 << endl;
			shapeRegistration_ICP(4);
			cout << "iter:" << 5 << endl;
			shapeRegistration_ICP(5);
			cout << "iter:" << 7 << endl;
			shapeRegistration_ICP(7);
			cout << "iter:" << 10 << endl;
			shapeRegistration_ICP(10);
			cout << "iter:" << 15 << endl;
			shapeRegistration_ICP(15);
			cout << "iter:" << 20 << endl;
			shapeRegistration_ICP(20);
			cout << "iter:" << 40 << endl;
			shapeRegistration_ICP(40);
		}
	
	
	}

	vector<vector<double>> shapeRegistration_pointSource() {

		return pointSource;

	}
	vector<vector<double>> shapeRegistration_pointTarget() {

		return pointTarget;
	
	}

private:	

	void shapeRegistration_3DSC_P() {

		vector<vector<double>> pointSourceNormal;
		vector<vector<double>> pointTargetNormal;
		NormalEstimation ne;
		pointSourceNormal = ne.estimateNormal_PCL_MP_return(pointSource);
		pointTargetNormal = ne.estimateNormal_PCL_MP_return(pointTarget);

		if (pointAlign.size() > 0) {
			pointAlign.clear();
		}
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

		//down sampling
		pcl::RandomSample<PointT> rs_src;
		rs_src.setInputCloud(cloud_s);
		rs_src.setSample(500);
		PointCloud::Ptr cloud_src(new PointCloud);
		rs_src.filter(*cloud_src);
		std::cout << "down size *cloud_src_o from " << cloud_s->size() << "to" << cloud_src->size() << endl;

		pcl::RandomSample<PointT> rs_tgt;
		rs_tgt.setInputCloud(cloud_t);
		rs_tgt.setSample(500);
		PointCloud::Ptr cloud_tgt(new PointCloud);
		rs_tgt.filter(*cloud_tgt);
		std::cout << "down size *cloud_tgt_o from " << cloud_t->size() << "to" << cloud_tgt->size() << endl;

		//normal estimation
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeS;//kdtree
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeT;//kdtree
		kdtreeS.setInputCloud(cloud_s);
		kdtreeT.setInputCloud(cloud_t);

		pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);

		for (int i = 0; i < cloud_src->size(); i++) {
			int K = 1;
			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);
			pcl::PointXYZ searchPoint = cloud_src->at(i);
			kdtreeS.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
			vector<double> normal_i = pointSourceNormal[pointIdxNKNSearch[0]];
			pcl::Normal ni;
			ni.normal_x = normal_i[0];
			ni.normal_y = normal_i[1];
			ni.normal_z = normal_i[2];
			cloud_src_normals->push_back(ni);
		}

		for (int i = 0; i < cloud_tgt->size(); i++) {
			int K = 1;
			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);
			pcl::PointXYZ searchPoint = cloud_tgt->at(i);
			kdtreeT.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
			vector<double> normal_i = pointTargetNormal[pointIdxNKNSearch[0]];
			pcl::Normal ni;
			ni.normal_x = normal_i[0];
			ni.normal_y = normal_i[1];
			ni.normal_z = normal_i[2];
			cloud_tgt_normals->push_back(ni);
		}

		pcl::ShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal, pcl::ShapeContext1980> sp_tgt;
		sp_tgt.setInputCloud(cloud_tgt);
		sp_tgt.setInputNormals(cloud_tgt_normals);
		pcl::search::KdTree<PointT>::Ptr tree_tgt_sp(new pcl::search::KdTree<PointT>);
		sp_tgt.setSearchMethod(tree_tgt_sp);
		pcl::PointCloud<pcl::ShapeContext1980>::Ptr sps_tgt(new pcl::PointCloud<pcl::ShapeContext1980>());
		sp_tgt.setRadiusSearch(0.5);
		sp_tgt.compute(*sps_tgt);
		cout << "compute *cloud_tgt_sps" << endl;

		pcl::ShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal, pcl::ShapeContext1980> sp_src;
		sp_src.setInputCloud(cloud_src);
		sp_src.setInputNormals(cloud_src_normals);
		pcl::search::KdTree<PointT>::Ptr tree_src_sp(new pcl::search::KdTree<PointT>);
		sp_src.setSearchMethod(tree_src_sp);
		pcl::PointCloud<pcl::ShapeContext1980>::Ptr sps_src(new pcl::PointCloud<pcl::ShapeContext1980>());
		sp_src.setRadiusSearch(0.5);
		sp_src.compute(*sps_src);

		cout << "compute *cloud_src_sps" << endl;

		pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::ShapeContext1980> scia;
		scia.setInputSource(cloud_src);
		scia.setInputTarget(cloud_tgt);
		scia.setSourceFeatures(sps_src);
		scia.setTargetFeatures(sps_tgt);
		//scia.setMinSampleDistance(1);
		//scia.setNumberOfSamples(2);
		//scia.setCorrespondenceRandomness(20);

		pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
		scia.align(*output);
		std::cout << "sac has converged:" << scia.hasConverged() << endl;
		std::cout << "score:" << scia.getFitnessScore() << endl;

		pcl::transformPointCloud(*cloud_s, *output, scia.getFinalTransformation());

		for (int i = 0; i < output->size(); i++) {
			pcl::PointXYZ pi = output->at(i);
			double xi = pi.x;
			double yi = pi.y;
			double zi = pi.z;
			vector<double> ppi;
			ppi.push_back(xi);
			ppi.push_back(yi);
			ppi.push_back(zi);
			pointAlign.push_back(ppi);
		}

		pointSource.clear();
		pointSource = pointAlign;
		pointAlign.clear();
		

	}

	void shapeRegistration_FPFH_P() {

		if (pointAlign.size() > 0) {
			pointAlign.clear();
		}

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

		pcl::RandomSample<PointT> rs_src;
		rs_src.setInputCloud(cloud_s);
		rs_src.setSample(2000);
		PointCloud::Ptr cloud_src(new PointCloud);
		rs_src.filter(*cloud_src);
		std::cout << "down size *cloud_src_o from " << cloud_s->size() << "to" << cloud_src->size() << endl;

		pcl::RandomSample<PointT> rs_tgt;
		rs_tgt.setInputCloud(cloud_t);
		rs_tgt.setSample(2000);
		PointCloud::Ptr cloud_tgt(new PointCloud);
		rs_tgt.filter(*cloud_tgt);
		std::cout << "down size *cloud_tgt_o from " << cloud_t->size() << "to" << cloud_tgt->size() << endl;

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_src;
		ne_src.setInputCloud(cloud_src);
		pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree< pcl::PointXYZ>());
		ne_src.setSearchMethod(tree_src);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
		ne_src.setRadiusSearch(0.02);
		ne_src.compute(*cloud_src_normals);

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_tgt;
		ne_tgt.setInputCloud(cloud_tgt);
		pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree< pcl::PointXYZ>());
		ne_tgt.setSearchMethod(tree_tgt);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
		//ne_tgt.setKSearch(20);
		ne_tgt.setRadiusSearch(0.02);
		ne_tgt.compute(*cloud_tgt_normals);

		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_src;
		fpfh_src.setInputCloud(cloud_src);
		fpfh_src.setInputNormals(cloud_src_normals);
		pcl::search::KdTree<PointT>::Ptr tree_src_fpfh(new pcl::search::KdTree<PointT>);
		fpfh_src.setSearchMethod(tree_src_fpfh);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
		fpfh_src.setRadiusSearch(0.05);
		fpfh_src.compute(*fpfhs_src);
		std::cout << "compute *cloud_src fpfh" << endl;

		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_tgt;
		fpfh_tgt.setInputCloud(cloud_tgt);
		fpfh_tgt.setInputNormals(cloud_tgt_normals);
		pcl::search::KdTree<PointT>::Ptr tree_tgt_fpfh(new pcl::search::KdTree<PointT>);
		fpfh_tgt.setSearchMethod(tree_tgt_fpfh);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
		fpfh_tgt.setRadiusSearch(0.05);
		fpfh_tgt.compute(*fpfhs_tgt);
		std::cout << "compute *cloud_tgt fpfh" << endl;

		pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
		scia.setInputSource(cloud_src);
		scia.setInputTarget(cloud_tgt);
		scia.setSourceFeatures(fpfhs_src);
		scia.setTargetFeatures(fpfhs_tgt);
		//scia.setMinSampleDistance(1);
		//scia.setNumberOfSamples(2);
		//scia.setCorrespondenceRandomness(20);
		pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
		scia.align(*output);
		std::cout << "sac has converged:" << scia.hasConverged() << endl;
		std::cout << "score:" << scia.getFitnessScore() << endl;

		pcl::transformPointCloud(*cloud_s, *output, scia.getFinalTransformation());

		for (int i = 0; i < output->size(); i++) {
			pcl::PointXYZ pi = output->at(i);
			double xi = pi.x;
			double yi = pi.y;
			double zi = pi.z;
			vector<double> ppi;
			ppi.push_back(xi);
			ppi.push_back(yi);
			ppi.push_back(zi);
			pointAlign.push_back(ppi);
		}
		pointSource.clear();
		pointSource = pointAlign;
		pointAlign.clear();
		

	}

	void shapeRegistration_NDT_P() {

		if (pointAlign.size() > 0) {
			pointAlign.clear();
		}

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

		/*
		pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
		voxel_grid.setLeafSize(0.012, 0.012, 0.012);
		voxel_grid.setInputCloud(cloud_s);
		PointCloud::Ptr cloud_src(new PointCloud);
		voxel_grid.filter(*cloud_src);
		std::cout << "down size *cloud_src_o from " << cloud_s->size() << "to" << cloud_src->size() << endl;

		pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_2;
		voxel_grid_2.setLeafSize(0.012, 0.012, 0.012);
		voxel_grid_2.setInputCloud(cloud_t);
		PointCloud::Ptr cloud_tgt(new PointCloud);
		voxel_grid_2.filter(*cloud_tgt);
		std::cout << "down size *cloud_tgt_o.pcd from " << cloud_t->size() << "to" << cloud_tgt->size() << endl;
		*/

		pcl::RandomSample<PointT> rs_src;
		rs_src.setInputCloud(cloud_s);
		rs_src.setSample(1000);
		PointCloud::Ptr cloud_src(new PointCloud);
		rs_src.filter(*cloud_src);
		std::cout << "down size *cloud_src_o from " << cloud_s->size() << "to" << cloud_src->size() << endl;

		pcl::RandomSample<PointT> rs_tgt;
		rs_tgt.setInputCloud(cloud_t);
		rs_tgt.setSample(1000);
		PointCloud::Ptr cloud_tgt(new PointCloud);
		rs_tgt.filter(*cloud_tgt);
		std::cout << "down size *cloud_tgt_o from " << cloud_t->size() << "to" << cloud_tgt->size() << endl;


		pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
		PointCloud::Ptr cloud_ndt(new PointCloud);
		ndt.setTransformationEpsilon(0.01);
		ndt.setStepSize(0.05);
		ndt.setResolution(3);//Íø¸ñ·Ö±æÂÊ
		ndt.setMaximumIterations(100);
		ndt.setInputSource(cloud_src);
		ndt.setInputTarget(cloud_tgt);
		Eigen::AngleAxisf init_rotation(M_PI / 4, Eigen::Vector3f::UnitZ());
		Eigen::Translation3f init_transtion(0, 0, 0);
		Eigen::Matrix4f init_guess = (init_transtion * init_rotation).matrix();
		ndt.align(*cloud_ndt, init_guess);
		pcl::transformPointCloud(*cloud_s, *cloud_ndt, ndt.getFinalTransformation());

		for (int i = 0; i < cloud_ndt->size(); i++) {
			pcl::PointXYZ pi = cloud_ndt->at(i);
			double xi = pi.x;
			double yi = pi.y;
			double zi = pi.z;
			vector<double> ppi;
			ppi.push_back(xi);
			ppi.push_back(yi);
			ppi.push_back(zi);
			pointAlign.push_back(ppi);
		}
		pointSource.clear();
		pointSource = pointAlign;
		pointAlign.clear();
		

	}

	void shapeRegistration_PFH_P() {

		if (pointAlign.size() > 0) {
			pointAlign.clear();
		}

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

		pcl::RandomSample<PointT> rs_src;
		rs_src.setInputCloud(cloud_s);
		rs_src.setSample(2000);
		PointCloud::Ptr cloud_src(new PointCloud);
		rs_src.filter(*cloud_src);
		std::cout << "down size *cloud_src_o from " << cloud_s->size() << "to" << cloud_src->size() << endl;

		pcl::RandomSample<PointT> rs_tgt;
		rs_tgt.setInputCloud(cloud_t);
		rs_tgt.setSample(2000);
		PointCloud::Ptr cloud_tgt(new PointCloud);
		rs_tgt.filter(*cloud_tgt);
		std::cout << "down size *cloud_tgt_o from " << cloud_t->size() << "to" << cloud_tgt->size() << endl;

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_src;
		ne_src.setInputCloud(cloud_src);
		pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree< pcl::PointXYZ>());
		ne_src.setSearchMethod(tree_src);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
		ne_src.setRadiusSearch(0.02);
		ne_src.compute(*cloud_src_normals);

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_tgt;
		ne_tgt.setInputCloud(cloud_tgt);
		pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree< pcl::PointXYZ>());
		ne_tgt.setSearchMethod(tree_tgt);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
		//ne_tgt.setKSearch(20);
		ne_tgt.setRadiusSearch(0.02);
		ne_tgt.compute(*cloud_tgt_normals);

		pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_src;
		pfh_src.setInputCloud(cloud_src);
		pfh_src.setInputNormals(cloud_src_normals);
		pcl::search::KdTree<PointT>::Ptr tree_src_fpfh(new pcl::search::KdTree<PointT>);
		pfh_src.setSearchMethod(tree_src_fpfh);
		pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs_src(new pcl::PointCloud<pcl::PFHSignature125>());
		pfh_src.setRadiusSearch(0.05);
		pfh_src.compute(*pfhs_src);
		std::cout << "compute *cloud_src pfh" << endl;

		pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_tgt;
		pfh_src.setInputCloud(cloud_tgt);
		pfh_src.setInputNormals(cloud_tgt_normals);
		pcl::search::KdTree<PointT>::Ptr tree_tgt_fpfh(new pcl::search::KdTree<PointT>);
		pfh_src.setSearchMethod(tree_tgt_fpfh);
		pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs_tgt(new pcl::PointCloud<pcl::PFHSignature125>());
		pfh_src.setRadiusSearch(0.05);
		pfh_src.compute(*pfhs_tgt);
		std::cout << "compute *cloud_tgt pfh" << endl;

		pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::PFHSignature125> scia;
		scia.setInputSource(cloud_src);
		scia.setInputTarget(cloud_tgt);
		scia.setSourceFeatures(pfhs_src);
		scia.setTargetFeatures(pfhs_tgt);
		//scia.setMinSampleDistance(1);
		//scia.setNumberOfSamples(2);
		//scia.setCorrespondenceRandomness(20);
		pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
		scia.align(*output);
		std::cout << "sac has converged:" << scia.hasConverged() << endl;
		std::cout << "score:" << scia.getFitnessScore() << endl;

		pcl::transformPointCloud(*cloud_s, *output, scia.getFinalTransformation());

		for (int i = 0; i < output->size(); i++) {
			pcl::PointXYZ pi = output->at(i);
			double xi = pi.x;
			double yi = pi.y;
			double zi = pi.z;
			vector<double> ppi;
			ppi.push_back(xi);
			ppi.push_back(yi);
			ppi.push_back(zi);
			pointAlign.push_back(ppi);
		}
		pointSource.clear();
		pointSource = pointAlign;
		pointAlign.clear();
		

	}

	void shapeRegistration_IIR_Sim_P() {

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
