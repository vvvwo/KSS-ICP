/*********************************************************************************

			     Point cloud Preprocessing Pipeline (New Vision!)
							   

							 Updating in 2020/10/26

							   By Dr. Chenlei Lv

			The functions includes:
			1. Load a point cloud from the file (txt/xyz/ply/obj/off)
			2. Compute the normal vectors
			3. Orctree down-sampling if needed			
			4. Denoising if needed			
			5. Construct a voxel structure for the point cloud

*********************************************************************************/
#pragma once

#include "LoadPointCloud.hpp"
#include "normalCompute.hpp"
#include "Method_Octree.hpp"
#include "ballRegionCompute.hpp"

class pointPipeline {

public:

	BallRegion br;//neibor structure construct	

private:

	LoadPointCloud lpc;//load different kinds of files to achieve point cloud
	NormalEstimation ne;//normal estimation for point cloud
	PCL_octree pot;
	

public:

	void pointPipeline_init(string filepath, bool denoisingJudge) {
		
		cout << "pointPipeline_init run!" << endl;

		vector<vector<double>> pdata; //point
		vector<vector<double>> ndata; //normal

		//1. load the point cloud
		lpc.PointCloud_Load(filepath);//support obj, off, ply, xyz, txt

		//2. normal estimation
		int index = filepath.find_last_of(".");
		string FileNormal = filepath.substr(0, index) + ".normal";
		ne.estimateNormal_init(FileNormal);
		if (ne.normalLoad()) {
			cout << "pointProcessPipeline_init: normal file exist and the data are loaded." << endl;
		}		
		else {
			cout << "pcl normal start." << endl;
			//ne.estimateNormal(lpc.pointSet_uniform, 'j');	
			ne.estimateNormal_PCL_MP(lpc.pointSet_uniform);
		}
		pdata = lpc.pointSet_uniform;
		ndata = ne.normalVector;		
		
		vector<int> borderdata = pointPipeline_Border(pdata);
		br.BallRegion_init(pdata, ndata, borderdata);		

	}

	void pointPipeline_init_point(vector<vector<double>> pointData) {

		cout << "pointPipeline_init run!" << endl;

		vector<vector<double>> pdata; //point
		vector<vector<double>> ndata; //normal

		//1. load the point cloud
		lpc.PointCloud_Load(pointData);//support obj, off, ply, xyz, txt

		//2. normal estimation		
		pdata = lpc.pointSet_uniform;
		ndata = ne.estimateNormal_PCL_MP_return(pdata);

		vector<int> borderdata = pointPipeline_Border(pdata);
		br.BallRegion_init(pdata, ndata, borderdata);	
	}

	void pointPipeline_init_point_withoutUniform(vector<vector<double>> pointData) {

		cout << "pointPipeline_init run!" << endl;

		vector<vector<double>> pdata; //point
		//vector<vector<double>> ndata; //normal		

		//2. normal estimation		
		pdata = pointData;
		//ndata = ne.estimateNormal_PCL_MP_return(pdata);

		vector<int> borderdata = pointPipeline_Border(pdata);
		br.BallRegion_init_withoutNormal(pdata, borderdata);
	}

private:

	vector<int> pointPipeline_Border(vector<vector<double>> pointSet) {

		double maxx, maxy, maxz, minx, miny, minz;		
		maxx = minx = pointSet[0][0];
		maxy = miny = pointSet[0][1];
		maxz = minz = pointSet[0][2];
		int indexMaxX = 0;
		int indexMaxY = 0;
		int indexMaxZ = 0;
		int indexMinX = 0;
		int indexMinY = 0;
		int indexMinZ = 0;
		for (int i = 0; i < pointSet.size(); i++) {
			double xi = pointSet[i][0];
			double yi = pointSet[i][1];
			double zi = pointSet[i][2];
			if (xi < minx) {
				minx = xi;
				indexMinX = i;
			}
			if (xi > maxx) {
				maxx = xi;
				indexMaxX = i;
			}
			if (yi < miny) {
				miny = yi;
				indexMinY = i;
			}
			if (yi > maxy) {
				maxy = yi;
				indexMaxY = i;
			}
			if (zi < minz) {
				minz = zi;
				indexMinZ = i;
			}
			if (zi > maxz) {
				maxz = zi;
				indexMaxZ = i;
			}
		}
		
		vector<int> indexBorder;

		indexBorder.push_back(indexMinX);
		indexBorder.push_back(indexMinY);
		indexBorder.push_back(indexMinZ);
		indexBorder.push_back(indexMaxX);
		indexBorder.push_back(indexMaxY);
		indexBorder.push_back(indexMaxZ);

		return indexBorder;

	}
		

};