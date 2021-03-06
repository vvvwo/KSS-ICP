#pragma once
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/vcm_estimate_edges.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_off_points.h>
#include <utility> // defines std::pair
#include <vector>
#include <fstream>
// Types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point3;
typedef Kernel::Vector_3 Vector3;
// Point with normal vector stored in a std::pair.
typedef std::pair<Point3, Vector3> PointVectorPair;
typedef std::vector<PointVectorPair> PointList;
typedef std::array<double, 6> Covariance;

using namespace std;

class CGAL_VCM {

public:

	vector<vector<double>> CGAL_VCM_BorderOut(vector<vector<double>> pData, vector<vector<double>> vData, double threshold) {
		clock_t t1;
		clock_t t2;

		cout << "CGAL_VCM Start!" << endl;
		vector<vector<double>> result;

		std::list<PointVectorPair> points;

		for (int i = 0; i < pData.size(); i++) {

			Point3 pi(pData[i][0], pData[i][1], pData[i][2]);
			Vector3 pni(vData[i][0], vData[i][1], vData[i][2]);
			PointVectorPair ppi;
			ppi.first = pi;
			ppi.second = pni;
			points.push_back(ppi);

		}

		// Estimates covariance matrices per points.
		double R = 0.1, r = 0.02;
		std::vector<Covariance> cov;
		CGAL::First_of_pair_property_map<PointVectorPair> point_map;

		t1 = clock();

		CGAL::compute_vcm(points, cov, R, r,
			CGAL::parameters::point_map(point_map).geom_traits(Kernel()));

		t2 = clock();

		// Find the points on the edges.
		// Note that this step is not expensive and can be done several time to get better results
		//double threshold = 0.3;
		int i = 0;
		for (const PointVectorPair& p : points)
		{

			if (CGAL::vcm_is_on_feature_edge(cov[i], threshold)) {

				Point3 pi = p.first;
				Vector3 pni = p.second;
				vector<double> result_i;
				result_i.push_back(pi[0]);
				result_i.push_back(pi[1]);
				result_i.push_back(pi[2]);
				result_i.push_back(pni[0]);
				result_i.push_back(pni[1]);
				result_i.push_back(pni[2]);
				result.push_back(result_i);

			}
			++i;
		}
		std::cout << "VCM running time:" << (t2 - t1) / 1000.0 << "s" << endl;
		cout << "CGAL_VCM Finish! Parameter: R = " << R << "," << "r = " << r << "," << "threshold = " << threshold << endl;
		return result;

	}

	vector<int> CGAL_VCM_BorderOutIndex(vector<vector<double>> pData, vector<vector<double>> vData, double threshold) {

		cout << "CGAL_VCM Start!" << endl;
		vector<int> pIndex;
		std::list<PointVectorPair> points;

		for (int i = 0; i < pData.size(); i++) {

			Point3 pi(pData[i][0], pData[i][1], pData[i][2]);
			Vector3 pni(vData[i][0], vData[i][1], vData[i][2]);
			PointVectorPair ppi;
			ppi.first = pi;
			ppi.second = pni;
			points.push_back(ppi);

		}
		// Estimates covariance matrices per points.
		double R = 0.1, r = 0.01;
		std::vector<Covariance> cov;
		CGAL::First_of_pair_property_map<PointVectorPair> point_map;
		CGAL::compute_vcm(points, cov, R, r,
			CGAL::parameters::point_map(point_map).geom_traits(Kernel()));
		// Find the points on the edges.
		// Note that this step is not expensive and can be done several time to get better results		
		int i = 0;
		for (const PointVectorPair& p : points)
		{
			if (CGAL::vcm_is_on_feature_edge(cov[i], threshold)) {
				pIndex.push_back(i);
			}
			++i;
		}
		cout << "CGAL_VCM Finish!" << endl;
		return pIndex;
	}



};



