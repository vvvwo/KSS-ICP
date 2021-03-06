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

//CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/hierarchy_simplify_point_set.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/wlop_simplify_and_regularize_point_set.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point3;

// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

class simplification_Method_CGAL {

public:

	vector<vector<double>> point_Cloud;
	vector<vector<double>> point_normal;
	double radius;

public:

	void simplification_Method_CGAL_init(vector<vector<double>> point_Cloud_input, vector<vector<double>> point_normal_input, double radius_input) {

		point_Cloud = point_Cloud_input;
		point_normal = point_normal_input;
		radius = radius_input;

	}

	void simplification_Method_CGAL_init(vector<vector<double>> point_Cloud_input, double radius_input) {

		point_Cloud = point_Cloud_input;		
		radius = radius_input;

	}

	vector<vector<double>> simplification_Method_CGAL_Grid(double cell_size) {

		std::cout << "simplification_Method_CGAL_Grid run." << endl;
		long start = clock();
		vector<Point3> points;
		for (int i = 0; i < point_Cloud.size(); i++) {
			vector<double> point_Cloud_i = point_Cloud[i];
			Point3 pi(point_Cloud_i[0], point_Cloud_i[1], point_Cloud_i[2]);
			points.push_back(pi);
		}
		//double cell_size = 0.001;
		points.erase(CGAL::grid_simplify_point_set(points, cell_size),
			points.end());
		// Optional: after erase(), use Scott Meyer's "swap trick" to trim excess capacity
		std::vector<Point3>(points).swap(points);

		vector<vector<double>> pointCloudSim;
		for (int i = 0; i < points.size(); i++) {
			Point3 p_i = points[i];
			vector<double> p_ii;
			p_ii.push_back(p_i[0]);
			p_ii.push_back(p_i[1]);
			p_ii.push_back(p_i[2]);
			pointCloudSim.push_back(p_ii);
		}

		long end = clock();
		std::cout << "time consume is:" << end - start << "ms" << endl;
		return pointCloudSim;
	}

	vector<vector<double>> simplification_Method_CGAL_Hierarchy(int paSize, double surVar) {

		std::cout << "simplification_Method_CGAL_Hierarchy run." << endl;
		long start = clock();
		vector<Point3> points;
		for (int i = 0; i < point_Cloud.size(); i++) {
			vector<double> point_Cloud_i = point_Cloud[i];
			Point3 pi(point_Cloud_i[0], point_Cloud_i[1], point_Cloud_i[2]);
			points.push_back(pi);
		}

		//paSize = 100
		//surVar = 0.01

		points.erase(CGAL::hierarchy_simplify_point_set(points,
			CGAL::parameters::size(paSize). // Max cluster size
			maximum_variation(surVar)), // Max surface variation
			points.end());
		// Optional: after erase(), use Scott Meyer's "swap trick" to trim excess capacity
		std::vector<Point3>(points).swap(points);

		vector<vector<double>> pointCloudSim;
		for (int i = 0; i < points.size(); i++) {
			Point3 p_i = points[i];
			vector<double> p_ii;
			p_ii.push_back(p_i[0]);
			p_ii.push_back(p_i[1]);
			p_ii.push_back(p_i[2]);
			pointCloudSim.push_back(p_ii);
		}
		long end = clock();
		std::cout << "time consume is:" << end - start << "ms" << endl;
		return pointCloudSim;
	}

	vector<vector<double>> simplification_Method_CGAL_WLOP(int pointsSize) {

		double retain_percentage;
		//double wan = (double)1000000 / (double)point_Cloud.size();
		retain_percentage = (double)pointsSize / (double)point_Cloud.size() * 100;

		std::cout << "simplification_Method_CGAL_Wlop run." << endl;
		long start = clock();
		vector<Point3> points;
		for (int i = 0; i < point_Cloud.size(); i++) {
			vector<double> point_Cloud_i = point_Cloud[i];
			Point3 pi(point_Cloud_i[0], point_Cloud_i[1], point_Cloud_i[2]);
			points.push_back(pi);
		}
		std::vector<Point3> output;
		//parameters
		//const double retain_percentage = 2;   // percentage of points to retain.
		const double neighbor_radius = radius;   // neighbors size.
		CGAL::wlop_simplify_and_regularize_point_set<Concurrency_tag>
			(points, std::back_inserter(output),
				CGAL::parameters::select_percentage(retain_percentage).
				neighbor_radius(neighbor_radius));

		vector<vector<double>> pointCloudSim;
		for (int i = 0; i < output.size(); i++) {
			Point3 p_i = output[i];
			vector<double> p_ii;
			p_ii.push_back(p_i[0]);
			p_ii.push_back(p_i[1]);
			p_ii.push_back(p_i[2]);
			pointCloudSim.push_back(p_ii);
		}

		long end = clock();
		std::cout << "time consume is:" << end - start << "ms" << endl;
		return pointCloudSim;
	}

};

