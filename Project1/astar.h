#pragma once
#include <vector>
#include <algorithm>
#include "shortest_path_header.h"
#include "geometry.h"
namespace astar{


	class astar{
	private:
		std::vector<int> tail, next, num, fa, last_calc;
		std::vector<double> dis, val, e_func;
		std::vector<std::pair<double, int>> heap;
		std::vector<geometry::Point> point_pos;
		int tot, n, m, estimate_data_added, calc_times;
		void clear();
		inline double estimate_func(int x, int y);
		inline void AddEdge(int x, int y, double z);
		void do_astar(int st, int ed, double &dist);
		void do_astar(int st, int ed, double &dist, std::vector<int> &route);
	public:
		double (*est_input_func)(int, int);
		astar();
		void init(char* file);
		void init(int tot_point, std::vector<sp_common::line> &lines);
		double one_end(int start, int end);
		double one_end(int start, int end, std::vector<int> &route);
		void add_estimate_data(std::vector<geometry::Point> &position);
	};
}