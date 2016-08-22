#pragma once
#include <vector>
#include <algorithm>
#include "shortest_path_header.h"
#include "geometry.h"
#include "taxi_data.h"
#include "common.h"
namespace dijkstra{
	class dijkstra{
	private:
		std::vector<int> tail, next, num, fa, last_calc;
		std::vector<double> dis, val;
		std::vector<std::pair<double, int>> heap;
		std::vector<geometry::Point> point_pos;
		std::vector<sp_common::mid_point> nearset_line_mid;
		int estimate_data_added, calc_times, tot, n, m;
		void clear();
		inline void AddEdge(int x, int y, double z);
		void do_mul_dijkstra(int st, std::vector<int> &ed, std::vector<double> &dist);
		void do_mul_dijkstra(int st, std::vector<int> &ed, std::vector<double> &dist, std::vector<int> &route_start, std::vector<int> &route);
		void do_mul_dijkstra(std::vector<int> &st, std::vector<double> &dist);
	public:
		dijkstra();
		void init(char* file);
		void init(int tot_point, std::vector<sp_common::line> &lines);
		double one_end(int start, int end);
		double one_end(int start, int end, std::vector<int> &route);
		void several_end(int start, std::vector<int> &end, std::vector<double> &dist);
		void several_end(int start, std::vector<int> &end, std::vector<double> &dist, std::vector<int> &route_start, std::vector<int> &route);
		void several_start(std::vector<int> &start, std::vector<double> &dist);
		void add_estimate_data(std::vector<geometry::Point> &position);
		void taxi_data_fitting(int tot, std::vector<Taxi_Data> &data, sp_common::mid_point start, double max_radius, double punish_rate, std::vector<int> &route_start, std::vector<geometry::Point> & ans_route, std::vector<taxi_detail> & Taxi_Detail);
	};
}