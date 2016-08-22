#pragma once
#include <vector>
#include "shortest_path_header.h"
#include "geometry.h"
namespace d_dijkstra{
	class d_dijkstra{
	private:
		std::vector<int> tail, next, num, fa, tailb, nextb, numb, fab, last_calc;
		std::vector<double> dis, val, disb, valb;
		std::vector<bool> visb;
		std::vector<std::pair<double, int>> heap, heapb;
		std::vector<geometry::Point> point_pos;
		//std::vector<sp_common::mid_point> nearset_line_mid;
		int estimate_data_added, calc_times, tot, totb, n, m;
		void clear();
		inline void AddEdge(int x, int y, double z);
		inline void AddEdge_back(int x, int y, double z);
		void do_d_dijkstra(int st, int ed, double &dist);
		void do_d_dijkstra(int st, int ed, double &dist, std::vector<int> &route);
	public:
		d_dijkstra();
		void init(char* file);
		void init(int tot_point, std::vector<sp_common::line> &lines);
		double one_end(int start, int end);
		double one_end(int start, int end, std::vector<int> &route);
		void add_estimate_data(std::vector<geometry::Point> &position);
	};
}