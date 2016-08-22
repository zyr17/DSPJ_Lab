#ifndef __SPFA_H__
#define __SPFA_H__
#include <vector>
#include "shortest_path_header.h"
namespace spfa{
	class spfa{
	private:
		std::vector<int> l, tail, next, num, fa;
		std::vector<double> dis, val;
		std::vector<bool> isin;
		int tot, n, m;
		void clear();
		inline void AddEdge(int x, int y, double z);
		void do_mul_spfa(int st, std::vector<int> &ed, std::vector<double> &dist);
		void do_mul_spfa(int st, std::vector<int> &ed, std::vector<double> &dist, std::vector<int> &route_start, std::vector<int> &route);
	public:
		spfa();
		void init(char* file);
		void init(int tot_point, std::vector<sp_common::line> &lines);
		double one_end(int start, int end);
		double one_end(int start, int end, std::vector<int> &route);
		void several_end(int start, std::vector<int> &end, std::vector<double> &dist);
		void several_end(int start, std::vector<int> &end, std::vector<double> &dist, std::vector<int> &route_start, std::vector<int> &route);
	};
}
#endif