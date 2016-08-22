#include "astar.h"

namespace astar{
	void astar::clear(){
		tot = estimate_data_added = 0;
		tail.clear();
		next.clear();
		num.clear();
		dis.clear();
		val.clear();
		fa.clear();
		last_calc.clear();
		e_func.clear();
		heap.resize(10000);
		calc_times = 0;
	}
	inline double astar::estimate_func(int x, int y){
		//return est_input_func(x, y);
		if (!estimate_data_added) return 0;
		if (last_calc[x] == calc_times) return e_func[x];
		last_calc[x] = calc_times;
		return e_func[x] = (point_pos[x] - point_pos[y]).len();
	}
	inline void astar::AddEdge(int x, int y, double z){
		num.push_back(y);
		val.push_back(z);
		next.push_back(-1);
		tail[x] = next[tail[x]] = tot++;
	}
	void astar::do_astar(int st, int ed, double &dist){
		std::vector<int> tmp;
		do_astar(st, ed, dist, tmp);
	}
	void astar::do_astar(int st, int end, double &dist, std::vector<int> &route){
		//printf("A******** %d %d\n", st, end);
		calc_times++;
		auto cmp = [](std::pair<double, int> x, std::pair<double, int> y) {return x > y; };
		//fa.clear();
		//fa.resize(n, -1);
		//dis.clear();
		//dis.resize(n, line_max);
		dis[st] = 0;
		fa[st] = -1;
		route.clear();
		//heap.clear();
		//heap.push_back(std::make_pair(calc_f(st, end), st));
		heap[0] = std::make_pair(estimate_func(st, end), st);
		int size = 1;
		for (; size;){
			int now = heap[0].second; //printf("[%d %d]", size, now);
			if (now == end) break;
			if (heap[0].first != dis[now] + estimate_func(now, end)){
				std::pop_heap(heap.begin(), heap.begin() + size -- , cmp);
				continue;
			}
			std::pop_heap(heap.begin(), heap.begin() + size--, cmp);
			for (int j = next[now]; ~j; j = next[j])
			if (last_calc[num[j]] != calc_times || dis[num[j]] > dis[now] + val[j]){
				dis[num[j]] = dis[now] + val[j];
				fa[num[j]] = now;
				heap[size++] = std::make_pair(dis[num[j]] + estimate_func(num[j], end), num[j]);
				std::push_heap(heap.begin(), heap.begin() + size, cmp);
			}
		}

		std::vector<int> &res = route;
		res.clear();
		int res_tot = 0;
		for (int j = end; ~j && last_calc[j] == calc_times; j = fa[j])
			res_tot++;
		res.resize(res_tot);
		for (int j = end; ~j && last_calc[j] == calc_times; j = fa[j])
			res[--res_tot] = j;
		if (res.size() && res[0] != st) res.clear();
		if (last_calc[end] != calc_times) dis[end] = line_max;
		dist = dis[end];
	}

	astar::astar(){

	}
	void astar::init(char* file_name){
		/* TODO: Input from File */
	}
	void astar::init(int tot_point, std::vector<sp_common::line> &lines){
		clear();
		n = tot_point;
		m = lines.size();
		tail.resize(n);
		next.resize(n, -1);
		num.resize(n);
		val.resize(n);
		dis.resize(n);
		fa.resize(n);
		last_calc.resize(n);
		e_func.resize(n);
		tot = n;
		for (int i = 1; i < n; i++)
			tail[i] = i;
		for (int i = 0; i < m; i++){
			AddEdge(lines[i].st, lines[i].ed, lines[i].dist);
#ifdef DOUBLE_WAY
			AddEdge(lines[i].ed, lines[i].st, lines[i].dist);
#endif
		}
	}
	double astar::one_end(int st, int ed){
		std::vector<int> ans;
		return one_end(st, ed, ans);
	}
	double astar::one_end(int st, int ed, std::vector<int> &res){
		double re;
		do_astar(st, ed, re, res);
		return re;
	}
	void astar::add_estimate_data(std::vector<geometry::Point> &vec){
		if (vec.size() < n)	return;
		point_pos = vec;
		estimate_data_added = 1;
	}
}
