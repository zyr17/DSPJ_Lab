#include "d_astar.h"
namespace d_astar{
	void d_astar::clear(){
		tot = totb = 0;
		tail.clear();
		next.clear();
		num.clear();
		val.clear();
		tailb.clear();
		nextb.clear();
		numb.clear();
		valb.clear();
		dis.clear();
		disb.clear();
		fa.clear();
		fab.clear();
		heap.clear();
		heapb.clear();
		visb.clear();
	}
	inline void d_astar::AddEdge(int x, int y, double z){
		num.push_back(y);
		val.push_back(z);
		next.push_back(-1);
		tail[x] = next[tail[x]] = tot++;//printf("a%d %d %f %d %d %d %d\n", x, y, z, tail.size(), next.size(), tot, tail[x]);
		d_astar::AddEdge_back(y, x, z);
	}
	inline void d_astar::AddEdge_back(int x, int y, double z){
		numb.push_back(y);
		valb.push_back(z);
		nextb.push_back(-1);
		tailb[x] = nextb[tailb[x]] = totb++;//printf("b%d %d %f %d %d %d %d\n", x, y, z, tailb.size(), nextb.size(), totb, tailb[x]);
	}

	void d_astar::do_d_astar(int st, int ed, double &dist){
		std::vector<int> tmp;
		do_d_astar(st, ed, dist, tmp);
	}
	void d_astar::do_d_astar(int st, int ed, double &dist, std::vector<int> &route){
		auto cmp = [](std::pair<double, int> a, std::pair<double, int> b) { return a > b; };
		std::vector<int> res;
		double nowmin = line_max;
		int nowid = 0;
		route.clear();
		//dist = 0;
		heap.clear();
		heapb.clear();
		heap.push_back(std::make_pair(estimate_func(st, ed), st));
		heapb.push_back(std::make_pair(estimate_funcb(ed, st), ed));
		dis.clear();
		dis.resize(n, line_max);
		disb.clear();
		disb.resize(n, line_max);
		fa.clear();
		fa.resize(n, -1);
		fab.clear();
		fab.resize(n, -1);
		visb.clear();
		visb.resize(n, 0);
		visb[ed] = 1;
		dis[st] = 0;
		disb[ed] = 0;
		bool step_2 = 0;
		for (; heap.size() && heapb.size();){//printf("|%d %d %.0f %d %.0f %d|\n", heap.size(), heapb.size(), heap[0].first, heap[0].second, heapb[0].first, heapb[0].second);
			if (heap[0].first != dis[heap[0].second] + estimate_func(heap[0].second, ed)){
				std::pop_heap(heap.begin(), heap.end(), cmp);
				heap.pop_back();
				continue;
			}
			if (heapb[0].first != disb[heapb[0].second] + estimate_funcb(heapb[0].second, st)){
				std::pop_heap(heapb.begin(), heapb.end(), cmp);
				heapb.pop_back();
				continue;
			}
			if (heap[0].first > nowmin && heapb[0].first > nowmin) break;
			if (heap[0].first < heapb[0].first){
				int now = heap[0].second;
				std::pop_heap(heap.begin(), heap.end(), cmp);
				if (dis[now] + disb[now] < nowmin){
					nowmin = dis[now] + disb[now];
					nowid = now;
				}
				heap.pop_back();
				for (int j = next[now]; ~j; j = next[j])
					if (dis[num[j]] > dis[now] + val[j]){//printf("%d %d\n", now, num[j]);
						dis[num[j]] = dis[now] + val[j];
						fa[num[j]] = now;
						heap.push_back(std::make_pair(dis[num[j]] + estimate_func(num[j], ed), num[j]));
						std::push_heap(heap.begin(), heap.end(), cmp);
					}
			}
			else{
				int now = heapb[0].second;
				visb[now] = 1;
				std::pop_heap(heapb.begin(), heapb.end(), cmp);
				if (dis[now] + disb[now] < nowmin){
					nowmin = dis[now] + disb[now];
					nowid = now;
				}
				heapb.pop_back();
				for (int j = nextb[now]; ~j; j = nextb[j])
					if (disb[numb[j]] > disb[now] + valb[j]){
						disb[numb[j]] = disb[now] + valb[j];
						fab[numb[j]] = now;
						heapb.push_back(std::make_pair(disb[numb[j]] + estimate_funcb(numb[j], st), numb[j]));
						std::push_heap(heapb.begin(), heapb.end(), cmp);
					}
			}
		}
		int end = nowid;
		res.clear();
		for (int j = end; ~j; j = fa[j])
			res.push_back(j);
		for (int j = 0; j < res.size() / 2; j++){
			int tmp = res[j];
			res[j] = res[res.size() - 1 - j];
			res[res.size() - 1 - j] = tmp;
		}
		for (int j = fab[nowid]; ~j; j = fab[j])
			res.push_back(j);
		if (res.size() && res[0] != st) res.clear();
		route = res;
		dist = nowmin;
		if (dist > line_max / 1024) dist = -1;
	}

	inline double d_astar::estimate_func(int x, int y){
		return 0;
	}

	inline double d_astar::estimate_funcb(int x, int y){
		return 0;
	}

	d_astar::d_astar(){

	}
	void d_astar::init(char* file_name){
		/* TODO: Input from File */
	}
	void d_astar::init(int tot_point, std::vector<sp_common::line> &lines){
		clear();
		n = tot_point;
		m = lines.size();
		visb.resize(n);
		tail.resize(n);
		next.resize(n, -1);
		num.resize(n);
		val.resize(n);
		fa.resize(n, -1);
		dis.resize(n);
		tailb.resize(n);
		nextb.resize(n, -1);
		numb.resize(n);
		valb.resize(n);
		fab.resize(n, -1);
		disb.resize(n);
		last_calc.resize(n);
		tot = totb = n;
		for (int i = 1; i < n; i++)
			tail[i] = tailb[i] = i;
		for (int i = 0; i < m; i++){
			AddEdge(lines[i].st, lines[i].ed, lines[i].dist);
#ifdef DOUBLE_WAY
			AddEdge(lines[i].ed, lines[i].st, lines[i].dist);
#endif
		}
	}
	double d_astar::one_end(int st, int ed){
		std::vector<int> res;
		return one_end(st, ed, res);
	}
	double d_astar::one_end(int st, int ed, std::vector<int> &res){
		double ans;
		do_d_astar(st, ed, ans, res);
		return ans;
	}
	void d_astar::add_estimate_data(std::vector<geometry::Point> &vec){
		if (vec.size() < n)	return;
		point_pos = vec;
		for (int i = 0; i < n; i++)
			point_pos[i]._re = i;
		estimate_data_added = 1;
	}
}