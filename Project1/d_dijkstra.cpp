#include "d_dijkstra.h"
namespace d_dijkstra{
	void d_dijkstra::clear(){
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
	inline void d_dijkstra::AddEdge(int x, int y, double z){
		num.push_back(y);
		val.push_back(z);
		next.push_back(-1);
		tail[x] = next[tail[x]] = tot++;
		d_dijkstra::AddEdge_back(y, x, z);
	}
	inline void d_dijkstra::AddEdge_back(int x, int y, double z){
		numb.push_back(y);
		valb.push_back(z);
		nextb.push_back(-1);//printf("%d %d %f %d %d %d %d\n", x, y, z, tailb.size(), nextb.size(), totb, tailb[x]);
		tailb[x] = nextb[tailb[x]] = totb++;
	}

	void d_dijkstra::do_d_dijkstra(int st, int ed, double &dist){
		std::vector<int> tmp;
		do_d_dijkstra(st, ed, dist, tmp);
	}
	void d_dijkstra::do_d_dijkstra(int st, int ed, double &dist, std::vector<int> &route){
		auto cmp = [](std::pair<double, int> a, std::pair<double, int> b) { return a > b; };
		std::vector<int> res;
		double nowmin = line_max;
		int nowid = 0;
		route.clear();
		//dist = 0;
		heap.clear();
		heapb.clear();
		heap.push_back(std::make_pair(0, st));
		heapb.push_back(std::make_pair(0, ed));
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
		for (; heap.size() && heapb.size();){//printf("|%d %d|\n", heap.size(), heapb.size());
			if (heap[0].first != dis[heap[0].second]){
				std::pop_heap(heap.begin(), heap.end(), cmp);
				heap.pop_back();
				continue;
			}
			if (heapb[0].first != disb[heapb[0].second]){
				std::pop_heap(heapb.begin(), heapb.end(), cmp);
				heapb.pop_back();
				continue;
			}
			if (heap[0].first + heapb[0].first > nowmin) break;
			if (heap[0].first < heapb[0].first){
				int now = heap[0].second;
				std::pop_heap(heap.begin(), heap.end(), cmp);
				if (dis[now] + disb[now] < nowmin){
					nowmin = dis[now] + disb[now];
					nowid = now;
				}
				heap.pop_back();
				for (int j = next[now]; ~j; j = next[j])
					if (dis[num[j]] > dis[now] + val[j]){
						dis[num[j]] = dis[now] + val[j];
						fa[num[j]] = now;
						heap.push_back(std::make_pair(dis[num[j]], num[j]));
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
						heapb.push_back(std::make_pair(disb[numb[j]], numb[j]));
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

	d_dijkstra::d_dijkstra(){

	}
	void d_dijkstra::init(char* file_name){
		/* TODO: Input from File */
	}
	void d_dijkstra::init(int tot_point, std::vector<sp_common::line> &lines){
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
	double d_dijkstra::one_end(int st, int ed){
		std::vector<int> res;
		return one_end(st, ed, res);
	}
	double d_dijkstra::one_end(int st, int ed, std::vector<int> &res){
		double ans;
		do_d_dijkstra(st, ed, ans, res);
		return ans;
	}
	void d_dijkstra::add_estimate_data(std::vector<geometry::Point> &vec){
		if (vec.size() < n)	return;
		point_pos = vec;
		for (int i = 0; i < n; i++)
			point_pos[i]._re = i;
		estimate_data_added = 1;
	}
}