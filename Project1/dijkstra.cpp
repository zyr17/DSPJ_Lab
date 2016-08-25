#include "dijkstra.h"

namespace dijkstra{
	void dijkstra::clear(){
		tot = 0;
		tail.clear();
		next.clear();
		num.clear();
		dis.clear();
		val.clear();
		fa.clear();
		heap.clear();
	}
	inline void dijkstra::AddEdge(int x, int y, double z){
		num.push_back(y);
		val.push_back(z);
		next.push_back(-1);
		tail[x] = next[tail[x]] = tot++;
	}

	void dijkstra::do_mul_dijkstra(int st, std::vector<int> &ed, std::vector<double> &dist){
		std::vector<int> tmp, tmp2;
		do_mul_dijkstra(st, ed, dist, tmp, tmp2);
	}
	void dijkstra::do_mul_dijkstra(int st, std::vector<int> &ed, std::vector<double> &dist, std::vector<int> &r_start, std::vector<int> &route){
		auto cmp = [](std::pair<double, int> a, std::pair<double, int> b) { return a > b; };
		std::vector<int> res;
		r_start.clear();
		route.clear();
		dist.clear();
		heap.clear();
		heap.push_back(std::make_pair(0, st));
		dis.clear();
		dis.resize(n, line_max);
		fa.clear();
		fa.resize(n, -1);
		dis[st] = 0;
		for (; heap.size();){
			if (ed.size() == 1 && heap[0].second == ed[0]) break;
			if (heap[0].first != dis[heap[0].second]){
				std::pop_heap(heap.begin(), heap.end(), cmp);
				heap.pop_back();
				continue;
			}
			int now = heap[0].second;
			std::pop_heap(heap.begin(), heap.end(), cmp);
			heap.pop_back();
			for (int j = next[now]; ~j; j = next[j])
				if (dis[num[j]] > dis[now] + val[j]){
					dis[num[j]] = dis[now] + val[j];
					fa[num[j]] = now;
					heap.push_back(std::make_pair(dis[num[j]], num[j]));
					std::push_heap(heap.begin(), heap.end(), cmp);
				}
		}

		for (int i = 0; i < ed.size(); i++){
			int end = ed[i];
			res.clear();
			for (int j = end; ~j; j = fa[j])
				res.push_back(j);
			for (int j = 0; j < res.size() / 2; j++){
				int tmp = res[j];
				res[j] = res[res.size() - 1 - j];
				res[res.size() - 1 - j] = tmp;
			}
			if (res.size() && res[0] != st) res.clear();
			r_start.push_back(route.size());
			for (int j = 0; j < res.size(); j++)
				route.push_back(res[j]);
			dist.push_back(dis[end]);
		}
		r_start.push_back(route.size());
	}
	void dijkstra::do_mul_dijkstra(std::vector<int> &st, std::vector<double> &dist){
#ifdef DEBUG
		int tt_tt = st.size();
#endif
		auto cmp = [](std::pair<double, int> a, std::pair<double, int> b) { return a > b; };
		dist.clear();
		heap.clear();
		for (auto i : st)
			heap.push_back(std::make_pair(0.0, i));
		std::make_heap(heap.begin(), heap.end(), cmp);
		dis.clear();
		dis.resize(n, line_max);
		for (auto i : st)
			dis[i] = 0;
		for (; heap.size();){
			if (heap[0].first != dis[heap[0].second]){
				std::pop_heap(heap.begin(), heap.end(), cmp);
				heap.pop_back();
				continue;
			}
			int now = heap[0].second;
			std::pop_heap(heap.begin(), heap.end(), cmp);
			heap.pop_back();
			for (int j = next[now]; ~j; j = next[j])
				if (dis[num[j]] > dis[now] + val[j]){
#ifdef DEBUG
					tt_tt ++ ;
#endif
					dis[num[j]] = dis[now] + val[j];
					heap.push_back(std::make_pair(dis[num[j]], num[j]));
					std::push_heap(heap.begin(), heap.end(), cmp);
				}
		}
		dist = dis;
#ifdef DEBUG
		printf("tt_tt in dijkstra: %d\n", tt_tt);
#endif
	}

	dijkstra::dijkstra(){

	}
	void dijkstra::init(char* file_name){
		/* TODO: Input from File */
	}
	void dijkstra::init(int tot_point, std::vector<sp_common::line> &lines){
		clear();
		n = tot_point;
		m = lines.size();
		tail.resize(n);
		next.resize(n, -1);
		num.resize(n);
		val.resize(n);
		fa.resize(n, -1);
		dis.resize(n);
		last_calc.resize(n);
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
	double dijkstra::one_end(int st, int ed){
		std::vector<int> eds;
		eds.push_back(ed);
		std::vector<double> ans;
		several_end(st, eds, ans);
		return ans[0];
	}
	double dijkstra::one_end(int st, int ed, std::vector<int> &res){
		std::vector<int> eds, tmp;
		eds.push_back(ed);
		std::vector<double> ans;
		several_end(st, eds, ans, tmp, res);
		return ans[0];
	}
	void dijkstra::several_end(int st, std::vector<int> &ed, std::vector<double> &dist){
		do_mul_dijkstra(st, ed, dist);
	}
	void dijkstra::several_end(int st, std::vector<int> &ed, std::vector<double> &dist, std::vector<int> &r_start, std::vector<int> &route){
		do_mul_dijkstra(st, ed, dist, r_start, route);
	}
	void dijkstra::several_start(std::vector<int> &start, std::vector<double> &dist){
		do_mul_dijkstra(start, dist);
	}
	void dijkstra::add_estimate_data(std::vector<geometry::Point> &vec){
		if (vec.size() < n)	return;
		point_pos = vec;
		for (int i = 0; i < n; i++)
			point_pos[i]._re = i;
		estimate_data_added = 1;
	}
	void dijkstra::taxi_data_fitting(int tot, std::vector<Taxi_Data> &data, sp_common::mid_point start, double max_r, double p_rate, std::vector<int> &r_start, std::vector<geometry::Point> & res, std::vector<taxi_detail> & Taxi_Detail){
		res.clear();
		double Clock();
		double starttime = Clock();
		if (tot > data.size()) return;
		geometry::Point tmp = point_pos[start.y] - point_pos[start.x];
		r_start.push_back(0);
		res.push_back(tmp / tmp.len() * start.z + point_pos[start.x]);
		Taxi_Detail.clear();
		Taxi_Detail.push_back(taxi_detail(-1, start, data[0].time, 0, data[0].empty));
		int nowres = 0;
		sp_common::mid_point last = start, mid_now;
		for (int i = 1; i < tot; i++){
			//printf("%d: \n", i);
			double now_closest = 1e100;
			geometry::Point now_point = geometry::Point(data[i].x, data[i].y);
			calc_times++;
			last_calc[last.x] = last_calc[last.y] = calc_times;
			dis[last.x] = last.z;
			dis[last.y] = last.z2;
			fa[last.x] = fa[last.y] = -1;
			heap.clear();
			heap.push_back(std::make_pair(-last.z, last.x));
			heap.push_back(std::make_pair(-last.z2, last.y));
			double max_dis = max_r * (now_point - geometry::Point(data[i - 1].x, data[i - 1].y)).len() + (now_point - point_pos[last.x]).len() + (geometry::Point(data[i - 1].x, data[i - 1].y) - point_pos[last.x]).len();
			for (; heap.size();){
				if (Clock() - starttime > 15){
					printf("too long!halt.\n");
					r_start.clear();
					res.clear();
					Taxi_Detail.clear();
					return;
				}
				int nowp = heap[0].second;
				if (dis[nowp] != -heap[0].first || dis[nowp] > max_dis){
					std::pop_heap(heap.begin(), heap.end());
					heap.pop_back();
					continue;
				}
				std::pop_heap(heap.begin(), heap.end());
				heap.pop_back();
				for (int j = next[nowp]; ~j; j = next[j]){
					if (last_calc[num[j]] != calc_times || dis[num[j]] > dis[nowp] + val[j]){
						dis[num[j]] = dis[nowp] + val[j];
						heap.push_back(std::make_pair(-dis[num[j]], num[j]));
						std::push_heap(heap.begin(), heap.end());
						last_calc[num[j]] = calc_times;
						fa[num[j]] = nowp;
					}
					double extradis = geometry::dis_Seg_P(point_pos[nowp], point_pos[num[j]], now_point);
					geometry::Point proj = geometry::proj(point_pos[nowp], point_pos[num[j]], now_point);
					double totdis = dis[nowp] + p_rate * extradis;
					if (geometry::is_point_onseg(point_pos[nowp], point_pos[num[j]], proj)){
						double ttt = (proj - point_pos[nowp]).len();
						totdis += ttt;
						if (totdis < now_closest){
							now_closest = totdis;
							mid_now = sp_common::mid_point(nowp, num[j], ttt, val[j] - ttt);
						}
					}
					else if (((point_pos[nowp].x < point_pos[num[j]].x) == (point_pos[nowp].x < proj.x)) && ((point_pos[nowp].y < point_pos[num[j]].y) == (point_pos[nowp].y < proj.y))){
						totdis += val[j];
						if (totdis < now_closest){
							now_closest = totdis;
							mid_now = sp_common::mid_point(nowp, num[j], val[j], 0);
						}
					}
					else{
						if (totdis < now_closest){
							now_closest = totdis;
							mid_now = sp_common::mid_point(nowp, num[j], 0, val[j]);
						}
					}
				}
			}
			r_start.push_back(res.size());
			geometry::Point tproj = geometry::proj(point_pos[last.x], point_pos[last.y], now_point);
			if (geometry::is_point_onseg(point_pos[last.x], point_pos[last.y], tproj)){
				double tnow_dis = std::abs((tproj - point_pos[last.x]).len() - last.z) + (now_point - tproj).len() * p_rate;
				if (tnow_dis < now_closest){
					res.push_back(tproj);
					Taxi_Detail.push_back(taxi_detail(-1, mid_now, data[i].time, std::abs((tproj - point_pos[last.x]).len() - last.z), data[i].empty));
					last.z = (tproj - point_pos[last.x]).len();
					last.z2 = (point_pos[last.x] - point_pos[last.y]).len() - last.z;
					continue;
				}
			}
			if (now_closest > 1e10){
				NNNOOO:;
				printf("cannot find ! what happened..");
				continue;
			}
			std::vector<int> tvec;
			int end = mid_now.x;
			for (int j = end; ~j && last_calc[j] == calc_times; j = fa[j])
				tvec.push_back(j);
			for (int j = 0; j < tvec.size() / 2; j++){
				int tmp = tvec[j];
				tvec[j] = tvec[tvec.size() - 1 - j];
				tvec[tvec.size() - 1 - j] = tmp;
			}
			if (tvec[0] != last.x && tvec[0] != last.y){
				tvec.clear();
				goto NNNOOO;
			}
			for (int j = 0; j < tvec.size(); j++)
				res.push_back(point_pos[tvec[j]]);
			double totlength = (tvec[0] == last.x ? last.z : last.z2) + mid_now.z;
			for (int j = 1; j < tvec.size(); j++)
				totlength += (point_pos[tvec[j]] - point_pos[tvec[j - 1]]).len();
			totlength = (data[i].time - data[i - 1].time) / totlength;
			double nowtime = (tvec[0] == last.x ? last.z : last.z2) * totlength + data[i - 1].time;
			double nowlength = tvec[0] == last.x ? last.z : last.z2;
			for (int j = 0; j < tvec.size(); j++){
				Taxi_Detail.push_back(taxi_detail(tvec[j], last, -1, nowlength, data[i - 1].empty));
				if (j < tvec.size() - 1) nowtime += (point_pos[tvec[j]] - point_pos[tvec[j + 1]]).len() * totlength;
				if (j < tvec.size() - 1) nowlength = (point_pos[tvec[j]] - point_pos[tvec[j + 1]]).len();
			}
			geometry::Point tmp = point_pos[mid_now.y] - point_pos[mid_now.x];
			res.push_back(tmp / tmp.len() * mid_now.z + point_pos[mid_now.x]);
			Taxi_Detail.push_back(taxi_detail(-1, mid_now, data[i].time, mid_now.z, data[i].empty));
			last = mid_now;
		}
		r_start.push_back(res.size());
	}
}
