#include "TDFP.h"
inline double TDFP::estimate_func(int now, int dest, double last){
	if (last < 0) last = 0;
	return has_group[now] && has_group[now] == has_group[dest] ? 0 : 
		   has_group[now] && has_group[dest] ? LTT_p2e[now] + LTT_e2e[has_group[now]][has_group[dest]] + LTT_e2p[dest] : 
		   has_group[now] ? no_group2e[has_group[now]][no_group_id[dest]] :
		   has_group[dest] ? no_group2e[has_group[dest]][no_group_id[now]] : last;
}
inline double TDFP::estimate_funcb(int now, int dest, double last){
	return estimate_func(now, dest, last);
}
void TDFP::clear(){
	/* TODO */
	s_tot = 0;
	LBT.resize(m);
}
void TDFP::init_base(int in, int im, std::vector<Two> &ipoint, std::vector<Three> &input){
	n = in;
	m = im;
	clear();
	point = ipoint;
	point_level.resize(ipoint.size());
	edges = input;
	for (int i = 0; i < input.size(); i ++ ){
		list[input[i].x].push_back(Four(input[i].y, i, 0, atan2(point[input[i].y].y - point[input[i].x].y, point[input[i].y].x - point[input[i].x].x)));
		listdone[input[i].x].push_back(0);
		list[input[i].y].push_back(Four(input[i].x, i, 1, atan2(point[input[i].x].y - point[input[i].y].y, point[input[i].x].x - point[input[i].y].x)));
		listdone[input[i].y].push_back(0);
		if (point_level[input[i].x] < input[i].z)
			point_level[input[i].x] = input[i].z;
		//maxline[input[i].x] = std::max(maxline[input[i].x], input[i].z);
		//maxline[input[i].y] = std::max(maxline[input[i].y], input[i].z);
	}
	for (int i = 0; i < n; i ++ )
		std::sort(list[i].begin(), list[i].end(), [=] (Four x, Four y) { return x.d < y.d; });
}
void TDFP::init_time(std::vector<double> &input){
	int now = 0, see = input.size();
	for (int i = 0; i < m; i++){
		//printf("-%d %d %d %d %d-\n", m, i, now, input.size(), input[now]);
		LBT[i] = 1e100;
		int ttttt;
		//printf("%f %d\n", input[now], ttttt = input[now]);
		for (int tt = input[now ++ ]; tt -- ; ){
			//printf("%d %d\n", i, edge_time[i].size());
			//if (edge_time[i].size()) printf("%lf %lf\n", edge_time[i][0].first, edge_time[i][0].second);
			edge_time[i].push_back(std::make_pair(input[now], input[now + 1]));
			LBT[i] = std::min(LBT[i], (double)input[now + 1]);
			now += 2;
		}
	}
}
void TDFP::divide_area(int k){
	//bool *done = new bool[m][2];
	FILE *tout;
	tout = fopen("output.txt", "w");
	memset(done, 0, sizeof done);
	const double pi = 3.14159265358979323846;
	for (int i = 0; i < m; i ++ )
		for (int j = 0; j < 2; j ++ )
			if (!done[i][j] && edges[i].z >= k){
				done[i][j] = 1;
				std::vector<int> line, line2;
				line.push_back(edges[i].x);
				line.push_back(edges[i].y);
				if (j) std::swap(line[0], line[1]);
				int line0 = line[0];
				//double totang = 0;
				for (int i = 1; line[i] != line[0]; i++){
					//printf("%d %d\n", i, line[i]);
					int see = line[i];
					double aa = atan2(point[line[i - 1]].y - point[line[i]].y, point[line[i - 1]].x - point[line[i]].x);
					int now = 0, t = line[i];
					for (; now < list[t].size(); now ++ )
						if (list[t][now].to == line[i - 1]) break;//now: now line number
					//printf("list:	%d %d %d %lf\n%d	%d %d %d %f\n", list[t][now].to, list[t][now].num, list[t][now].rev, list[t][now].d, i, t, line[0], now, aa);
					//for (int i = 0; i < list[t].size(); i++)
					//	System::Console::WriteLine("alllist: {0} {1} {2} {3} {4} {5} {6} {7} {8}", list[t][i].to, list[t][i].num, list[t][i].rev, list[t][i].d, done[list[t][i].num][list[t][i].rev], point[t].x, point[t].y, point[list[t][i].to].x, point[list[t][i].to].y);
					if (now == list[t].size()) for (;;);
					for (int TT = 0; TT < list[t].size(); TT ++ ){
						if (++now >= list[t].size()) now = 0;
						if (edges[list[t][now].num].z >= k){
							/*if (done[list[t][now].num][list[t][now].rev] == 1){
								printf("boom");
								for (;;);
							}*/
							if (done[list[t][now].num][list[t][now].rev]) continue;
							done[list[t][now].num][list[t][now].rev] = 1;
							line.push_back(list[t][now].to);
							/*double delta = list[t][now].d - aa;
							if (delta < 0) delta += 2 * pi;
							totang += delta;*/
							break;
						}
						else line2.push_back(list[t][now].to);
					}
				}
				std::vector<int> tvec;
				for (int i = 0; i < line.size(); i++)
					if (tvec.size() > 1 && tvec[tvec.size() - 2] == line[i])
						tvec.pop_back();
					else tvec.push_back(line[i]);
				if (tvec.size() < 4) line2.clear();
				tvec.push_back(tvec[1]);
				double totang = 0;
				for (int i = 1; i < tvec.size() - 1; i++){
					double t1 = atan2(point[tvec[i - 1]].y - point[tvec[i]].y, point[tvec[i - 1]].x - point[tvec[i]].x);
					double t2 = atan2(point[tvec[i + 1]].y - point[tvec[i]].y, point[tvec[i + 1]].x - point[tvec[i]].x);
					t2 -= t1;
					if (t2 < 0) t2 += 2 * pi;
					totang += t2;
				}
				fprintf(tout, "%d:	%f	%f answer: %c\n", s_tot + 1, totang, (tvec.size() - 2) * pi, totang >(tvec.size() - 2) * pi ? '+' : '-');
				if (totang > (tvec.size() - 3) * pi) line2.clear();
				std::vector<int> tline;
				bool *scanned = new bool[n];
				memset(scanned, 0, sizeof(bool) * n);
				for (auto t : line)
					scanned[t] = 1;
				for (auto t : line2)
					if (!scanned[t]) tline.push_back(t);
				line2 = tline;
				for (auto t : line2)
					scanned[t] = 1;
				//fprintf(tout, "line2size: %d	", line2.size());
				for (int t = 0; t < line2.size(); t ++ )
					for (auto i : list[line2[t]])
						if (edges[i.num].z < k && !scanned[i.to]){
							line2.push_back(i.to);
							//printf("out, %d\n", s_tot + 1);
							scanned[i.to] = 1;
						}
				if (line2.size() > 9999) line2.clear();
				fprintf(tout, "line2size - end: %d\n", line2.size());
				//line: points on edge; line2:points in area
				if (line2.size()){
					s_edge[++s_tot] = line;
					s_in[s_tot] = line2;
				}
				//printf("%d %d %d\n", s_tot, line.size(), line2.size());
				delete scanned;
			}
	fclose(tout);
}
void TDFP::divide_area2(int klevel){
	for (int i = 1; i < n; i++)
		for (int j = 0; j < list[i].size(); j++)
			if (!listdone[i][j] && edges[list[i][j].num].z < klevel){
				std::vector<std::pair<int, int>> l;
				l.push_back(std::make_pair(i, j));
				for (int t = 0; t < l.size(); t++){
					int y = l[t].first, x = l[t].second;
					if (listdone[y][x]) continue;
					listdone[y][x] = 1;
					x = list[y][x].to;
					int start = 0;
					for (int q = 0; q < list[x].size(); q++)
						if (list[x][q].to == y){
							start = q;
							break;
						}
					for (int k = start, qq = 0;; k++){
						//printf("%d ", k);
						if (k == list[x].size()) k = 0;
						if (++qq > list[x].size()) break;
						if (edges[list[x][k].num].z >= klevel) break;
						l.push_back(std::make_pair(x, k));
					}
					//printf("\n");
					for (int k = start, qq = 0;; k--){
						//printf("%d ", k);
						if (!~k) k = list[x].size() - 1;
						if (++qq > list[x].size()) break;
						if (edges[list[x][k].num].z >= klevel) break;
						l.push_back(std::make_pair(x, k));
					}
					//printf("\n");
				}
				std::set<int> line, line2;
				for (auto t : l){
					int pp = t.first;
					if (point_level[pp] >= klevel) line.insert(pp);
					else line2.insert(pp);
					pp = list[t.first][t.second].to;
					if (point_level[pp] >= klevel) line.insert(pp);
					else line2.insert(pp);
				}
				if (line.size() + line2.size() < 10){
					for (auto i : l)
						listdone[i.first][i.second] = 0;
					continue;
				}
				s_tot++;
				for (auto t : line){
					s_edge[s_tot].push_back(t);
					has_group[t] = s_tot;
				}
				for (auto t : line2){
					s_in[s_tot].push_back(t);
					has_group[t] = s_tot;
				}
				//printf("new area:%d %d %d\n", s_tot, line.size(), line2.size());
			}
#ifdef DEBUG_LITTLE
	bb.resize(n, 0);
	for (int i = s_tot; i; i--){
		for (auto j : s_edge[i])
			bb[j] = 1;
		for (auto j : s_in[i])
			bb[j] = 1;
	}
	int ttt = 0;
	for (int i = 0; i < bb.size(); i++)
		if (!list[i].size()) bb[i] = 1;
	printf("ttt: %d\n", ttt);
#endif
	/*
	for (int i = s_tot; i; i--){
		for (auto j : s_edge[i])
			add_empty(i, j);
	}
	*/
	int &totot = no_group_num;
	for (int i = 0; i < n; i++)
		if (list[i].size() && !has_group[i]) no_group_id[i] = ++totot;
	printf("no group point number : %d\n", totot);
}
void TDFP::add_empty(int s_num, int k){

}
void TDFP::calc_LTT(){
	dijkstra::dijkstra dijk;
	std::vector<sp_common::line> t_line;
	for (int i = 0; i < m; i++)
		t_line.push_back(sp_common::line(edges[i].x, edges[i].y, LBT[i]));
	dijk.init(n, t_line);
	for (int i = 1; i <= s_tot; i++){
		std::vector<double> res;
		dijk.several_start(s_edge[i], res);
#ifdef DEBUG_LITTLE
		double sum = 0;
		for (auto i : res)
			if (i != line_max) sum += i;
		printf("%d %d %e\n", i, s_tot, sum);
#endif
		for (auto j : s_in[i])
			LTT_p2e[j] = LTT_e2p[j] = res[j];
		for (int j = 1; j <= s_tot; j ++ ){
			LTT_e2e[i][j] = 1e100;
			for (auto k : s_edge[j])
				if (res[k] < LTT_e2e[i][j]) LTT_e2e[i][j] = res[k];
		}
		for (int j = 0; j < N; j++)
			if (no_group_id[j])
				no_group2e[i][no_group_id[j]] = res[j];
	}
}
double TDFP::single(int st, int ed, double start_time){
	auto cmp = [] (std::pair<double, int> x, std::pair<double, int> y) { return x > y; };
	std::vector<double> dis;
	dis.resize(n);
	for (int i = 0; i < n; i ++ )
		dis[i] = 1e100;
	std::vector<std::pair<double, int>> heap;
	heap.push_back(std::make_pair(start_time, st));
	dis[st] = start_time;
	for (; heap.size(); ){
		if (heap[0].second == ed) break;
		int now = heap[0].second;
		if (dis[now] + estimate_func(now, ed) != heap[0].first){
			std::pop_heap(heap.begin(), heap.end(), cmp);
			heap.pop_back();
			continue;
		}
		std::pop_heap(heap.begin(), heap.end(), cmp);
		heap.pop_back();
		for (auto i : list[now]){
			double nowtime = 0;
			auto k = std::lower_bound(edge_time[i.num].begin(), edge_time[i.num].end(), std::make_pair(dis[now], 0.0));
			auto k0 = k;
			k0 -- ;
			nowtime = k0 -> first + (k -> first - k0 -> first) / (k -> second - k0 -> second) * (dis[now] - k0 -> second);
			if (dis[i.to] > dis[now] + nowtime){
				dis[i.to] = dis[now] + nowtime;
				heap.push_back(std::make_pair(dis[i.to] + estimate_func(i.to, ed), i.to));
				std::push_heap(heap.begin(), heap.end(), cmp);
			}
		}
	}
	if (dis[ed] == 1e100) return - 1;
	return dis[ed];
}
double TDFP::bruteforce(int start, int end, double start_time){
	std::vector<double> dis;
	dis.resize(n);
	dis[start] = start_time;
	std::vector<int> l;
	l.push_back(start);
	for (int t = 0; t < l.size(); t++){
		int now = l[t];
		for (auto i : list[now]){
			double nowtime = 0;
			auto k = std::lower_bound(edge_time[i.num].begin(), edge_time[i.num].end(), std::make_pair(dis[now], 0.0));
			auto k0 = k;
			k0--;
			nowtime = k0->first + (k->first - k0->first) / (k->second - k0->second) * (dis[now] - k0->second);
			if (dis[i.to] > dis[now] + nowtime){
				dis[i.to] = dis[now] + nowtime;
				l.push_back(i.to);
			}
		}
	}
	return dis[end];
}
inline double TDFP::get_real_dis(int st, double time){

	//printf("-real dis: %d %f %f-\n", st, edges[st].add, edge_time[st][0].second);
	//return edges[st].add;

	for (; time > 86400; time -= 86400);
	auto pos =  std::upper_bound(edge_time[st].begin(), edge_time[st].end(), std::make_pair(time, 0.0));
	std::pair<double, double> &t2 = *pos;
	if (pos == edge_time[st].begin()) pos++;
	else pos--;
	std::pair<double, double> &t1 = *pos;
	if (t1.first == t2.first) printf("t1: %f %f, t2: %f %f\n", t1.first, t1.second, t2.first, t2.second);
	double ans = t1.second + (t2.second - t1.second) * (time - t1.first) / (t2.first - t1.first);
	return ans;
}
double TDFP::dijkstra(int st, int ed, double time, std::vector<int> &res){
#ifdef DEBUG_LITTLE
	vnum = 0;
#endif
	auto cmp = [](std::pair<double, int> a, std::pair<double, int> b) { return a > b; };
#ifdef DEBUG_LITTLE
	visited.clear();
#endif
	for (int i = 0; i < n; i++)
		dist[i] = 1e100;
	dist[st] = time;
	fa[st] = 0;
	std::vector<std::pair<double, int>> heap;
	heap.push_back(std::make_pair(time, st));
	for (; heap.size();){
		if (heap[0].second == ed) break;
		if (heap[0].first != dist[heap[0].second]){
			std::pop_heap(heap.begin(), heap.end(), cmp);
			heap.pop_back();
			continue;
		}
		int now = heap[0].second;
		std::pop_heap(heap.begin(), heap.end(), cmp);
		heap.pop_back();
#ifdef DEBUG_LITTLE
		visited.insert(now);
		vnum++;
#endif
		//printf("%d\n", now);
		for (int i = 0; i < list[now].size(); i++){
			double realdis = get_real_dis(list[now][i].num, dist[now]);
			int num = list[now][i].to;
			if (dist[num] > dist[now] + realdis){
				dist[num] = dist[now] + realdis;
				//printf("-%d to %d-\n", now, num);
				fa[num] = now;
				heap.push_back(std::make_pair(dist[num], num));
				std::push_heap(heap.begin(), heap.end(), cmp);
			}
		}
	}
	res.clear();
	if (dist[ed] > 1e10) return dist[ed];
	for (int i = ed; i; i = fa[i])
		res.push_back(i);
	std::reverse(res.begin(), res.end());
	printf("res size: %d\n", res.size());
	return dist[ed];
}
void TDFP::outputfile(){
	FILE *out = fopen("TDFP.txt", "w");
	fprintf(out, "%d\n", s_tot);
	for (int i = 1; i <= s_tot; i++){
		fprintf(out, "%d %d\n", s_edge[i].size(), s_in[i].size());
		for (auto j : s_edge[i])
			fprintf(out, "%d ", j);
		fprintf(out, "\n");
		for (auto j : s_in[i])
			fprintf(out, "%d ", j);
		fprintf(out, "\n");
	}
	for (int i = 0; i < n; i++)
		fprintf(out, "%f ", LTT_e2p[i]);
	fprintf(out, "\n");
	for (int i = 0; i < n; i++)
		fprintf(out, "%f ", LTT_p2e[i]);
	fprintf(out, "\n");
	for (int i = 1; i <= s_tot; i++){
		for (int j = 1; j <= s_tot; j++){
			fprintf(out, "%f ", LTT_e2e[i][j]);
			/*if (LTT_e2e[i][j] == 0 || j == s_tot){
				fprintf(ooo, "e2e zero: %d %d %f\n", i, j, LTT_e2e[i][j]);
				//getchar();
			}*/
		}
		fprintf(out, "\n");
	}
	fprintf(out, "%d\n", no_group_num);
	for (int i = 0; i < N; i++)
		fprintf(out, "%d ", no_group_id[i]);
	fprintf(out, "\n");
	fclose(out);
	FILE *out2 = fopen("NOGROUP.bin", "wb");
	int ans = 0;
	for (int i = 0; i < sizeof(no_group2e) / sizeof(no_group2e[0]); i ++ )
		ans += fwrite(no_group2e[i], sizeof(double), sizeof(no_group2e[i]) / sizeof(double), out2);
	printf("bin file write %d. size: %d\n", ans, sizeof(no_group2e) / sizeof(double));
	fclose(out2);
}
void TDFP::inputfile(){
	memset(has_group, 0, sizeof has_group);
	FILE *out = fopen("TDFP.txt", "r");
	fscanf(out, "%d", &s_tot);
	for (int i = 1; i <= s_tot; i++){
		int t1, t2;
		fscanf(out, "%d%d", &t1, &t2);
		s_edge[i].clear();
		s_in[i].clear();
		for (; t1--;){
			int tt;
			fscanf(out, "%d", &tt);
			s_edge[i].push_back(tt);
			has_group[tt] = i;
		}
		for (; t2--;){
			int tt;
			fscanf(out, "%d", &tt);
			s_in[i].push_back(tt);
			has_group[tt] = i;
		}
	}
	for (int i = 0; i < n; i++)
		fscanf(out, "%lf", &LTT_e2p[i]);
	for (int i = 0; i < n; i++)
		fscanf(out, "%lf", &LTT_p2e[i]);
#ifdef DEBUG_LITTLE
	int tototot = 0;
#endif;
	for (int i = 1; i <= s_tot; i++)
		for (int j = 1; j <= s_tot; j++){
			fscanf(out, "%lf", &LTT_e2e[i][j]);
#ifdef DEBUG_LITTLE
			if (!LTT_e2e[i][j]) tototot++;
#endif
		}
#ifdef DEBUG_LITTLE
	printf("zeros: %d / %d\n", tototot, s_tot);
#endif
	fscanf(out, "%d", &no_group_num);
	for (int i = 0; i < N; i++)
		fscanf(out, "%d", &no_group_id[i]);
	area_tot.clear();
	area_tot.resize(s_tot + 1, 0);
	for (auto i : has_group)
		area_tot[i]++;
	fclose(out);
	FILE *out2 = fopen("NOGROUP.bin", "rb");
	int ans = 0;
	for (int i = 0; i < sizeof(no_group2e) / sizeof(no_group2e[0]); i++)
		ans += fread(no_group2e[i], sizeof(double), sizeof(no_group2e[i]) / sizeof(double), out2);
	printf("bin file read %d.\n", ans);
	fclose(out2);
}
double TDFP::mainway2(int st, int ed, double time, std::vector<int> &route){
	calc_time++;
#ifdef DEBUG_LITTLE
	mv1num = mv2num = mv3num = 0;
#endif
	auto cmp = [](std::pair<double, int> a, std::pair<double, int> b) { return a > b; };
	std::vector<int> res;
	double nowmin = line_max;
	int nowid = -1;
	route.clear();
	std::vector<std::pair<double, int>> heap, heapb;
	heap.clear();
	heapb.clear();
#ifdef DEBUG_LITTLE
	mvis1.clear();
	mvis2.clear();
	mvis3.clear();
#endif
	heap.push_back(std::make_pair(last_esti[st] = time + estimate_func(st, ed), st));
	heapb.push_back(std::make_pair(last_estib[ed] = estimate_funcb(ed, st), ed));
	for (int i = 0; i < n; i++){
		dist[i] = distb[i] = 1e100;
		visb[i] = 0;
	}
	dist[st] = time;
	distb[ed] = 0;
	fa[st] = fab[ed] = 0;
	visb[ed] = 1;
	bool step_2 = 0;
	for (; heap.size() && heapb.size();){//printf("|%d %d %.0f %d %.0f %d|\n", heap.size(), heapb.size(), heap[0].first, heap[0].second, heapb[0].first, heapb[0].second);getchar();
		if (last_calc[heap[0].second] == calc_time){
			std::pop_heap(heap.begin(), heap.end(), cmp);
			heap.pop_back();
			continue;
		}
		if (last_calcb[heapb[0].second] == calc_time){
			std::pop_heap(heapb.begin(), heapb.end(), cmp);
			heapb.pop_back();
			continue;
		}
		if (heap[0].second == ed) break;
		if (heap[0].first > nowmin && heapb[0].first > nowmin) break;//break and forward go on with backward points
		int heaps1 = heap.size(), heaps2 = heapb.size();
		if (!heapb.size() || heap.size() && heap[0].first < heapb[0].first){
			int now = heap[0].second;
			last_calc[now] = calc_time;
#ifdef DEBUG_LITTLE
			mvis1.insert(now);
			mv1num++;
#endif
			if (nowmin == line_max && dist[now] + distb[now] < nowmin){
				nowid = now;
				goto CROSS;
			}
			std::pop_heap(heap.begin(), heap.end(), cmp);
			heap.pop_back();
			for (int i = 0; i < list[now].size(); i++){
				double realdis = get_real_dis(list[now][i].num, dist[now]);
				int num = list[now][i].to;
				double ested = estimate_func(num, ed, last_esti[now] - dist[now] - realdis);
				if (dist[num] > dist[now] + realdis){// && last_esti[num] != dist[num] + ested
					dist[num] = dist[now] + realdis;
					fa[num] = now;
					heap.push_back(std::make_pair(last_esti[num] = dist[num] + ested, num));
					//printf("%d %f %e\n", num, dist[num], ested);
					std::push_heap(heap.begin(), heap.end(), cmp);
				}
			}
		}
		else{
			int now = heapb[0].second;
			last_calcb[now] = calc_time;
			visb[now] = 1;
#ifdef DEBUG_LITTLE
			mvis2.insert(now);
			mv2num++;
#endif
			if (nowmin == line_max && dist[now] + distb[now] < nowmin){
				nowid = now;
				goto CROSS;
			}
			std::pop_heap(heapb.begin(), heapb.end(), cmp);
			heapb.pop_back();
			for (int i = 0; i < list[now].size(); i++){
				double realdis = LBT[list[now][i].num];
				int num = list[now][i].to;
				double ested = estimate_funcb(num, st, last_estib[now] - distb[now] - realdis);
				//if (ested != 0) printf("est sth!%f\n", distb[num] + ested);
				if (distb[num] > distb[now] + realdis){// && last_estib[num] != distb[num] + ested
					distb[num] = distb[now] + realdis;
					fab[num] = now;
					heapb.push_back(std::make_pair(last_estib[num] = distb[num] + ested, num));
					std::push_heap(heapb.begin(), heapb.end(), cmp);
				}
			}
		}
		continue;
	CROSS:;
		nowmin = dist[nowid];
		for (int i = nowid; i != ed; i = fab[i]){
			for (auto j : list[i])
				if (j.to == fab[i]){
					nowmin += get_real_dis(j.num, nowmin);
					break;
				}
		}
		printf("corss! nowmin:%f\n", nowmin);
	}



	for (; heap.size();){//printf("|%d %d %.0f %d %.0f %d|\n", heap.size(), heapb.size(), heap[0].first, heap[0].second, heapb[0].first, heapb[0].second);
		if (last_calc[heap[0].second] == calc_time){
			std::pop_heap(heap.begin(), heap.end(), cmp);
			heap.pop_back();
			continue;
		}
		if (heap[0].second == ed) break;
		int now = heap[0].second;
		last_calc[now] = calc_time;
#ifdef DEBUG_LITTLE
		mvis3.insert(now);
		mv3num++;
#endif
		std::pop_heap(heap.begin(), heap.end(), cmp);
		heap.pop_back();
		for (int i = 0; i < list[now].size(); i++){
			if (!visb[list[now][i].to]) continue;
			double realdis = get_real_dis(list[now][i].num, dist[now]);
			int num = list[now][i].to;
			double ested = estimate_func(num, ed, last_esti[now] - dist[now] - realdis);
			if (dist[num] > dist[now] + realdis){// && last_esti[num] != dist[num] + ested
				dist[num] = dist[now] + realdis;
				fa[num] = now;
				heap.push_back(std::make_pair(last_esti[num] = dist[num] + ested, num));
				std::push_heap(heap.begin(), heap.end(), cmp);
			}
		}
	}


	res.clear();
	for (int j = ed; j; j = fa[j])
		res.push_back(j);
	std::reverse(res.begin(), res.end());
	//if (res.size() && res[0] != st) res.clear();
	route = res;
	return dist[ed];
}
double TDFP::oneside(int st, int ed, double time, std::vector<int> &route){
	calc_time++;
#ifdef DEBUG_LITTLE
	mv1num = 0;
#endif
	auto cmp = [](std::pair<double, int> a, std::pair<double, int> b) { return a > b; };
	std::vector<int> res;
	double nowmin = line_max;
	int nowid = -1;
	route.clear();
	std::vector<std::pair<double, int>> heap;
	heap.clear();
#ifdef DEBUG_LITTLE
	mvis1.clear();
#endif
	heap.push_back(std::make_pair(last_esti[st] = time + estimate_func(st, ed), st));
	for (int i = 0; i < n; i++){
		dist[i] = 1e100;
	}
	dist[st] = time;
	fa[st] = 0;

	for (; heap.size();){//printf("|%d %d %.0f %d %.0f %d|\n", heap.size(), heapb.size(), heap[0].first, heap[0].second, heapb[0].first, heapb[0].second);
		if (heap[0].first != last_esti[heap[0].second]){// last_calc[heap[0].second] == calc_time){
			std::pop_heap(heap.begin(), heap.end(), cmp);
			heap.pop_back();
			continue;
		}
		if (heap[0].second == ed) break;
		int now = heap[0].second;
		last_calc[now] = calc_time;
#ifdef DEBUG_LITTLE
		mvis1.insert(now);
		mv1num++;
#endif
		std::pop_heap(heap.begin(), heap.end(), cmp);
		heap.pop_back();
		for (int i = 0; i < list[now].size(); i++){
			double realdis = get_real_dis(list[now][i].num, dist[now]);
			int num = list[now][i].to;
			double ested = estimate_func(num, ed, last_esti[now] - dist[now] - realdis);
			if (dist[num] > dist[now] + realdis){// && last_esti[num] != dist[num] + ested
				//if (last_calc[num] == calc_time) printf("%d %f %f %f %f\n", num, dist[num], dist[now] + realdis, last_esti[num], dist[now] + realdis + ested);
				last_calc[num] = 0;
				dist[num] = dist[now] + realdis;
				fa[num] = now;
				heap.push_back(std::make_pair(last_esti[num] = dist[num] + ested, num));
				std::push_heap(heap.begin(), heap.end(), cmp);
			}
		}
	}


	res.clear();
	for (int j = ed; j; j = fa[j])
		res.push_back(j);
	std::reverse(res.begin(), res.end());
	route = res;
	return dist[ed];
}
double TDFP::mainway(int st, int ed, double time, std::vector<int> &route){
	//calc_time++;
#ifdef DEBUG_LITTLE
	mv2num = 0;
#endif
	auto cmp = [](std::pair<double, int> a, std::pair<double, int> b) { return a > b; };
	for (int i = 0; i < s_tot; i++){
		s_heap[i].clear();
		now_in[i] = 0;
	}
	std::vector<int> res;
	double nowmin = line_max;
	int nowid = -1;
	route.clear();
	std::vector<std::pair<double, int>> heap;
	heap.clear();
#ifdef DEBUG_LITTLE
	mvis2.clear();
#endif
	heap.push_back(std::make_pair(last_esti[st] = time + estimate_func(st, ed), st));
	now_in[has_group[st]]++;
	for (int i = 0; i < n; i++){
		dist[i] = 1e100;
	}
	dist[st] = time;
	fa[st] = 0;

	for (; heap.size();){//printf("|%d %d %.0f %d %.0f %d|\n", heap.size(), heapb.size(), heap[0].first, heap[0].second, heapb[0].first, heapb[0].second);
		if (heap[0].first != last_esti[heap[0].second]){// last_calc[heap[0].second] == calc_time){
			now_in[has_group[heap[0].second]]--;
			std::pop_heap(heap.begin(), heap.end(), cmp);
			heap.pop_back();
			continue;
		}
		if (heap[0].second == ed) break;// && (!has_group[ed] || !s_heap[has_group[ed]].size() || s_heap[has_group[ed]][0].first > dist[ed])
		int now = heap[0].second;
		int nowg = has_group[now];
		//printf("%d %d %d %d\n", now, nowg, now_in[nowg], heap.size());
		//getchar();
		//last_calc[now] = calc_time;
#ifdef DEBUG_LITTLE
		mvis2.insert(now);
		mv2num++;
#endif
		std::pop_heap(heap.begin(), heap.end(), cmp);
		heap.pop_back();
		now_in[nowg]--;
		for (int i = 0; i < list[now].size(); i++){
			double realdis = get_real_dis(list[now][i].num, dist[now]);
			int num = list[now][i].to;
			double ested = estimate_func(num, ed, last_esti[now] - dist[now] - realdis);
			if (dist[num] > dist[now] + realdis){// && last_esti[num] != dist[num] + ested
				//if (last_calc[num] == calc_time) printf("%d %f %f %f %f\n", num, dist[num], dist[now] + realdis, last_esti[num], dist[now] + realdis + ested);
				//last_calc[num] = 0;
				dist[num] = dist[now] + realdis;
				fa[num] = now;
				if (!nowg || nowg == has_group[num] && (!s_heap[nowg].size() || s_heap[nowg][0].first >= dist[num])){
					now_in[has_group[num]]++;
					heap.push_back(std::make_pair(last_esti[num] = dist[num] + ested, num));
					std::push_heap(heap.begin(), heap.end(), cmp);
				}
				else{
					s_heap[nowg].push_back(std::make_pair(dist[num], num));// + (has_group[num] ? LTT_p2e[num] : 0)
					std::push_heap(s_heap[nowg].begin(), s_heap[nowg].end(), cmp);
					last_esti[num] = dist[num] + ested;
				}
			}
		}
		if (!now_in[nowg] && s_heap[nowg].size()){
			do{
				int num = s_heap[nowg][0].second;
				now_in[has_group[num]]++;
				heap.push_back(std::make_pair(last_esti[num], num));
				std::push_heap(heap.begin(), heap.end(), cmp);
				std::pop_heap(s_heap[nowg].begin(), s_heap[nowg].end(), cmp);
				s_heap[nowg].pop_back();
			} while (s_heap[nowg].size() && (has_group[s_heap[nowg][0].second] == nowg || !now_in[nowg]));
		}
	}


	res.clear();
	for (int j = ed; j; j = fa[j])
		res.push_back(j);
	std::reverse(res.begin(), res.end());
	route = res;
	return dist[ed];
}
void TDFP::init_timedata(const int *normalspeed){
	for (int i = 0; i < edges.size(); i++)
		for (int j = 0; j < TDFP_BLOCKS; j++)
			edgetottime[j].push_back(std::make_pair(1.0, normalspeed[10 - edges[i].z]));
	for (int i = 15; i--; )
		roadspeed.push_back(std::make_pair(0.0, 0.0));
	//printf("init timedata ok! %d\n", edgetottime[0].size());
#ifdef TDFP_DEBUG
	outtemp = fopen("../temp.txt", "w");
#endif
}
void TDFP::update_timedata(std::vector<int> &inedges, int timegroup, double speed, double effect, int timespan){
	if (isnan(speed)) return;
#ifdef TDFP_DEBUG
	fprintf(outtemp, "update_timedata[%d] timegroup:%d, speed %f, effect %f\n", inedges.size(), timegroup, speed, effect);
	fflush(outtemp);
#endif
	for (auto i : inedges){
#ifdef TDFP_DEBUG
		fprintf(outtemp, "doing %d\n", i);
		fflush(outtemp);
#endif
		for (int j = 0; j <= timespan; j++){
			edgetottime[(j + timegroup) % TDFP_BLOCKS][i].first += effect * (1 - j / (timespan + 1.0));
			edgetottime[(j + timegroup) % TDFP_BLOCKS][i].second += effect * (1 - j / (timespan + 1.0)) * speed;
			if (j){
				edgetottime[(TDFP_BLOCKS + timegroup - j) % TDFP_BLOCKS][i].first += effect * (1 - j / (timespan + 1.0));
				edgetottime[(TDFP_BLOCKS + timegroup - j) % TDFP_BLOCKS][i].second += effect * (1 - j / (timespan + 1.0)) * speed;
			}
		}
	}
}
void TDFP::update_timedata(std::vector<taxi_detail> &Taxi_Detail, std::vector<bool> & choose, int timespan, int areaspan){
	int now = 0;
	for (; now < Taxi_Detail.size() && (Taxi_Detail[now].tag == -1 || !choose[now]); now++);
	for (;;){
		update_timedata_time++;
		if (now > Taxi_Detail.size()) return;
#ifdef TDFP_DEBUG
		fprintf(outtemp, "now: %d %d\n", now, Taxi_Detail[now].tag);
		fflush(outtemp);
#endif
		int st = Taxi_Detail[now].tag;
		int timegroup = Taxi_Detail[now].time / TDFP_timespan;
#ifdef TDFP_DEBUG
		fprintf(outtemp, "timeread: %f\n", Taxi_Detail[now].time);
		fflush(outtemp);
#endif
		double dist = 0, time = 0;
		for (now++;;){
			if (now > Taxi_Detail.size()) break;
			if (choose[now]){
				time += now ? Taxi_Detail[now].time - Taxi_Detail[now - 1].time : Taxi_Detail[now].time;
				dist += Taxi_Detail[now].dis;
				if (Taxi_Detail[now].tag == -1) now++;
				else break;
			}
			else now++;
		}
		if (now > Taxi_Detail.size()) return;
		if (time <1e-3) continue;
		int ed = Taxi_Detail[now].tag;
		std::vector<int> inedges;
		int num = -1;
		for (int i = 0; i < list[st].size(); i++)
			if (list[st][i].to == ed)
				num = list[st][i].num;
		if (num < 0 || !(timegroup >= 0 && timegroup < TDFP_BLOCKS)) continue;
		inedges.push_back(num);
		last_update_timedata[num] = update_timedata_time;
		double noweffect = 1;
		dist = dist * __convert_to_km / (time / 3600);
		if (!isnan(dist)){
			roadspeed[edges[num].z].first += 1;
			roadspeed[edges[num].z].second += dist;
		}
		for (; noweffect > 1e-5;){
			update_timedata(inedges, timegroup, dist, noweffect, timespan);
			noweffect -= 1.0 / (areaspan + 1);
			if (noweffect < 1e-5) break;
			std::vector<int> tmp;
			for (auto i : inedges){
				for (auto j : list[edges[i].x])
					if (last_update_timedata[j.num] != update_timedata_time){
						last_update_timedata[j.num] = update_timedata_time;
						tmp.push_back(j.num);
					}
				for (auto j : list[edges[i].y])
					if (last_update_timedata[j.num] != update_timedata_time){
						last_update_timedata[j.num] = update_timedata_time;
						tmp.push_back(j.num);
					}
			}
			inedges = tmp;
		}
	}
}
void TDFP::print_timedata_result(const char *InputFile){
	FILE *cout = fopen(InputFile, "w");
	fprintf(cout, "%d %d\n", point.size(), edges.size());
	for (int i = 0; i < point.size(); i++)
		fprintf(cout, "%d %d\n", (int)(point[i].x), (int)(point[i].y));
	for (auto i : edges)
		fprintf(cout, "%d %d %d\n", i.x, i.y, 10 - i.z);

	//FILE *ccc = fopen("../temp.txt", "w");

	for (int i = 0; i < edges.size(); i++){
		bool print = 0;
		for (int j = 0; j < TDFP_BLOCKS; j++)
			if (edgetottime[j][i].first != 1) print = 1;
		//if (!print) continue;
		double distance = (point[edges[i].x] - point[edges[i].y]).len() / 10000000;
		//fprintf(ccc, "%d %f %f %f\n", i, distance, edges[i].add, distance - edges[i].add);
		distance = distance * __convert_to_km * 1000;
		distance = distance * 3.6;
		fprintf(cout, "%d", TDFP_BLOCKS + 1);
		for (int j = 0; j < TDFP_BLOCKS; j++)
			fprintf(cout, " %d %.6f", j * TDFP_timespan, distance / (edgetottime[j][i].second / edgetottime[j][i].first));
		fprintf(cout, " %d %.6f\n", 86400, distance / (edgetottime[0][i].second / edgetottime[0][i].first));
	}
	fclose(cout);
#ifdef TDFP_DEBUG
	fclose(outtemp);
#endif
	//output average speed
	/*
	cout = fopen("../temp.txt", "w");
	for (int j = 0; j < roadspeed.size(); j++){
		auto i = roadspeed[j];
		if (i.first != 0) fprintf(cout, "%f %e %e\n", i.second / i.first, i.first, i.second);
		else fprintf(cout, "no data\n");
	}
	fclose(cout);*/
}