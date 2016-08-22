#include "spfa.h"

namespace spfa{
	void spfa::clear(){
		tot = 0;
		tail.clear();
		next.clear();
		num.clear();
		dis.clear();
		val.clear();
		fa.clear();
		isin.clear();
	}
	inline void spfa::AddEdge(int x, int y, double z){
		num.push_back(y);
		val.push_back(z);
		next.push_back(-1);
		tail[x] = next[tail[x]] = tot++;
	}

	void spfa::do_mul_spfa(int st, std::vector<int> &ed, std::vector<double> &dist){
		std::vector<int> tmp, tmp2;
		do_mul_spfa(st, ed, dist, tmp, tmp2);
	}
	void spfa::do_mul_spfa(int st, std::vector<int> &ed, std::vector<double> &dist, std::vector<int> &r_start, std::vector<int> &route){
		l.clear();
		dis.clear();
		isin.clear();
		fa.clear();
		dist.clear();
		r_start.clear();
		route.clear();
		std::vector<int> res;
		fa.resize(n, -1);
		isin.resize(n);
		dis.resize(n, line_max);
		l.push_back(st);
		dis[st] = 0;
		for (int t = 0; t < l.size(); t++){
			double nowdis = dis[l[t]];
			isin[l[t]] = 0;
			if (ed.size() == 1 && dis[ed[0]] < dis[l[t]]) continue;
			for (int j = next[l[t]]; ~j; j = next[j])
				if (dis[num[j]] > nowdis + val[j]){
					dis[num[j]] = nowdis + val[j];
					fa[num[j]] = l[t];
					if (!isin[num[j]]){
						isin[num[j]] = 1;
						l.push_back(num[j]);
					}
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

	spfa::spfa(){

	}
	void spfa::init(char* file_name){
		/* TODO: Input from File */
	}
	void spfa::init(int tot_point, std::vector<sp_common::line> &lines){
		clear();
		n = tot_point;
		m = lines.size();
		tail.resize(n);
		next.resize(n, -1);
		num.resize(n);
		val.resize(n);
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
	double spfa::one_end(int st, int ed){
		std::vector<int> eds;
		eds.push_back(ed);
		std::vector<double> ans;
		several_end(st, eds, ans);
		return ans[0];
	}
	double spfa::one_end(int st, int ed, std::vector<int> &res){
		std::vector<int> eds, tmp;
		eds.push_back(ed);
		std::vector<double> ans;
		several_end(st, eds, ans, tmp, res);
		return ans[0];
	}
	void spfa::several_end(int st, std::vector<int> &ed, std::vector<double> &dist){
		do_mul_spfa(st, ed, dist);
	}
	void spfa::several_end(int st, std::vector<int> &ed, std::vector<double> &dist, std::vector<int> &r_start, std::vector<int> &route){
		do_mul_spfa(st, ed, dist, r_start, route);
	}
}