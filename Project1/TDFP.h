#pragma once
#include <vector>
#include <set>
#include <algorithm>
#include "dijkstra.h"
#include "common.h"
//#define TDFP_DEBUG
#define N 2222222
struct Two{
	int x, y;
	Two(){}
	Two(int x, int y) : x(x), y(y) {}
	Two operator- (Two b){
		return Two(x - b.x, y - b.y);
	}
	double len(){
		return sqrt(1.0 * x * x + 1.0 * y * y);
	}
};
struct Three{
	int x, y, z;
	double add;
	Three(){}
	Three(int x, int y, int z) : x(x), y(y), z(z) {}
	Three(int x, int y, int z, double add) : x(x), y(y), z(z), add(add) {}
};
struct Four{
	int to, num, rev;
	double d;
	Four(){}
	Four(int to, int num, int rev, double d) : to(to), num(num), rev(rev), d(d) {}
};
class TDFP{
private:
#define TDFP_BLOCKS 48
#define TDFP_timespan (86400 / TDFP_BLOCKS)
	std::vector<Four> list[N];//to: connect to point; num: edge num; rev: is rev(0/1); d:degree
	std::vector<bool> listdone[N];
	std::vector<std::pair<double, double>> edge_time[N];//first: time; second: cost
	std::vector<double> LBT;//lower bound time
	double LTT_p2e[N], LTT_e2p[N], LTT_e2e[2222][2222];
	int no_group_num;
	double no_group2e[1333][77777];
	bool done[N][2], visb[N];
	double dist[N], distb[N];
	double last_esti[N], last_estib[N];
	int fa[N], fab[N];
	inline double estimate_func(int now, int dest, double last = 0);
	inline double estimate_funcb(int now, int dest, double last = 0);
	void add_empty(int s_num, int k);
	inline double get_real_dis(int st, double time);
	int calc_time = 0;
	int last_calc[N], last_calcb[N];
	int last_update_timedata[N];
	int update_timedata_time;
	int now_in[2222];
	std::vector<std::pair<double, int>> s_heap[2222];
public:
	std::vector<Three> edges;//x, y:point; z:road level
	std::vector<std::pair<double, double>> edgetottime[TDFP_BLOCKS], roadspeed;
	std::vector<int> s_edge[N], s_in[N];
	std::vector<Two> point;
	std::vector<int> point_level;
	std::vector<int> area_tot;
	int no_group_id[N];
#ifdef DEBUG_LITTLE
	std::vector<bool> bb;
	std::set<int> visited, mvis1, mvis2, mvis3;
	int vnum, mv1num, mv2num, mv3num;
#endif
#ifdef TDFP_DEBUG
	FILE *outtemp;
#endif
	int has_group[N];
	int n, m, s_tot;
	void clear();
	void init_base(int n, int m, std::vector<Two> &point, std::vector<Three> &input);
	void init_time(std::vector<double> &input);
	void divide_area(int level);
	void divide_area2(int level);
	void calc_LTT();
	double single(int start, int end, double start_time);
	double bruteforce(int start, int end, double start_time);
	double dijkstra(int st, int ed, double time, std::vector<int> &res);
	void outputfile();
	void inputfile();
	double mainway(int st, int ed, double time, std::vector<int> &res);
	double mainway2(int st, int ed, double time, std::vector<int> &res);
	double oneside(int st, int ed, double time, std::vector<int> &res);
	void init_timedata(const int *normalspeed);
	void update_timedata(std::vector<int> &inedges, int timegroup, double speed, double effect, int timespan);
	void update_timedata(std::vector<taxi_detail> &Taxi_Detail, std::vector<bool> & choose, int timespan, int areaspan);
	void print_timedata_result(const char *InputFile);
};
