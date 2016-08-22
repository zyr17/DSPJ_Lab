#ifndef __SUFFIX_H__
#define __SUFFIX_H__
#include <cstring>
#include <vector>
#include <iostream>
#define SUFFIX_N 600000
#define SUFFIX_MAX_CHAR 256
#define CASE_INSENSITIVE

namespace sa{

	class suffix_array{

	private:
		int cc[SUFFIX_N], sa[SUFFIX_N], h[SUFFIX_N], rank[SUFFIX_N], wa[SUFFIX_N], ws[SUFFIX_N], wb[SUFFIX_N], wv[SUFFIX_N], tot_length, ori[SUFFIX_N], n, Search_times, last_search[SUFFIX_N], s[SUFFIX_N];
		char c[SUFFIX_N];
		bool cmp(int *r, int a, int b, int l);
		void da(int *r, int *sa, int n, int m);
		void gua(int *r, int *sa, int n);

	public:
		suffix_array();
		void clear();
		void init(std::vector<std::string> input_string, int num = - 1);
		//int name_search(std::string name, std::vector<int> &result);
		int name_search(const char* name, std::vector<int> &result);
		//int name_search(std::string name, std::vector<int> &result, int maxnum);
		int name_search(const char* name, std::vector<int> &result, int maxnum);
	};

}
#endif
