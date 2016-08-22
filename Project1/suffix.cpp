#include "suffix.h"

namespace sa{

	suffix_array::suffix_array(){
		clear();
	}
	bool suffix_array::cmp(int *r, int a, int b, int l){
		return r[a] == r[b] && r[a + l] == r[b + l];
	}
	void suffix_array::da(int *r, int *sa, int n, int m){
		int i, j, p, *x = wa, *y = wb, *t;
		memset(ws, 0, sizeof ws);
		for (i = 0; i < n; i++) ws[x[i] = r[i]] ++;
		for (i = 1; i < m; i++) ws[i] += ws[i - 1];
		for (i = n - 1; i >= 0; i--) sa[--ws[x[i]]] = i;
		for (j = 1, p = 1; p < n; j *= 2, m = p){
			for (p = 0, i = n - j; i < n; i++) y[p++] = i;
			for (i = 0; i < n; i++) if (sa[i] >= j) y[p++] = sa[i] - j;
			for (i = 0; i < n; i++) wv[i] = x[y[i]];
			memset(ws, 0, sizeof ws);
			for (i = 0; i < n; i++) ws[wv[i]] ++;
			for (i = 1; i < m; i++) ws[i] += ws[i - 1];
			for (i = n - 1; i >= 0; i--) sa[--ws[wv[i]]] = y[i];
			for (t = x, x = y, y = t, p = 1, i = 1, x[sa[0]] = 0; i < n; i++)
				x[sa[i]] = cmp(y, sa[i], sa[i - 1], j) ? p - 1 : p++;
		}
		return;
	}
	void suffix_array::gua(int *r, int *sa, int n){
		int i, j, k = 0;
		for (i = 1; i <= n; i++) rank[sa[i]] = i;
		for (i = 0; i < n; h[rank[i++]] = k)
		for (k ? k-- : 0, j = sa[rank[i] - 1]; r[i + k] == r[j + k]; k++);
		return;
	}
	void suffix_array::clear(){
		memset(sa, 0, sizeof sa);
		memset(rank, 0, sizeof rank);
		memset(h, 0, sizeof h);
		memset(c, 0, sizeof c);
		tot_length = n = 0;
	}
	void suffix_array::init(std::vector<std::string> input_string, int num){
		clear();
		if (num == -1) num = input_string.size();
		n = num;
		for (int i = 0; i < num; i++)
			tot_length += input_string[i].size() + 1;
		int now = 0;
		for (int i = 0; i < num; i++){
			memcpy(c + now, input_string[i].c_str(), input_string[i].size());
			now += input_string[i].size();
			now++;
		}

#ifdef CASE_INSENSITIVE
		for (int i = 0; i < tot_length; i ++ )
			if (c[i] >= 'A' && c[i] <= 'Z'){
				c[i] += 'a' - 'A';
			}
#endif

		c[tot_length++] = 0;
		for (int i = 0; i < tot_length; i++)
		if (c[i]) cc[i] = (unsigned char)c[i];
		else cc[i] = SUFFIX_MAX_CHAR;
		da(cc, sa, tot_length, SUFFIX_MAX_CHAR + 1);
		gua(cc, sa, --tot_length);
		now = 0;
		for (int i = 0; i < num; i++)
		for (int j = 0; j <= input_string[i].size(); j++)
			ori[rank[now++]] = i;
	}
	int suffix_array::name_search(const char *s, std::vector<int> &result){
		return suffix_array::name_search(s, result, INT_MAX);
	}
	/*int suffix_array::name_search(std::string s, std::vector<int> &result){
		return suffix_array::name_search(s.c_str(), result, INT_MAX);
	}
	int suffix_array::name_search(std::string s, std::vector<int> &result, int maxnum){
		return suffix_array::name_search(s.c_str(), result, maxnum);
	}*/
	int suffix_array::name_search(const char* init, std::vector<int> &result, int maxnum){
		Search_times++;
		int len = strlen(init);
		for (int i = 0; i < len; i++)
			s[i] = (unsigned char)init[i];
		s[len] = 0;

#ifdef CASE_INSENSITIVE
		for (int i = 0; i < len; i ++ )
			if (s[i] >= 'A' && s[i] <= 'Z'){
				s[i] += 'a' - 'A';
			}
#endif

		result.clear();
		int l = 0, r = tot_length;//find lower_bound, in r.
		for (; l + 1 < r;){
			int p = (l + r) / 2;
			if ([=]() -> bool{
				for (int i = sa[p]; i < tot_length; i++)
					if (s[i - sa[p]] < cc[i]) return 0;
					else if (s[i - sa[p]] > cc[i]) return 1;
				return 0;
			}()) l = p;
			else r = p;
		}
		for (int i = sa[r]; i < len + sa[r]; i++)
		if (s[i - sa[r]] != cc[i]) return 0;
		for (int i = r; i < tot_length && result.size() < maxnum; i++){
			if (last_search[ori[i]] != Search_times) result.push_back(ori[i]);
			last_search[ori[i]] = Search_times;
			if (h[i + 1] < len) break;
		}
		return result.size();
	}

}