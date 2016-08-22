#include "maker.h"
namespace maker{
	int n = 10, m = 10;
	void make_input(const char *input_file){
		srand(unsigned(time(NULL) + clock()));
		FILE *cout = fopen(input_file, "r");
		fprintf(cout, "%d %d\n", n, m);
		for (int i = 1; i <= n; i++)
			fprintf(cout, "%d %d\n", rand(), rand());
		for (int i = m; i--;){
			fprintf(cout, "%d %d %d\n", rand() % n, rand() % n, rand());
		}
	}
}