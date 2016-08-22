#pragma once

#define DOUBLE_WAY

const double line_max = 1e100;
namespace sp_common{
	struct line{
		int st, ed;
		double dist;
		line(){}
		line(int st, int ed, double dist) : st(st), ed(ed), dist(dist) {}
	};

	struct mid_point{
		int x, y;
		double z, z2;
		mid_point(){}
		mid_point(int x, int y, double z, double z2) : x(x), y(y), z(z), z2(z2) {}
	};
}