#include "geometry.h"


namespace geometry{
	inline double xmul(Point a, Point b, Point c) {
		return (b - a) * (c - a);

	}
	double disLP(Point p1, Point p2, Point q) {
		return fabs((p1 - q) * (p2 - q)) / (p1 - p2).len();

	}
	double dis_Seg_P(Point p1, Point p2, Point q) {
		if ((p2 - p1) % (q - p1) < eps) return (q - p1).len();
		if ((p1 - p2) % (q - p2) < eps) return (q - p2).len();
		return disLP(p1, p2, q);
	}
	Point proj(Point p1, Point p2, Point q) {
		return p1 + ((p2 - p1) * ((p2 - p1) % (q - p1) / (p2 - p1).len2()));
	}
	bool is_point_onseg(Point p1, Point p2, Point P)
	{
		if (!(std::min(p1.x, p2.x) - eps <= P.x && P.x - eps <= std::max(p1.x, p2.x) && std::min(p1.y, p2.y) - eps <= P.y && P.y - eps <= std::max(p1.y, p2.y))) return false;
		if (dcmp((P - p1)*(p2 - p1)) == 0) return true;
		return false;
	}
}