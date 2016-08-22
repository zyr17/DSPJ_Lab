#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__
#include <cmath>
#include <algorithm>

namespace geometry{

	static double eps = 1e-8;
	static double pi = 3.14159265358979323846;

	static int dcmp(double x){
		if (x > eps) return 1;
		if (x < -eps) return -1;
		return 0;
	}

	struct Point {
		double x, y;
		int _re;
		Point(){}
		Point(double x, double y, int _re = - 1) : x(x), y(y), _re(_re) {}
		Point operator - (const Point &b) {
			return Point(x - b.x, y - b.y);
		}
		Point operator + (const Point &b) {
			return Point(x + b.x, y + b.y);
		}
		Point operator * (const double &b) {
			return Point(x * b, y * b);
		}
		Point operator / (const double &b) {
			return Point(x / b, y / b);
		}
		Point rot90(int t) { return Point(-y, x) * t; }
		Point rot(double ang) {
			return Point(x * cos(ang) - y * sin(ang), x * sin(ang) + y * cos(ang));
		}
		double ang() {
			double res = atan2(y, x); if (dcmp(res) < 0) res += pi * 2; return res;
		}
		double operator * (const Point &b) {
			return x * b.y - y * b.x;
		}
		double operator % (const Point &b) {
			return x * b.x + y * b.y;
		}
		double len2() { return x * x + y * y; }
		double len() { return sqrt(x * x + y * y); }
	};

	inline double xmul(Point a, Point b, Point c);
	double disLP(Point p1, Point p2, Point q);
	double dis_Seg_P(Point p1, Point p2, Point q);
	Point proj(Point p1, Point p2, Point q);
	bool is_point_onseg(Point p1, Point p2, Point P);
}
#endif