#pragma once

#include <fstream>
#include <vector>
#include <iostream>


namespace VisLib {

	struct Point {
		double m_x, m_y;

		Point() = default;

		Point(const double pnt, const double pnt1):
		m_x(pnt),
		m_y(pnt1)
		{}

		friend std::ostream& operator<<(std::ostream& os, const Point& s) {
			os << "(" << s.m_x << ", " << s.m_y << ")";
			return os;
		}

	};
	struct Segment {
		int id;
		Point m_points[2];

		Segment() = default;

		Segment(const int segment_id, const Point& p1, const Point& p2) :
			id(segment_id),
			m_points{p1, p2}
		{}
		friend std::ostream& operator<<(std::ostream& os, const Segment& s) {
			os << "Segment id=" << s.id
			   << ", p1=" << s.m_points[0] << ", p2=" << s.m_points[1];
			return os;
		}
	};

	//////////////////////////////////////////////////////////////////////////////
	inline void writeFile(const std::string& filename, const std::vector<Segment>& segments, const std::vector<Point>& polygon) {
		const size_t n = segments.size();
		const size_t m = polygon.size();
		double pnts[4];
		std::ofstream ofs(filename, std::ios::binary);

		if (!ofs.is_open()) {
			std::cerr << "file could'n open for writing" << std::endl;
			return;
		}

		ofs.write(reinterpret_cast<const char*>(&n), sizeof(n));
		for (const auto& s : segments) {
			const auto& p = s.m_points;
			pnts[0] = p[0].m_x;
			pnts[1] = p[0].m_y;
			pnts[2] = p[1].m_x;
			pnts[3] = p[1].m_y;
			ofs.write(reinterpret_cast<const char*>(pnts), sizeof(pnts));
		}

		ofs.write(reinterpret_cast<const char*>(&m), sizeof(m));
		for (const auto& p : polygon) {
			ofs.write(reinterpret_cast<const char*>(&p), sizeof(Point));
		}
	}

	//////////////////////////////////////////////////////////////////////////////
	inline void readFile(const std::string& filename, std::vector<Segment>& segments, std::vector<Point>& polygon) {
		size_t n;
		size_t m;
		double pnts[4];
		Point p;
		std::ifstream ifs(filename, std::ios::binary);

		if (!ifs.is_open()) {
			std::cerr << "file could'n open for reading" << std::endl;
			return;
		}

		ifs.read(reinterpret_cast<char*>(&n), sizeof(n));

		segments.reserve(n);
		for (size_t i = 0; i < n; i++) {
			ifs.read(reinterpret_cast<char*>(pnts), sizeof(pnts));
			segments.emplace_back(int(i + 1), Point{ pnts[0], pnts[1] }, Point{ pnts[2], pnts[3] });
		}

		ifs.read(reinterpret_cast<char*>(&m), sizeof(m));
		polygon.reserve(m);
		for (size_t i = 0; i < m; i++) {
			ifs.read(reinterpret_cast<char*>(&p), sizeof(Point));
			polygon.push_back(p);
		}
	}

	inline bool intersect(Segment segment, Point n1, Point n2) {
		auto xs1 = segment.m_points[0].m_x;
		auto ys1 = segment.m_points[0].m_y;
		auto xs2 = segment.m_points[1].m_x;
		auto ys2 = segment.m_points[1].m_y;
		auto xp1 = n1.m_x;
		auto yp1 = n1.m_y;
		auto xp2 = n2.m_x;
		auto yp2 = n2.m_y;

		double dx1 = xs2 - xs1;
		double dy1 = ys2 - ys1;
		double dx2 = xp2 - xp1;
		double dy2 = yp2 - yp1;

		double det = dx1 * dy2 - dy1 * dx2;

		if (std::abs(det) < 1e-10) {
			return false;
		}

		double t = ((xp1 - xs1) * dy2 - (yp1 - ys1) * dx2) / det;
		double u = ((xp1 - xs1) * dy1 - (yp1 - ys1) * dx1) / det;

		return (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0);
	}

}
