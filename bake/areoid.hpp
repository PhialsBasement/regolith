#pragma once

#include <string>
#include <vector>

namespace regolith::bake {

// MOLA MEGDR areoid (mega90n000cb.img): 1440 x 720 MSB int16,
// radius = value + 3396000 m. Bilinear, lon periodic.
class Areoid {
public:
	static constexpr int W = 1440;
	static constexpr int H = 720;

	bool open(const std::string &p_path);
	double radius(double p_lat, double p_lon) const;

private:
	std::vector<double> r_;
};

} // namespace regolith::bake
