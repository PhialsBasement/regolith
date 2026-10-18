#include "mars_dem.hpp"

#include <cmath>
#include <cstdio>
#include <limits>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "parallel.hpp"

namespace regolith::bake {

namespace {

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

} // namespace

MarsDem::~MarsDem() {
	if (map_) {
		munmap(map_, map_size_);
	}
}

bool MarsDem::open(const std::string &p_path, int p_mip_count) {
	const int fd = ::open(p_path.c_str(), O_RDONLY);
	if (fd < 0) {
		std::fprintf(stderr, "mars dem: cannot open %s\n", p_path.c_str());
		return false;
	}
	struct stat st;
	fstat(fd, &st);
	const uint64_t expect = DATA_OFFSET + uint64_t(SRC_W) * SRC_H * 2;
	if (uint64_t(st.st_size) != expect) {
		std::fprintf(stderr, "mars dem: size %lld, expected %llu\n", (long long)st.st_size,
				(unsigned long long)expect);
		::close(fd);
		return false;
	}
	map_size_ = size_t(st.st_size);
	map_ = mmap(nullptr, map_size_, PROT_READ, MAP_SHARED, fd, 0);
	::close(fd);
	if (map_ == MAP_FAILED) {
		map_ = nullptr;
		return false;
	}
	raw_ = reinterpret_cast<const int16_t *>(static_cast<const uint8_t *>(map_) + DATA_OFFSET);

	mips_.resize(size_t(p_mip_count));
	mips_[0] = { SRC_W, SRC_H, PIXEL_DEG, {} };
	for (size_t k = 1; k < mips_.size(); ++k) {
		build_mip(k);
		std::printf("  mip %zu: %lld x %lld\n", k, (long long)mips_[k].w, (long long)mips_[k].h);
		std::fflush(stdout);
	}
	north_cap_ = row_mean(0);
	south_cap_ = row_mean(SRC_H - 1);
	std::printf("  pole caps: north %.1f m, south %.1f m\n", north_cap_, south_cap_);
	return true;
}

double MarsDem::value(const Mip &p_mip, int64_t p_x, int64_t p_y) const {
	if (p_mip.px.empty()) {
		const int16_t v = raw_[p_y * SRC_W + p_x];
		return v == NODATA ? NaN : double(v);
	}
	return double(p_mip.px[size_t(p_y * p_mip.w + p_x)]);
}

void MarsDem::build_mip(size_t p_level) {
	const Mip &src = mips_[p_level - 1];
	Mip &dst = mips_[p_level];
	dst.w = (src.w + 1) / 2;
	dst.h = (src.h + 1) / 2;
	dst.deg = src.deg * 2.0;
	dst.px.assign(size_t(dst.w * dst.h), 0.0f);
	parallel_for(size_t(dst.h), [&](size_t p_y) {
		const int64_t y = int64_t(p_y);
		for (int64_t x = 0; x < dst.w; ++x) {
			double sum = 0.0;
			int n = 0;
			for (int64_t sy = 2 * y; sy <= 2 * y + 1 && sy < src.h; ++sy) {
				for (int64_t sx = 2 * x; sx <= 2 * x + 1 && sx < src.w; ++sx) {
					const double v = value(src, sx, sy);
					if (!std::isnan(v)) {
						sum += v;
						++n;
					}
				}
			}
			dst.px[size_t(y * dst.w + x)] = n ? float(sum / n) : std::numeric_limits<float>::quiet_NaN();
		}
	});
}

double MarsDem::row_mean(int64_t p_y) const {
	double sum = 0.0;
	int64_t n = 0;
	for (int64_t x = 0; x < SRC_W; ++x) {
		const double v = value(mips_[0], x, p_y);
		if (!std::isnan(v)) {
			sum += v;
			++n;
		}
	}
	return n ? sum / double(n) : 0.0;
}

double MarsDem::sample(int p_mip, double p_lat, double p_lon) const {
	const Mip &mip = mips_[size_t(p_mip)];
	const double period = 360.0 / mip.deg;
	double fx = (p_lon + 180.0) / mip.deg - 0.5;
	fx = std::fmod(fx, period);
	if (fx < 0.0) {
		fx += period;
	}
	int64_t x0, x1;
	double tx;
	if (fx <= double(mip.w - 1)) {
		x0 = int64_t(std::floor(fx));
		x1 = std::min(x0 + 1, mip.w - 1);
		tx = fx - double(x0);
	} else {
		// Across the antimeridian gap: last column to the first.
		x0 = mip.w - 1;
		x1 = 0;
		tx = (fx - double(mip.w - 1)) / (period - double(mip.w - 1));
	}
	double fy = (90.0 - p_lat) / mip.deg - 0.5;
	fy = std::clamp(fy, 0.0, double(mip.h - 1));
	const int64_t y0 = int64_t(std::floor(fy));
	const int64_t y1 = std::min(y0 + 1, mip.h - 1);
	const double ty = fy - double(y0);

	const double w[4] = { (1 - tx) * (1 - ty), tx * (1 - ty), (1 - tx) * ty, tx * ty };
	const double v[4] = { value(mip, x0, y0), value(mip, x1, y0), value(mip, x0, y1), value(mip, x1, y1) };
	double sum = 0.0;
	double wsum = 0.0;
	for (int k = 0; k < 4; ++k) {
		if (!std::isnan(v[k])) {
			sum += w[k] * v[k];
			wsum += w[k];
		}
	}
	return wsum > 1e-12 ? sum / wsum : NaN;
}

} // namespace regolith::bake
