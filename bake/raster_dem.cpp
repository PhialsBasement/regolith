// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "raster_dem.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#include "parallel.hpp"

namespace regolith::bake {

namespace {

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

size_t pixel_bytes(RasterSpec::Type p_type) {
	return p_type == RasterSpec::INT16 ? 2 : 4;
}

} // namespace

RasterDem::~RasterDem() {
#ifdef _WIN32
	if (map_) {
		UnmapViewOfFile(map_);
	}
	if (mapping_) {
		CloseHandle(mapping_);
	}
	if (file_) {
		CloseHandle(file_);
	}
#else
	if (map_) {
		munmap(map_, map_size_);
	}
#endif
}

bool RasterDem::open(const RasterSpec &p_spec, int p_mip_count) {
	spec_ = p_spec;
	if (spec_.width <= 0 || spec_.height <= 0) {
		std::fprintf(stderr, "raster: bad size %lld x %lld\n", (long long)spec_.width, (long long)spec_.height);
		return false;
	}
	const uint64_t expect = spec_.offset + uint64_t(spec_.width) * uint64_t(spec_.height) * pixel_bytes(spec_.type);
#ifdef _WIN32
	file_ = CreateFileA(spec_.path.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr, OPEN_EXISTING, 0, nullptr);
	if (file_ == INVALID_HANDLE_VALUE) {
		file_ = nullptr;
		std::fprintf(stderr, "raster: cannot open %s\n", spec_.path.c_str());
		return false;
	}
	LARGE_INTEGER size;
	GetFileSizeEx(file_, &size);
	const uint64_t actual = uint64_t(size.QuadPart);
#else
	const int fd = ::open(spec_.path.c_str(), O_RDONLY);
	if (fd < 0) {
		std::fprintf(stderr, "raster: cannot open %s\n", spec_.path.c_str());
		return false;
	}
	struct stat st;
	fstat(fd, &st);
	const uint64_t actual = uint64_t(st.st_size);
#endif
	if (actual != expect) {
		std::fprintf(stderr, "raster: %s is %llu bytes, expected %llu\n", spec_.path.c_str(),
				(unsigned long long)actual, (unsigned long long)expect);
#ifndef _WIN32
		::close(fd);
#endif
		return false;
	}
	map_size_ = size_t(actual);
#ifdef _WIN32
	mapping_ = CreateFileMappingA(file_, nullptr, PAGE_READONLY, 0, 0, nullptr);
	map_ = mapping_ ? MapViewOfFile(mapping_, FILE_MAP_READ, 0, 0, 0) : nullptr;
	if (!map_) {
		std::fprintf(stderr, "raster: cannot map %s\n", spec_.path.c_str());
		return false;
	}
#else
	map_ = mmap(nullptr, map_size_, PROT_READ, MAP_SHARED, fd, 0);
	::close(fd);
	if (map_ == MAP_FAILED) {
		map_ = nullptr;
		return false;
	}
#endif
	data_ = static_cast<const uint8_t *>(map_) + spec_.offset;
	const double deg = spec_.pixel_deg > 0.0 ? spec_.pixel_deg : 360.0 / double(spec_.width);
	mips_.resize(size_t(std::max(p_mip_count, 1)));
	mips_[0] = { spec_.width, spec_.height, deg, {} };
	for (size_t k = 1; k < mips_.size(); ++k) {
		build_mip(k);
		std::printf("  mip %zu: %lld x %lld\n", k, (long long)mips_[k].w, (long long)mips_[k].h);
		std::fflush(stdout);
	}
	north_cap_ = row_mean(0);
	south_cap_ = row_mean(spec_.height - 1);
	std::printf("  pole caps: north %.1f m, south %.1f m\n", north_cap_, south_cap_);
	return true;
}

double RasterDem::raw_value(int64_t p_index) const {
	double v;
	if (spec_.type == RasterSpec::INT16) {
		int16_t x;
		std::memcpy(&x, data_ + size_t(p_index) * 2, 2);
		v = double(x);
	} else {
		float x;
		std::memcpy(&x, data_ + size_t(p_index) * 4, 4);
		v = double(x);
	}
	return v == spec_.nodata ? NaN : v;
}

double RasterDem::value(const Mip &p_mip, int64_t p_x, int64_t p_y) const {
	if (p_mip.px.empty()) {
		return raw_value(p_y * spec_.width + p_x);
	}
	return double(p_mip.px[size_t(p_y * p_mip.w + p_x)]);
}

void RasterDem::build_mip(size_t p_level) {
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

double RasterDem::row_mean(int64_t p_y) const {
	double sum = 0.0;
	int64_t n = 0;
	for (int64_t x = 0; x < spec_.width; ++x) {
		const double v = value(mips_[0], x, p_y);
		if (!std::isnan(v)) {
			sum += v;
			++n;
		}
	}
	return n ? sum / double(n) : 0.0;
}

double RasterDem::sample(int p_mip, double p_lat, double p_lon) const {
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
