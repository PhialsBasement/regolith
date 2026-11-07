// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "tile_bake.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>

#include "parallel.hpp"
#include "regolith/codec.hpp"

namespace regolith::bake {

namespace {

using Clock = std::chrono::steady_clock;

double seconds_since(Clock::time_point p_start) {
	return std::chrono::duration<double>(Clock::now() - p_start).count();
}

std::vector<TileKey> keys_in_order(int p_min_level, int p_max_level) {
	std::vector<TileKey> keys;
	for (int face = 0; face < FACE_COUNT; ++face) {
		for (int level = p_min_level; level <= p_max_level; ++level) {
			const uint32_t n = 1u << level;
			for (uint32_t x = 0; x < n; ++x) {
				for (uint32_t y = 0; y < n; ++y) {
					keys.push_back(TileKey::make(face, level, x, y));
				}
			}
		}
	}
	return keys;
}

} // namespace

void compute_tile(const LevelSampler &p_sampler, TileKey p_key, const TileLayout &p_layout, std::vector<double> &r_rho) {
	const int n = p_layout.intervals;
	const int b = p_layout.border;
	const int64_t m = int64_t(n) << p_key.level();
	r_rho.resize(size_t(p_layout.count()));
	for (int j = -b; j <= n + b; ++j) {
		for (int i = -b; i <= n + b; ++i) {
			const int64_t gi = int64_t(p_key.x()) * n + i;
			const int64_t gj = int64_t(p_key.y()) * n + j;
			const IVec3 p = fold_lattice(p_key.face(), gi, gj, m);
			const FaceSample c = canonical_sample(p, m);
			r_rho[size_t(p_layout.index(i, j))] = p_sampler.rho(p_key.level(), c, m, p);
		}
	}
}

QuantizedTile quantize_tile(const std::vector<double> &p_rho, const TileLayout &p_layout, double p_step) {
	QuantizedTile q;
	q.values.resize(p_rho.size());
	double lo = std::numeric_limits<double>::infinity();
	double hi = -lo;
	for (int j = -p_layout.border; j <= p_layout.intervals + p_layout.border; ++j) {
		for (int i = -p_layout.border; i <= p_layout.intervals + p_layout.border; ++i) {
			const size_t k = size_t(p_layout.index(i, j));
			double rho = p_rho[k];
			if (std::isnan(rho)) {
				++q.invalid;
				rho = 0.0;
			}
			q.values[k] = int32_t(std::llround(rho / p_step));
			if (i >= 0 && j >= 0 && i <= p_layout.intervals && j <= p_layout.intervals) {
				const double decoded = double(q.values[k]) * p_step;
				lo = std::min(lo, decoded);
				hi = std::max(hi, decoded);
			}
		}
	}
	q.min = float(lo);
	q.max = float(hi);
	return q;
}

bool bake_pack(const LevelSampler &p_sampler, const BakeConfig &p_config, BakeStats &r_stats) {
	const Clock::time_point start = Clock::now();
	PackHeader header = p_config.header;
	header.intervals = uint32_t(p_config.layout.intervals);
	header.border = uint32_t(p_config.layout.border);
	header.min_level = uint32_t(p_config.min_level);
	header.max_level = uint32_t(p_config.max_level);
	header.codec_level = uint32_t(p_config.zstd_level);

	PackWriter writer;
	if (!writer.open(p_config.path, header, p_config.attribution)) {
		std::fprintf(stderr, "cannot write %s\n", p_config.path.c_str());
		return false;
	}
	const std::vector<TileKey> keys = keys_in_order(p_config.min_level, p_config.max_level);
	const size_t chunk = 2048;
	std::vector<std::vector<uint8_t>> blobs(chunk);
	std::vector<QuantizedTile> tiles(chunk);
	std::atomic<uint64_t> invalid{ 0 };
	for (size_t first = 0; first < keys.size(); first += chunk) {
		const size_t count = std::min(chunk, keys.size() - first);
		parallel_for(count, [&](size_t p_k) {
			std::vector<double> rho;
			compute_tile(p_sampler, keys[first + p_k], p_config.layout, rho);
			tiles[p_k] = quantize_tile(rho, p_config.layout, header.step);
			blobs[p_k] = encode_tile(tiles[p_k].values.data(), p_config.layout.side(), Codec(header.codec),
					p_config.zstd_level);
			invalid += uint64_t(tiles[p_k].invalid);
		});
		for (size_t k = 0; k < count; ++k) {
			if (blobs[k].empty() || !writer.add(keys[first + k], blobs[k], tiles[k].min, tiles[k].max)) {
				std::fprintf(stderr, "failed to add tile %zu\n", first + k);
				return false;
			}
		}
		const double done = double(first + count) / double(keys.size());
		const double elapsed = seconds_since(start);
		std::printf("  %s: %zu/%zu tiles, %.2f GB, %.0f s, eta %.0f s\n", header.body.c_str(), first + count,
				keys.size(), double(writer.bytes_written()) / 1e9, elapsed, elapsed / done - elapsed);
		std::fflush(stdout);
	}
	if (!writer.finish()) {
		return false;
	}
	r_stats.tiles = keys.size();
	r_stats.bytes = writer.bytes_written();
	r_stats.invalid_samples = invalid;
	r_stats.seconds = seconds_since(start);
	return true;
}

void codec_trial(const LevelSampler &p_sampler, const TileLayout &p_layout, const std::vector<TileKey> &p_keys) {
	std::vector<std::vector<double>> rho(p_keys.size());
	parallel_for(p_keys.size(), [&](size_t p_k) { compute_tile(p_sampler, p_keys[p_k], p_layout, rho[p_k]); });
	const size_t samples = p_keys.size() * size_t(p_layout.count());
	std::printf("codec trial: %zu tiles, %zu samples\n", p_keys.size(), samples);
	std::printf("  %-8s %-6s %-5s %10s %9s %10s %10s\n", "step", "codec", "zstd", "bits/smp", "max err", "enc MB/s",
			"dec us/t");

	for (double step : { 1.0 / 64.0, 1.0 / 16.0, 1.0 / 4.0, 1.0 }) {
		std::vector<QuantizedTile> q(p_keys.size());
		std::atomic<int64_t> worst_nano{ 0 };
		parallel_for(p_keys.size(), [&](size_t p_k) {
			q[p_k] = quantize_tile(rho[p_k], p_layout, step);
			int64_t local = 0;
			for (size_t s = 0; s < rho[p_k].size(); ++s) {
				const double err = std::abs(double(q[p_k].values[s]) * step - rho[p_k][s]);
				local = std::max(local, int64_t(err * 1e9));
			}
			int64_t prev = worst_nano.load();
			while (local > prev && !worst_nano.compare_exchange_weak(prev, local)) {
			}
		});
		for (Codec codec : { CODEC_ROW_DELTA, CODEC_MED, CODEC_MED_NO_PLANES }) {
			for (int level : { 3, 9, 19 }) {
				std::vector<std::vector<uint8_t>> blobs(p_keys.size());
				const Clock::time_point t0 = Clock::now();
				parallel_for(p_keys.size(), [&](size_t p_k) {
					blobs[p_k] = encode_tile(q[p_k].values.data(), p_layout.side(), codec, level);
				});
				const double enc = seconds_since(t0);
				size_t bytes = 0;
				for (const std::vector<uint8_t> &b : blobs) {
					bytes += b.size();
				}
				std::vector<int32_t> out(size_t(p_layout.count()));
				const Clock::time_point t1 = Clock::now();
				bool ok = true;
				for (size_t k = 0; k < p_keys.size(); ++k) {
					ok = ok && decode_tile(blobs[k].data(), blobs[k].size(), p_layout.side(), codec, out.data());
					ok = ok && out == q[k].values;
				}
				const double dec_us = seconds_since(t1) * 1e6 / double(p_keys.size());
				std::printf("  %-8.4f %-6d %-5d %10.3f %9.4f %10.1f %10.1f%s\n", step, int(codec), level,
						8.0 * double(bytes) / double(samples), double(worst_nano.load()) * 1e-9,
						double(samples) * 4.0 / 1e6 / enc, dec_us, ok ? "" : "  DECODE MISMATCH");
				std::fflush(stdout);
			}
		}
	}
}

} // namespace regolith::bake
