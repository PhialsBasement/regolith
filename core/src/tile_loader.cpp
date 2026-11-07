// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "regolith/tile_loader.hpp"

#include <algorithm>
#include <chrono>

#ifdef __linux__
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

#include "regolith/cube.hpp"
#include "regolith/horizon.hpp"

namespace regolith {

HeightTilePtr TileCache::get(TileKey p_key) {
	std::lock_guard<std::mutex> guard(mutex_);
	auto it = map_.find(p_key.bits);
	if (it == map_.end()) {
		return nullptr;
	}
	lru_.splice(lru_.begin(), lru_, it->second.lru);
	return it->second.tile;
}

bool TileCache::contains(TileKey p_key) const {
	std::lock_guard<std::mutex> guard(mutex_);
	return map_.count(p_key.bits) != 0;
}

void TileCache::put(const HeightTilePtr &p_tile) {
	std::lock_guard<std::mutex> guard(mutex_);
	auto it = map_.find(p_tile->key.bits);
	if (it != map_.end()) {
		it->second.tile = p_tile;
		lru_.splice(lru_.begin(), lru_, it->second.lru);
		return;
	}
	lru_.push_front(p_tile->key.bits);
	map_[p_tile->key.bits] = { p_tile, lru_.begin() };
	// Evict from the cold end, skipping pinned levels.
	auto victim = lru_.end();
	while (map_.size() > capacity_ && victim != lru_.begin()) {
		--victim;
		if (TileKey{ *victim }.level() <= pinned_level_) {
			continue;
		}
		map_.erase(*victim);
		victim = lru_.erase(victim);
	}
}

size_t TileCache::size() const {
	std::lock_guard<std::mutex> guard(mutex_);
	return map_.size();
}

TileLoader::TileLoader(const PackTileSource &p_source, TileCache &p_cache, int p_threads, const Synth *p_synth) :
		synth_(p_synth), source_(p_source), cache_(p_cache) {
	for (int t = 0; t < p_threads; ++t) {
		threads_.emplace_back([this] { worker(); });
	}
}

TileLoader::~TileLoader() {
	{
		std::lock_guard<std::mutex> guard(mutex_);
		stop_ = true;
	}
	wake_.notify_all();
	for (std::thread &t : threads_) {
		t.join();
	}
}

void TileLoader::request(TileKey p_key, float p_priority) {
	// cached with its horizon: done; cached without one (made
	// off the workers) still goes to a worker for the horizon
	if (horizon_ready(p_key)) {
		std::lock_guard<std::mutex> guard(mutex_);
		finished_.push_back(p_key);
		return;
	}
	{
		std::lock_guard<std::mutex> guard(mutex_);
		if (in_flight_.count(p_key.bits)) {
			return;
		}
		Request &r = queue_[p_key.bits];
		r.priority = p_priority;
		r.epoch = epoch_;
	}
	wake_.notify_one();
}

void TileLoader::end_frame() {
	std::lock_guard<std::mutex> guard(mutex_);
	for (auto it = queue_.begin(); it != queue_.end();) {
		it = it->second.epoch < epoch_ ? queue_.erase(it) : std::next(it);
	}
	++epoch_;
}

std::vector<TileKey> TileLoader::take_finished() {
	std::lock_guard<std::mutex> guard(mutex_);
	std::vector<TileKey> out;
	out.swap(finished_);
	return out;
}

size_t TileLoader::pending() const {
	std::lock_guard<std::mutex> guard(mutex_);
	return queue_.size() + in_flight_.size();
}

HeightTilePtr TileLoader::produce(TileKey p_key) {
	if (HeightTilePtr t = cache_.get(p_key)) {
		return t;
	}
	const auto t0 = std::chrono::steady_clock::now();
	HeightTilePtr tile = make(p_key);
	if (tile) {
		cache_.put(tile);
	}
	const auto us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();
	make_us_[p_key.level()] += uint64_t(us);
	++make_count_[p_key.level()];
	return tile;
}

void TileLoader::level_timing(int p_level, double &r_ms, uint64_t &r_count) const {
	r_count = make_count_[p_level];
	r_ms = r_count ? double(make_us_[p_level]) / double(r_count) / 1000.0 : 0.0;
}

HeightTilePtr TileLoader::make(TileKey p_key) {
	const TileLayout lay = source_.layout();
	const bool synth = synth_ && synth_->enabled();
	const int level = p_key.level();
	auto tile = std::make_shared<HeightTile>();
	if (level <= source_.max_level()) {
		if (!source_.load(p_key, *tile)) {
			return nullptr;
		}
		if (synth) {
			if (level == synth_->config().detail_level) {
				synth_->detail(*tile, lay);
			}
			synth_->borehole(*tile, lay);
		}
	} else if (synth && level == synth_->config().relief_level) {
		relief_level(p_key, *tile);
	} else {
		const HeightTilePtr parent = produce(p_key.parent());
		if (!parent) {
			return nullptr;
		}
		const int quadrant = int(p_key.x() & 1u) | int((p_key.y() & 1u) << 1);
		subdivide(*parent, lay, quadrant, *tile);
		if (synth && level == synth_->config().rough_level) {
			synth_->roughness(*tile, lay);
		}
	}
	if (synth) {
		stamp_craters(p_key, *tile, 0);
		stamp_craters(p_key, *tile, 1);
	}
	if (synth_) {
		synth_->geology(p_key, lay, tile->geology);
	}
	return tile;
}

// Terrain shadows, a stage after the heights: the near field
// over a window reaching into the neighbours, the far field from
// the parent's page. Neighbours come from the cache (or their
// deepest cached ancestor) so one tile never makes another.
std::shared_ptr<const std::vector<uint8_t>> TileLoader::horizon_ready(TileKey p_key) {
	const HeightTilePtr tile = cache_.get(p_key);
	if (!tile) {
		return nullptr;
	}
	std::lock_guard<std::mutex> guard(horizon_mutex_);
	return tile->horizon;
}

std::shared_ptr<const std::vector<uint8_t>> TileLoader::horizon(TileKey p_key) {
	const HeightTilePtr tile = produce(p_key);
	if (!tile) {
		return nullptr;
	}
	{
		std::lock_guard<std::mutex> guard(horizon_mutex_);
		if (tile->horizon) {
			return tile->horizon;
		}
	}
	const std::shared_ptr<const std::vector<uint8_t>> parent = p_key.level() > 0 ? horizon(p_key.parent()) : nullptr;
	const TileLayout lay = source_.layout();
	const int n = lay.intervals, b = lay.border;
	const int lo = -HORIZON_PAD, hi = n + HORIZON_PAD, w = hi - lo + 1;
	const int level = p_key.level();
	const int64_t m = int64_t(n) << level;
	std::unordered_map<uint64_t, HeightTilePtr> near;
	auto cached = [&](TileKey p_k) {
		auto it = near.find(p_k.bits);
		if (it == near.end()) {
			it = near.emplace(p_k.bits, p_k == p_key ? tile : cache_.get(p_k)).first;
		}
		return it->second;
	};
	// Height of canonical lattice sample (face, gi, gj) at level
	// L, from the deepest cached tile holding it.
	auto height = [&](const FaceSample &p_c) -> double {
		for (int a = level; a >= 0; --a) {
			const double s = double(int64_t(1) << (level - a));
			const double gi = double(p_c.gi) / s, gj = double(p_c.gj) / s;
			const uint32_t tiles = 1u << a;
			const uint32_t tx = std::min(uint32_t(gi / n), tiles - 1), ty = std::min(uint32_t(gj / n), tiles - 1);
			const HeightTilePtr t = cached(TileKey::make(p_c.face, a, tx, ty));
			if (!t) {
				continue;
			}
			const double li = gi - double(tx) * n, lj = gj - double(ty) * n;
			const int i0 = std::min(int(li), n - 1), j0 = std::min(int(lj), n - 1);
			const double fx = li - i0, fy = lj - j0;
			return (t->at(lay, i0, j0) * (1 - fx) + t->at(lay, i0 + 1, j0) * fx) * (1 - fy) +
					(t->at(lay, i0, j0 + 1) * (1 - fx) + t->at(lay, i0 + 1, j0 + 1) * fx) * fy;
		}
		return 0.0;
	};
	std::vector<DVec3> pos(size_t(w) * w);
	for (int j = lo; j <= hi; ++j) {
		for (int i = lo; i <= hi; ++i) {
			const IVec3 q = fold_lattice(p_key.face(), int64_t(p_key.x()) * n + i, int64_t(p_key.y()) * n + j, m);
			const bool own = i >= -b && i <= n + b && j >= -b && j <= n + b;
			const double h = own ? double(tile->at(lay, i, j)) : height(canonical_sample(q, m));
			pos[size_t(j - lo) * w + size_t(i - lo)] = lattice_dir(q, m) * (source_.r_ref() + h);
		}
	}
	auto page = std::make_shared<std::vector<uint8_t>>();
	compute_horizon(pos, p_key, lay, parent.get(), *page);
	std::lock_guard<std::mutex> guard(horizon_mutex_);
	if (!tile->horizon) {
		tile->horizon = page;
	}
	return tile->horizon;
}

// Relief before thermal, cached apart: thermal of one tile needs
// its neighbours' too.
HeightTilePtr TileLoader::pre_thermal(TileKey p_key) {
	{
		std::lock_guard<std::mutex> guard(pre_mutex_);
		auto it = pre_.find(p_key.bits);
		if (it != pre_.end()) {
			return it->second;
		}
	}
	const HeightTilePtr parent = produce(p_key.parent());
	if (!parent) {
		return nullptr;
	}
	auto tile = std::make_shared<HeightTile>();
	const int quadrant = int(p_key.x() & 1u) | int((p_key.y() & 1u) << 1);
	subdivide(*parent, source_.layout(), quadrant, *tile);
	synth_->relief(*tile, source_.layout());
	std::lock_guard<std::mutex> guard(pre_mutex_);
	if (pre_.emplace(p_key.bits, tile).second) {
		pre_lru_.push_back(p_key.bits);
		while (pre_lru_.size() > 512) {
			pre_.erase(pre_lru_.front());
			pre_lru_.pop_front();
		}
	}
	return tile;
}

// terra_fine runs on a padded grid so its thermal steps never see
// a region edge; here the pad is gathered from the neighbouring
// tiles' pre-thermal relief, across face edges by the fold.
void TileLoader::relief_level(TileKey p_key, HeightTile &r_tile) {
	const TileLayout lay = source_.layout();
	const int n_int = lay.intervals;
	const int pad = lay.border + synth_->config().relief_margin;
	const int side = n_int + 1 + 2 * pad;
	const int level = p_key.level();
	const int64_t m = int64_t(n_int) << level;
	const uint32_t tiles = 1u << level;
	std::vector<float> h(size_t(side) * side), rep(size_t(side) * side);
	for (int j = -pad; j <= n_int + pad; ++j) {
		for (int i = -pad; i <= n_int + pad; ++i) {
			const FaceSample c = canonical_sample(fold_lattice(p_key.face(), int64_t(p_key.x()) * n_int + i,
														 int64_t(p_key.y()) * n_int + j, m),
					m);
			const uint32_t tx = std::min(uint32_t(c.gi / n_int), tiles - 1);
			const uint32_t ty = std::min(uint32_t(c.gj / n_int), tiles - 1);
			const HeightTilePtr src = pre_thermal(TileKey::make(c.face, level, tx, ty));
			const size_t k = size_t(j + pad) * side + size_t(i + pad);
			if (!src) {
				h[k] = 0.0f;
				rep[k] = 1.0f;
				continue;
			}
			const int li = int(c.gi - int64_t(tx) * n_int), lj = int(c.gj - int64_t(ty) * n_int);
			h[k] = src->at(lay, li, lj);
			rep[k] = src->repose[size_t(lay.index(li, lj))];
		}
	}
	synth_->thermal(h, rep, side, synth_->spacing(level));
	r_tile.key = p_key;
	r_tile.rho.resize(size_t(lay.count()));
	const int off = pad - lay.border;
	for (int j = -lay.border; j <= n_int + lay.border; ++j) {
		for (int i = -lay.border; i <= n_int + lay.border; ++i) {
			r_tile.rho[size_t(lay.index(i, j))] = h[size_t(j + lay.border + off) * side + size_t(i + lay.border + off)];
		}
	}
	r_tile.update_bounds(lay);
}

// A crater band appears at the first level resolving its biggest
// craters, so they show from afar; finer levels swap the shape
// inherited from the parent for the exact one, rims kept sharp.
void TileLoader::stamp_craters(TileKey p_key, HeightTile &r_tile, int p_band) {
	const int level = p_key.level();
	const int first = synth_->crater_first_level(p_band);
	if (level < first || level > synth_->crater_last_level(p_band)) {
		return;
	}
	const TileLayout lay = source_.layout();
	std::vector<float> add;
	if (!synth_->crater_values(p_key, lay, p_band, add)) {
		return;
	}
	if (level > source_.max_level() && level > first) {
		HeightTile parent_field, inherited;
		parent_field.key = p_key.parent();
		synth_->crater_values(parent_field.key, lay, p_band, parent_field.rho);
		subdivide(parent_field, lay, int(p_key.x() & 1u) | int((p_key.y() & 1u) << 1), inherited);
		for (size_t k = 0; k < add.size(); ++k) {
			add[k] -= inherited.rho[k];
		}
	}
	for (size_t k = 0; k < add.size(); ++k) {
		r_tile.rho[k] += add[k];
	}
	r_tile.update_bounds(lay);
}

void TileLoader::worker() {
#ifdef __linux__
	// Streaming must never take cores from the game's own threads
	// (main, render, Jolt's jobs): run at the lowest priority.
	setpriority(PRIO_PROCESS, pid_t(syscall(SYS_gettid)), 19);
#endif
	for (;;) {
		uint64_t key = 0;
		{
			std::unique_lock<std::mutex> lock(mutex_);
			wake_.wait(lock, [this] { return stop_ || !queue_.empty(); });
			if (stop_) {
				return;
			}
			auto best = queue_.begin();
			for (auto it = queue_.begin(); it != queue_.end(); ++it) {
				if (it->second.priority > best->second.priority) {
					best = it;
				}
			}
			key = best->first;
			queue_.erase(best);
			in_flight_.insert(key);
		}
		horizon(TileKey{ key });
		{
			std::lock_guard<std::mutex> guard(mutex_);
			in_flight_.erase(key);
			finished_.push_back(TileKey{ key });
		}
	}
}

} // namespace regolith
