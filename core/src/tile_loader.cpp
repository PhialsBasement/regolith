#include "regolith/tile_loader.hpp"

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

TileLoader::TileLoader(const PackTileSource &p_source, TileCache &p_cache, int p_threads) :
		source_(p_source), cache_(p_cache) {
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
	if (cache_.contains(p_key)) {
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
	auto tile = std::make_shared<HeightTile>();
	if (p_key.level() <= source_.max_level()) {
		if (!source_.load(p_key, *tile)) {
			return nullptr;
		}
	} else {
		const HeightTilePtr parent = produce(p_key.parent());
		if (!parent) {
			return nullptr;
		}
		const int quadrant = int(p_key.x() & 1u) | int((p_key.y() & 1u) << 1);
		subdivide(*parent, source_.layout(), quadrant, *tile);
	}
	cache_.put(tile);
	return tile;
}

void TileLoader::worker() {
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
		produce(TileKey{ key });
		{
			std::lock_guard<std::mutex> guard(mutex_);
			in_flight_.erase(key);
			finished_.push_back(TileKey{ key });
		}
	}
}

} // namespace regolith
