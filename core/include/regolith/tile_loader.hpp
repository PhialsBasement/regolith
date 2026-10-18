#pragma once

#include <condition_variable>
#include <cstdint>
#include <list>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "regolith/height_tile.hpp"

namespace regolith {

// Thread-safe LRU of decoded tiles. Levels at or below
// p_pinned_level are never evicted (the "never a hole" set).
class TileCache {
public:
	TileCache(size_t p_capacity, int p_pinned_level) :
			capacity_(p_capacity), pinned_level_(p_pinned_level) {}
	HeightTilePtr get(TileKey p_key);
	bool contains(TileKey p_key) const;
	void put(const HeightTilePtr &p_tile);
	size_t size() const;

private:
	struct Slot {
		HeightTilePtr tile;
		std::list<uint64_t>::iterator lru;
	};
	mutable std::mutex mutex_;
	std::unordered_map<uint64_t, Slot> map_;
	std::list<uint64_t> lru_; // front is most recent
	size_t capacity_;
	int pinned_level_;
};

// Produces tiles on worker threads: data levels from the pack,
// finer levels by subdividing their (recursively made) parent.
class TileLoader {
public:
	TileLoader(const PackTileSource &p_source, TileCache &p_cache, int p_threads);
	~TileLoader();

	// Higher priority runs sooner; asking again refreshes it.
	void request(TileKey p_key, float p_priority);
	// Drops requests nobody asked for since the last call.
	void end_frame();
	// Keys finished since the last call.
	std::vector<TileKey> take_finished();
	// Synchronous, on the calling thread.
	HeightTilePtr produce(TileKey p_key);
	size_t pending() const;

private:
	void worker();

	struct Request {
		float priority = 0.0f;
		uint64_t epoch = 0;
	};

	const PackTileSource &source_;
	TileCache &cache_;
	mutable std::mutex mutex_;
	std::condition_variable wake_;
	std::unordered_map<uint64_t, Request> queue_;
	std::unordered_set<uint64_t> in_flight_;
	std::vector<TileKey> finished_;
	uint64_t epoch_ = 1;
	bool stop_ = false;
	std::vector<std::thread> threads_;
};

} // namespace regolith
