// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <thread>
#include <vector>

namespace regolith::bake {

// Runs p_fn(i) for every i in [0, p_count) on all cores.
template <typename F>
void parallel_for(size_t p_count, F &&p_fn) {
	const unsigned threads = std::max(1u, std::thread::hardware_concurrency());
	std::atomic<size_t> next{ 0 };
	std::vector<std::thread> pool;
	pool.reserve(threads);
	for (unsigned t = 0; t < threads; ++t) {
		pool.emplace_back([&] {
			for (size_t i = next++; i < p_count; i = next++) {
				p_fn(i);
			}
		});
	}
	for (std::thread &th : pool) {
		th.join();
	}
}

} // namespace regolith::bake
