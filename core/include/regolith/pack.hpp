// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "regolith/tile_key.hpp"

namespace regolith {

// See docs/rgp1.md for the byte layout.
enum PackLayer : uint32_t {
	PACK_LAYER_HEIGHT = 0,
	PACK_LAYER_AREOID = 1,
};

struct PackHeader {
	uint32_t version = 1;
	uint32_t layer = PACK_LAYER_HEIGHT;
	std::string body;
	uint32_t intervals = 0;
	uint32_t border = 0;
	uint32_t projection = 1;
	uint32_t sample_format = 1;
	double step = 0.0;
	double r_ref = 0.0;
	uint32_t codec = 0;
	uint32_t codec_level = 0;
	uint32_t min_level = 0;
	uint32_t max_level = 0;
	uint64_t tile_count = 0;
	uint64_t index_offset = 0;
	uint64_t attribution_offset = 0;
	uint32_t attribution_size = 0;
};

struct PackEntry {
	TileKey key;
	uint64_t offset = 0;
	uint32_t size = 0;
	uint32_t crc = 0;
	float min = 0.0f;
	float max = 0.0f;
};

constexpr size_t PACK_HEADER_SIZE = 256;
constexpr size_t PACK_ENTRY_SIZE = 32;

// Blobs must be added in ascending key order.
class PackWriter {
public:
	~PackWriter();
	bool open(const std::string &p_path, const PackHeader &p_header, const std::string &p_attribution);
	bool add(TileKey p_key, const std::vector<uint8_t> &p_blob, float p_min, float p_max);
	bool finish();
	uint64_t bytes_written() const { return offset_; }

private:
	FILE *file_ = nullptr;
	PackHeader header_;
	std::vector<PackEntry> entries_;
	uint64_t offset_ = 0;
};

// Shares one FILE*, so use one reader per thread.
class PackReader {
public:
	~PackReader();
	bool open(const std::string &p_path);
	const PackHeader &header() const { return header_; }
	const std::vector<PackEntry> &entries() const { return entries_; }
	const std::string &attribution() const { return attribution_; }
	const PackEntry *find(TileKey p_key) const;
	bool read_blob(const PackEntry &p_entry, std::vector<uint8_t> &r_blob) const;
	// Safe to call from many threads at once.
	bool read_blob_concurrent(const PackEntry &p_entry, std::vector<uint8_t> &r_blob) const;

private:
	FILE *file_ = nullptr;
	int fd_ = -1;
	PackHeader header_;
	std::vector<PackEntry> entries_;
	std::string attribution_;
};

} // namespace regolith
