// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <cmath>
#include <cstdio>
#include <random>
#include <string>

#include "regolith/codec.hpp"
#include "regolith/pack.hpp"
#include "test.hpp"

using namespace regolith;

namespace {

constexpr int ZSTD_LEVEL = 3;

// The test pack: two level-1 tiles of face 3, 13 x 13 samples
// (8 intervals, border 2).
const char *const PACK_PATH = "regolith_test_pack.rgp";
constexpr int PACK_SIDE = 13;
const char *const PACK_ATTRIBUTION = "attribution text";

// Smooth swells around -1.5e6 steps with a little noise, plus
// the extremes a codec must survive.
std::vector<int32_t> terrain_like(int p_side, uint64_t p_seed) {
	std::mt19937_64 rng(p_seed);
	std::uniform_int_distribution<int32_t> noise(-40, 40);
	std::vector<int32_t> samples(size_t(p_side) * p_side);
	for (int j = 0; j < p_side; ++j) {
		for (int i = 0; i < p_side; ++i) {
			const double swell = 300000.0 * std::sin(i * 0.031) * std::cos(j * 0.047) - 1500000.0;
			samples[size_t(j) * p_side + i] = int32_t(swell) + noise(rng);
		}
	}
	samples[0] = INT32_MIN / 2;
	samples[1] = INT32_MAX / 2;
	return samples;
}

PackHeader test_header() {
	PackHeader header;
	header.body = "testbody";
	header.intervals = 8;
	header.border = 2;
	header.step = 1.0 / 16.0;
	header.r_ref = 1234.5;
	header.codec = CODEC_MED;
	header.max_level = 1;
	return header;
}

std::vector<uint8_t> encoded_tile(uint64_t p_seed) {
	const std::vector<int32_t> samples = terrain_like(PACK_SIDE, p_seed);
	return encode_tile(samples.data(), PACK_SIDE, CODEC_MED, ZSTD_LEVEL);
}

// Tile (3, 1, 1, y) holds terrain seeded with y, and its max
// bound is y.
bool write_test_pack() {
	PackWriter writer;
	bool ok = writer.open(PACK_PATH, test_header(), PACK_ATTRIBUTION);
	for (uint32_t y = 0; y < 2; ++y) {
		ok = ok && writer.add(TileKey::make(3, 1, 1, y), encoded_tile(y), -1.0f, float(y));
	}
	return ok && writer.finish();
}

} // namespace

TEST_CASE(crc32_matches_zlib) {
	// the standard check value of CRC-32/ISO-HDLC
	const std::string check_input = "123456789";
	CHECK(crc32(reinterpret_cast<const uint8_t *>(check_input.data()), check_input.size()) == 0xCBF43926u);
}

TEST_CASE(codecs_round_trip) {
	const int side = 213;
	const std::vector<int32_t> samples = terrain_like(side, 5);
	for (Codec codec : { CODEC_ROW_DELTA, CODEC_MED, CODEC_MED_NO_PLANES }) {
		const std::vector<uint8_t> blob = encode_tile(samples.data(), side, codec, ZSTD_LEVEL);
		std::vector<int32_t> decoded(samples.size());
		CHECK(!blob.empty());
		CHECK(decode_tile(blob.data(), blob.size(), side, codec, decoded.data()));
		CHECK(decoded == samples);
	}
}

TEST_CASE(pack_keeps_header_and_attribution) {
	CHECK(write_test_pack());
	PackReader reader;
	CHECK(reader.open(PACK_PATH));
	CHECK(reader.header().body == "testbody");
	CHECK(reader.header().step == 1.0 / 16.0);
	CHECK(reader.header().r_ref == 1234.5);
	CHECK(reader.header().tile_count == 2);
	CHECK(reader.attribution() == PACK_ATTRIBUTION);
	std::remove(PACK_PATH);
}

TEST_CASE(pack_writer_refuses_out_of_order_tiles) {
	PackWriter writer;
	CHECK(writer.open(PACK_PATH, test_header(), PACK_ATTRIBUTION));
	CHECK(writer.add(TileKey::make(3, 1, 1, 0), encoded_tile(0), -1.0f, 0.0f));
	// (0, 0, 0, 0) sorts before the tile already written
	CHECK(!writer.add(TileKey::make(0, 0, 0, 0), encoded_tile(9), 0, 0));
	CHECK(writer.finish());
	std::remove(PACK_PATH);
}

TEST_CASE(pack_finds_tiles_by_key) {
	CHECK(write_test_pack());
	PackReader reader;
	CHECK(reader.open(PACK_PATH));
	CHECK(reader.find(TileKey::make(3, 1, 1, 1)) != nullptr);
	CHECK(reader.find(TileKey::make(3, 1, 0, 1)) == nullptr);
	std::remove(PACK_PATH);
}

TEST_CASE(pack_tile_reads_back_as_written) {
	CHECK(write_test_pack());
	PackReader reader;
	CHECK(reader.open(PACK_PATH));
	const PackEntry *entry = reader.find(TileKey::make(3, 1, 1, 1));
	CHECK(entry != nullptr);
	if (!entry) {
		std::remove(PACK_PATH);
		return;
	}
	std::vector<uint8_t> blob;
	std::vector<int32_t> decoded(size_t(PACK_SIDE) * PACK_SIDE);
	CHECK(reader.read_blob(*entry, blob));
	CHECK(decode_tile(blob.data(), blob.size(), PACK_SIDE, CODEC_MED, decoded.data()));
	CHECK(decoded == terrain_like(PACK_SIDE, 1));
	CHECK(entry->max == 1.0f);
	std::remove(PACK_PATH);
}
