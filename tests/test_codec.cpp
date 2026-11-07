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

std::vector<int32_t> terrain_like(int p_side, uint64_t p_seed) {
	std::mt19937_64 rng(p_seed);
	std::uniform_int_distribution<int32_t> noise(-40, 40);
	std::vector<int32_t> v(size_t(p_side) * p_side);
	for (int j = 0; j < p_side; ++j) {
		for (int i = 0; i < p_side; ++i) {
			const double h = 300000.0 * std::sin(i * 0.031) * std::cos(j * 0.047) - 1500000.0;
			v[size_t(j) * p_side + i] = int32_t(h) + noise(rng);
		}
	}
	v[0] = INT32_MIN / 2;
	v[1] = INT32_MAX / 2;
	return v;
}

} // namespace

TEST_CASE(crc32_matches_zlib) {
	const std::string s = "123456789";
	CHECK(crc32(reinterpret_cast<const uint8_t *>(s.data()), s.size()) == 0xCBF43926u);
}

TEST_CASE(codecs_round_trip) {
	const int side = 213;
	const std::vector<int32_t> v = terrain_like(side, 5);
	for (Codec codec : { CODEC_ROW_DELTA, CODEC_MED, CODEC_MED_NO_PLANES }) {
		const std::vector<uint8_t> blob = encode_tile(v.data(), side, codec, 3);
		std::vector<int32_t> out(v.size());
		CHECK(!blob.empty());
		CHECK(decode_tile(blob.data(), blob.size(), side, codec, out.data()));
		CHECK(out == v);
	}
}

TEST_CASE(pack_write_read) {
	const char *path = "regolith_test_pack.rgp";
	const int side = 13;
	PackHeader h;
	h.body = "testbody";
	h.intervals = 8;
	h.border = 2;
	h.step = 1.0 / 16.0;
	h.r_ref = 1234.5;
	h.codec = CODEC_MED;
	h.max_level = 1;
	{
		PackWriter w;
		CHECK(w.open(path, h, "attribution text"));
		for (uint32_t y = 0; y < 2; ++y) {
			const std::vector<int32_t> v = terrain_like(side, y);
			CHECK(w.add(TileKey::make(3, 1, 1, y), encode_tile(v.data(), side, CODEC_MED, 3), -1.0f, float(y)));
		}
		const std::vector<int32_t> v = terrain_like(side, 9);
		CHECK(!w.add(TileKey::make(0, 0, 0, 0), encode_tile(v.data(), side, CODEC_MED, 3), 0, 0));
		CHECK(w.finish());
	}
	PackReader r;
	CHECK(r.open(path));
	CHECK(r.header().body == "testbody");
	CHECK(r.header().step == 1.0 / 16.0);
	CHECK(r.header().r_ref == 1234.5);
	CHECK(r.header().tile_count == 2);
	CHECK(r.attribution() == "attribution text");
	const PackEntry *e = r.find(TileKey::make(3, 1, 1, 1));
	CHECK(e != nullptr);
	CHECK(r.find(TileKey::make(3, 1, 0, 1)) == nullptr);
	if (e) {
		std::vector<uint8_t> blob;
		std::vector<int32_t> out(size_t(side) * side);
		CHECK(r.read_blob(*e, blob));
		CHECK(decode_tile(blob.data(), blob.size(), side, CODEC_MED, out.data()));
		CHECK(out == terrain_like(side, 1));
		CHECK(e->max == 1.0f);
	}
	std::remove(path);
}
