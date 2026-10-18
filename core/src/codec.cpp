#include "regolith/codec.hpp"

#include <algorithm>
#include <array>

#include <zstd.h>

namespace regolith {

namespace {

int32_t predict(const int32_t *p_v, int p_side, int p_i, int p_j, Codec p_codec) {
	if (p_j == 0) {
		return p_i == 0 ? 0 : p_v[p_i - 1];
	}
	const int32_t b = p_v[(p_j - 1) * p_side + p_i];
	if (p_i == 0) {
		return b;
	}
	const int32_t a = p_v[p_j * p_side + p_i - 1];
	if (p_codec == CODEC_ROW_DELTA) {
		return a;
	}
	const int32_t c = p_v[(p_j - 1) * p_side + p_i - 1];
	if (c >= std::max(a, b)) {
		return std::min(a, b);
	}
	if (c <= std::min(a, b)) {
		return std::max(a, b);
	}
	return int32_t(int64_t(a) + b - c);
}

uint32_t zigzag(int32_t p_e) {
	return (uint32_t(p_e) << 1) ^ uint32_t(p_e >> 31);
}

int32_t unzigzag(uint32_t p_z) {
	return int32_t(p_z >> 1) ^ -int32_t(p_z & 1);
}

bool uses_planes(Codec p_codec) {
	return p_codec == CODEC_ROW_DELTA || p_codec == CODEC_MED;
}

const std::array<uint32_t, 256> &crc_table() {
	static const std::array<uint32_t, 256> table = [] {
		std::array<uint32_t, 256> t{};
		for (uint32_t n = 0; n < 256; ++n) {
			uint32_t c = n;
			for (int k = 0; k < 8; ++k) {
				c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
			}
			t[n] = c;
		}
		return t;
	}();
	return table;
}

} // namespace

std::vector<uint8_t> encode_tile(const int32_t *p_values, int p_side, Codec p_codec, int p_zstd_level) {
	const size_t n = size_t(p_side) * p_side;
	std::vector<uint8_t> raw(n * 4);
	for (int j = 0; j < p_side; ++j) {
		for (int i = 0; i < p_side; ++i) {
			const size_t k = size_t(j) * p_side + i;
			const int64_t e = int64_t(p_values[k]) - predict(p_values, p_side, i, j, p_codec);
			const uint32_t z = zigzag(int32_t(e));
			for (int b = 0; b < 4; ++b) {
				const uint8_t byte = uint8_t(z >> (8 * b));
				if (uses_planes(p_codec)) {
					raw[size_t(b) * n + k] = byte;
				} else {
					raw[k * 4 + b] = byte;
				}
			}
		}
	}
	std::vector<uint8_t> out(ZSTD_compressBound(raw.size()));
	const size_t written = ZSTD_compress(out.data(), out.size(), raw.data(), raw.size(), p_zstd_level);
	if (ZSTD_isError(written)) {
		return {};
	}
	out.resize(written);
	return out;
}

bool decode_tile(const uint8_t *p_data, size_t p_size, int p_side, Codec p_codec, int32_t *r_values) {
	const size_t n = size_t(p_side) * p_side;
	std::vector<uint8_t> raw(n * 4);
	const size_t got = ZSTD_decompress(raw.data(), raw.size(), p_data, p_size);
	if (ZSTD_isError(got) || got != raw.size()) {
		return false;
	}
	for (int j = 0; j < p_side; ++j) {
		for (int i = 0; i < p_side; ++i) {
			const size_t k = size_t(j) * p_side + i;
			uint32_t z = 0;
			for (int b = 0; b < 4; ++b) {
				const uint8_t byte = uses_planes(p_codec) ? raw[size_t(b) * n + k] : raw[k * 4 + b];
				z |= uint32_t(byte) << (8 * b);
			}
			const int64_t v = int64_t(unzigzag(z)) + predict(r_values, p_side, i, j, p_codec);
			r_values[k] = int32_t(v);
		}
	}
	return true;
}

uint32_t crc32(const uint8_t *p_data, size_t p_size) {
	const std::array<uint32_t, 256> &table = crc_table();
	uint32_t c = 0xFFFFFFFFu;
	for (size_t k = 0; k < p_size; ++k) {
		c = table[(c ^ p_data[k]) & 0xFFu] ^ (c >> 8);
	}
	return c ^ 0xFFFFFFFFu;
}

} // namespace regolith
