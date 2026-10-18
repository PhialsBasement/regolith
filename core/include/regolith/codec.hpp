#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace regolith {

// See docs/rgp1.md for the exact transforms.
enum Codec : uint32_t {
	CODEC_ROW_DELTA = 1,
	CODEC_MED = 2,
	CODEC_MED_NO_PLANES = 3,
};

std::vector<uint8_t> encode_tile(const int32_t *p_values, int p_side, Codec p_codec, int p_zstd_level);
bool decode_tile(const uint8_t *p_data, size_t p_size, int p_side, Codec p_codec, int32_t *r_values);

// CRC-32/ISO-HDLC, the one zlib computes.
uint32_t crc32(const uint8_t *p_data, size_t p_size);

} // namespace regolith
