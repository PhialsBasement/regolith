#include "regolith/pack.hpp"

#include <algorithm>
#include <cstring>
#include <mutex>

#ifndef _WIN32
#include <fcntl.h>
#include <unistd.h>
#endif

#include "regolith/codec.hpp"

namespace regolith {

namespace {

bool seek(FILE *p_file, uint64_t p_offset) {
#ifdef _WIN32
	return _fseeki64(p_file, int64_t(p_offset), SEEK_SET) == 0;
#else
	return fseeko(p_file, off_t(p_offset), SEEK_SET) == 0;
#endif
}

template <typename T>
void put(uint8_t *p_buf, size_t p_at, T p_value) {
	std::memcpy(p_buf + p_at, &p_value, sizeof(T));
}

template <typename T>
T get(const uint8_t *p_buf, size_t p_at) {
	T value;
	std::memcpy(&value, p_buf + p_at, sizeof(T));
	return value;
}

void encode_header(const PackHeader &p_h, uint8_t r_buf[PACK_HEADER_SIZE]) {
	std::memset(r_buf, 0, PACK_HEADER_SIZE);
	std::memcpy(r_buf, "RGP1", 4);
	put<uint32_t>(r_buf, 4, p_h.version);
	put<uint32_t>(r_buf, 8, uint32_t(PACK_HEADER_SIZE));
	put<uint32_t>(r_buf, 12, p_h.layer);
	std::memcpy(r_buf + 16, p_h.body.data(), std::min<size_t>(p_h.body.size(), 31));
	put<uint32_t>(r_buf, 48, p_h.intervals);
	put<uint32_t>(r_buf, 52, p_h.border);
	put<uint32_t>(r_buf, 56, p_h.projection);
	put<uint32_t>(r_buf, 60, p_h.sample_format);
	put<double>(r_buf, 64, p_h.step);
	put<double>(r_buf, 72, p_h.r_ref);
	put<uint32_t>(r_buf, 80, p_h.codec);
	put<uint32_t>(r_buf, 84, p_h.codec_level);
	put<uint32_t>(r_buf, 88, p_h.min_level);
	put<uint32_t>(r_buf, 92, p_h.max_level);
	put<uint64_t>(r_buf, 96, p_h.tile_count);
	put<uint64_t>(r_buf, 104, p_h.index_offset);
	put<uint64_t>(r_buf, 112, p_h.attribution_offset);
	put<uint32_t>(r_buf, 120, p_h.attribution_size);
}

bool decode_header(const uint8_t p_buf[PACK_HEADER_SIZE], PackHeader &r_h) {
	if (std::memcmp(p_buf, "RGP1", 4) != 0 || get<uint32_t>(p_buf, 8) != PACK_HEADER_SIZE) {
		return false;
	}
	r_h.version = get<uint32_t>(p_buf, 4);
	r_h.layer = get<uint32_t>(p_buf, 12);
	r_h.body.assign(reinterpret_cast<const char *>(p_buf + 16), strnlen(reinterpret_cast<const char *>(p_buf + 16), 32));
	r_h.intervals = get<uint32_t>(p_buf, 48);
	r_h.border = get<uint32_t>(p_buf, 52);
	r_h.projection = get<uint32_t>(p_buf, 56);
	r_h.sample_format = get<uint32_t>(p_buf, 60);
	r_h.step = get<double>(p_buf, 64);
	r_h.r_ref = get<double>(p_buf, 72);
	r_h.codec = get<uint32_t>(p_buf, 80);
	r_h.codec_level = get<uint32_t>(p_buf, 84);
	r_h.min_level = get<uint32_t>(p_buf, 88);
	r_h.max_level = get<uint32_t>(p_buf, 92);
	r_h.tile_count = get<uint64_t>(p_buf, 96);
	r_h.index_offset = get<uint64_t>(p_buf, 104);
	r_h.attribution_offset = get<uint64_t>(p_buf, 112);
	r_h.attribution_size = get<uint32_t>(p_buf, 120);
	return r_h.version == 1;
}

} // namespace

PackWriter::~PackWriter() {
	if (file_) {
		std::fclose(file_);
	}
}

bool PackWriter::open(const std::string &p_path, const PackHeader &p_header, const std::string &p_attribution) {
	file_ = std::fopen(p_path.c_str(), "wb");
	if (!file_) {
		return false;
	}
	header_ = p_header;
	uint8_t blank[PACK_HEADER_SIZE] = {};
	std::fwrite(blank, 1, PACK_HEADER_SIZE, file_);
	header_.attribution_offset = PACK_HEADER_SIZE;
	header_.attribution_size = uint32_t(p_attribution.size());
	std::fwrite(p_attribution.data(), 1, p_attribution.size(), file_);
	offset_ = PACK_HEADER_SIZE + p_attribution.size();
	return true;
}

bool PackWriter::add(TileKey p_key, const std::vector<uint8_t> &p_blob, float p_min, float p_max) {
	if (!entries_.empty() && !(entries_.back().key.bits < p_key.bits)) {
		return false;
	}
	PackEntry e;
	e.key = p_key;
	e.offset = offset_;
	e.size = uint32_t(p_blob.size());
	e.crc = crc32(p_blob.data(), p_blob.size());
	e.min = p_min;
	e.max = p_max;
	if (std::fwrite(p_blob.data(), 1, p_blob.size(), file_) != p_blob.size()) {
		return false;
	}
	offset_ += p_blob.size();
	entries_.push_back(e);
	return true;
}

bool PackWriter::finish() {
	header_.tile_count = entries_.size();
	header_.index_offset = offset_;
	std::vector<uint8_t> index(entries_.size() * PACK_ENTRY_SIZE);
	for (size_t k = 0; k < entries_.size(); ++k) {
		uint8_t *p = index.data() + k * PACK_ENTRY_SIZE;
		put<uint64_t>(p, 0, entries_[k].key.bits);
		put<uint64_t>(p, 8, entries_[k].offset);
		put<uint32_t>(p, 16, entries_[k].size);
		put<uint32_t>(p, 20, entries_[k].crc);
		put<float>(p, 24, entries_[k].min);
		put<float>(p, 28, entries_[k].max);
	}
	bool ok = std::fwrite(index.data(), 1, index.size(), file_) == index.size();
	offset_ += index.size();
	uint8_t head[PACK_HEADER_SIZE];
	encode_header(header_, head);
	ok = ok && seek(file_, 0);
	ok = ok && std::fwrite(head, 1, PACK_HEADER_SIZE, file_) == PACK_HEADER_SIZE;
	ok = std::fclose(file_) == 0 && ok;
	file_ = nullptr;
	return ok;
}

PackReader::~PackReader() {
	if (file_) {
		std::fclose(file_);
	}
#ifndef _WIN32
	if (fd_ >= 0) {
		::close(fd_);
	}
#endif
}

bool PackReader::open(const std::string &p_path) {
	file_ = std::fopen(p_path.c_str(), "rb");
	if (!file_) {
		return false;
	}
#ifndef _WIN32
	fd_ = ::open(p_path.c_str(), O_RDONLY);
#endif
	uint8_t head[PACK_HEADER_SIZE];
	if (std::fread(head, 1, PACK_HEADER_SIZE, file_) != PACK_HEADER_SIZE || !decode_header(head, header_)) {
		return false;
	}
	attribution_.resize(header_.attribution_size);
	if (!seek(file_, header_.attribution_offset) ||
			std::fread(attribution_.data(), 1, attribution_.size(), file_) != attribution_.size()) {
		return false;
	}
	std::vector<uint8_t> index(header_.tile_count * PACK_ENTRY_SIZE);
	if (!seek(file_, header_.index_offset) || std::fread(index.data(), 1, index.size(), file_) != index.size()) {
		return false;
	}
	entries_.resize(header_.tile_count);
	for (size_t k = 0; k < entries_.size(); ++k) {
		const uint8_t *p = index.data() + k * PACK_ENTRY_SIZE;
		entries_[k].key.bits = get<uint64_t>(p, 0);
		entries_[k].offset = get<uint64_t>(p, 8);
		entries_[k].size = get<uint32_t>(p, 16);
		entries_[k].crc = get<uint32_t>(p, 20);
		entries_[k].min = get<float>(p, 24);
		entries_[k].max = get<float>(p, 28);
	}
	return true;
}

const PackEntry *PackReader::find(TileKey p_key) const {
	auto it = std::lower_bound(entries_.begin(), entries_.end(), p_key,
			[](const PackEntry &p_e, TileKey p_k) { return p_e.key.bits < p_k.bits; });
	return (it != entries_.end() && it->key == p_key) ? &*it : nullptr;
}

bool PackReader::read_blob(const PackEntry &p_entry, std::vector<uint8_t> &r_blob) const {
	r_blob.resize(p_entry.size);
	if (!seek(file_, p_entry.offset)) {
		return false;
	}
	if (std::fread(r_blob.data(), 1, r_blob.size(), file_) != r_blob.size()) {
		return false;
	}
	return crc32(r_blob.data(), r_blob.size()) == p_entry.crc;
}

bool PackReader::read_blob_concurrent(const PackEntry &p_entry, std::vector<uint8_t> &r_blob) const {
	r_blob.resize(p_entry.size);
#ifndef _WIN32
	size_t done = 0;
	while (done < r_blob.size()) {
		const ssize_t got = ::pread(fd_, r_blob.data() + done, r_blob.size() - done, off_t(p_entry.offset + done));
		if (got <= 0) {
			return false;
		}
		done += size_t(got);
	}
#else
	static std::mutex lock;
	std::lock_guard<std::mutex> guard(lock);
	if (!seek(file_, p_entry.offset) || std::fread(r_blob.data(), 1, r_blob.size(), file_) != r_blob.size()) {
		return false;
	}
#endif
	return crc32(r_blob.data(), r_blob.size()) == p_entry.crc;
}

} // namespace regolith
