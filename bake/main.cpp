#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <numbers>
#include <string>
#include <vector>

#include "regolith/orbit.hpp"
#include "samplers.hpp"
#include "tile_bake.hpp"

using namespace regolith;
using namespace regolith::bake;

namespace {

const char *USAGE =
		"usage:\n"
		"  regolith_bake dem --name NAME --raster F --width W --height H --datum-radius R --out DIR\n"
		"        [--type float32|int16] [--nodata V] [--intervals N] [--attribution TEXT]\n"
		"        [--datum-raster F --datum-width W --datum-height H] [options]\n"
		"      any body from a global equirectangular raster of heights (m) over its datum:\n"
		"      row 0 at +90 lat, column 0 at -180 lon, little endian; NaN or --nodata missing.\n"
		"      The datum is a sphere of radius R, or per point a float32 raster of radii (m).\n"
		"      Writes NAME_height.rgp (and NAME_datum.rgp with a datum raster).\n"
		"  regolith_bake slice --in PACK --out PACK --max-level N\n"
		"      the pack's levels up to N, tiles copied: a small pack for demos\n"
		"  regolith_bake mars --dem TIF --areoid IMG [--craters BIN] [--pits BIN] --out DIR [options]\n"
		"  regolith_bake phobos --icq TAB --out DIR [options]\n"
		"  regolith_bake orbit --horizons FILE[,FILE...] --out RGE\n"
		"        [--parent-gm M3S2 --parent-j2 J2 --parent-radius M --pole-ra DEG --pole-dec DEG]\n"
		"      (the parent defaults to Mars)\n"
		"options: --min-level N --max-level N --codec 1|2|3 --zstd N --step M --trial\n";

struct Args {
	std::string mode;
	std::map<std::string, std::string> opts;

	std::string get(const std::string &p_key, const std::string &p_default = "") const {
		auto it = opts.find(p_key);
		return it == opts.end() ? p_default : it->second;
	}
	bool has(const std::string &p_key) const { return opts.count(p_key) != 0; }
};

bool parse(int p_argc, char **p_argv, Args &r_args) {
	if (p_argc < 2) {
		return false;
	}
	r_args.mode = p_argv[1];
	for (int k = 2; k < p_argc; ++k) {
		const std::string key = p_argv[k];
		if (key.rfind("--", 0) != 0) {
			return false;
		}
		const bool flag = key == "--trial";
		r_args.opts[key.substr(2)] = flag ? "1" : (k + 1 < p_argc ? p_argv[++k] : "");
	}
	return true;
}

std::vector<TileKey> trial_keys(int p_max_level) {
	std::vector<TileKey> keys;
	size_t counter = 0;
	for (int face = 0; face < FACE_COUNT; ++face) {
		for (int level = 0; level <= p_max_level; ++level) {
			const uint32_t n = 1u << level;
			const size_t stride = level <= 2 ? 1 : (level == p_max_level ? 197 : 61);
			for (uint32_t x = 0; x < n; ++x) {
				for (uint32_t y = 0; y < n; ++y) {
					if (counter++ % stride == 0) {
						keys.push_back(TileKey::make(face, level, x, y));
					}
				}
			}
		}
	}
	return keys;
}

void report(const char *p_name, const BakeStats &p_stats) {
	std::printf("%s: %llu tiles, %.3f GB, %.1f s, %llu invalid samples\n", p_name,
			(unsigned long long)p_stats.tiles, double(p_stats.bytes) / 1e9, p_stats.seconds,
			(unsigned long long)p_stats.invalid_samples);
}

BakeConfig config_from(const Args &p_args, const std::string &p_file, const char *p_body, int p_intervals,
		int p_max_level, double p_step, double p_r_ref) {
	BakeConfig c;
	c.path = p_args.get("out", ".") + "/" + p_file;
	c.layout = { p_intervals, 2 };
	c.min_level = std::atoi(p_args.get("min-level", "0").c_str());
	c.max_level = std::atoi(p_args.get("max-level", std::to_string(p_max_level)).c_str());
	c.zstd_level = std::atoi(p_args.get("zstd", "9").c_str());
	c.header.body = p_body;
	c.header.codec = uint32_t(std::atoi(p_args.get("codec", "2").c_str()));
	c.header.step = p_args.has("step") ? std::atof(p_args.get("step").c_str()) : p_step;
	c.header.r_ref = p_r_ref;
	return c;
}

int run_mars(const Args &p_args) {
	MarsDem dem;
	Areoid areoid;
	Catalog catalog;
	std::printf("loading DEM and building mips\n");
	if (!dem.open(p_args.get("dem"), 8) || !areoid.open(p_args.get("areoid"))) {
		return 1;
	}
	if (p_args.has("craters") && !catalog.load_craters(p_args.get("craters"))) {
		return 1;
	}
	if (p_args.has("pits") && !catalog.load_pits(p_args.get("pits"))) {
		return 1;
	}
	catalog.build_index();
	std::printf("catalogue: %zu craters, %zu pits\n", catalog.crater_count(), catalog.pit_count());
	const MarsHeightSampler height(dem, areoid, &catalog);
	if (p_args.has("trial")) {
		codec_trial(height, { 208, 2 }, trial_keys(MarsHeightSampler::DATA_LEVEL));
		return 0;
	}

	BakeConfig h = config_from(p_args, "mars_height.rgp", "mars", 208, 7, 1.0 / 4.0, MarsHeightSampler::R_REF);
	h.header.layer = PACK_LAYER_HEIGHT;
	h.attribution = "Mars HRSC/MOLA Blended DEM Global 200m v2 (USGS Astrogeology; HRSC: ESA/DLR/FU Berlin; "
					"MOLA: NASA/GSFC). Areoid: MGS MOLA MEGDR mega90n000cb (PDS Geosciences Node). "
					"Crater restoration: Lagain et al. 2021 Mars crater database, Garvin 2003 morphometry "
					"(mors crater_restore.bin). Cave pits: MGC3, Cushing 2017, USGS (mors cave_sites.bin).";
	BakeStats hs;
	if (!bake_pack(height, h, hs)) {
		return 1;
	}
	report("mars_height", hs);
	std::printf("  mip fallbacks: %llu\n", (unsigned long long)height.mip_fallbacks());

	const MarsAreoidSampler geoid(areoid);
	BakeConfig a = config_from(p_args, "mars_areoid.rgp", "mars", 208, 3, 1.0 / 16.0, MarsHeightSampler::R_REF);
	a.min_level = 0;
	a.max_level = 3;
	a.header.layer = PACK_LAYER_AREOID;
	a.attribution = "Areoid: MGS MOLA MEGDR mega90n000cb (PDS Geosciences Node).";
	BakeStats as;
	if (!bake_pack(geoid, a, as)) {
		return 1;
	}
	report("mars_areoid", as);
	return 0;
}

double number(const Args &p_args, const std::string &p_key, double p_default) {
	return p_args.has(p_key) ? std::atof(p_args.get(p_key).c_str()) : p_default;
}

// The level whose lattice spacing is nearest the raster's
// pixel: finer levels would only interpolate.
int data_level_for(double p_pixel_deg, int p_intervals) {
	const double pixel_rad = p_pixel_deg * std::numbers::pi / 180.0;
	const double l = std::log2((std::numbers::pi / 2.0) / (double(p_intervals) * pixel_rad));
	return std::clamp(int(std::lround(l)), 0, 24);
}

bool raster_spec(const Args &p_args, const std::string &p_prefix, RasterSpec &r_spec) {
	r_spec.path = p_args.get(p_prefix + "raster");
	r_spec.width = std::atoll(p_args.get(p_prefix + "width", "0").c_str());
	r_spec.height = std::atoll(p_args.get(p_prefix + "height", "0").c_str());
	r_spec.type = p_args.get(p_prefix + "type", "float32") == "int16" ? RasterSpec::INT16 : RasterSpec::FLOAT32;
	if (p_args.has(p_prefix + "nodata")) {
		r_spec.nodata = std::atof(p_args.get(p_prefix + "nodata").c_str());
	}
	if (r_spec.path.empty() || r_spec.width <= 0 || r_spec.height <= 0) {
		std::fprintf(stderr, "dem: --%sraster, --%swidth and --%sheight are required\n", p_prefix.c_str(),
				p_prefix.c_str(), p_prefix.c_str());
		return false;
	}
	if (std::abs(double(r_spec.width) - 2.0 * double(r_spec.height)) > 2.0) {
		std::fprintf(stderr, "dem: %s is %lld x %lld; a global raster is twice as wide as it is high\n",
				r_spec.path.c_str(), (long long)r_spec.width, (long long)r_spec.height);
		return false;
	}
	return true;
}

int run_dem(const Args &p_args) {
	const std::string name = p_args.get("name");
	const double radius = number(p_args, "datum-radius", 0.0);
	if (name.empty() || name.size() > 31 || radius <= 0.0) {
		std::fprintf(stderr, "dem: --name (up to 31 characters) and --datum-radius are required\n");
		return 2;
	}
	RasterSpec spec;
	if (!raster_spec(p_args, "", spec)) {
		return 2;
	}
	const int intervals = std::atoi(p_args.get("intervals", "256").c_str());
	const int data_level = data_level_for(360.0 / double(spec.width), intervals);
	std::printf("%s: %lld x %lld raster, %.1f m pixels; data level %d\n", name.c_str(), (long long)spec.width,
			(long long)spec.height, 360.0 / double(spec.width) * std::numbers::pi / 180.0 * radius, data_level);
	RasterDem dem;
	if (!dem.open(spec, data_level + 1)) {
		return 1;
	}
	RasterDem datum;
	const bool has_datum = p_args.has("datum-raster");
	if (has_datum) {
		RasterSpec ds;
		if (!raster_spec(p_args, "datum-", ds) || !datum.open(ds, 1)) {
			return 1;
		}
	}
	const DemHeightSampler height(dem, has_datum ? &datum : nullptr, radius, radius, data_level);
	if (p_args.has("trial")) {
		codec_trial(height, { intervals, 2 }, trial_keys(data_level));
		return 0;
	}
	BakeConfig h = config_from(p_args, name + "_height.rgp", name.c_str(), intervals, data_level, 1.0 / 4.0, radius);
	h.header.layer = PACK_LAYER_HEIGHT;
	h.attribution = p_args.get("attribution");
	BakeStats hs;
	if (!bake_pack(height, h, hs)) {
		return 1;
	}
	report((name + "_height").c_str(), hs);
	std::printf("  mip fallbacks: %llu\n", (unsigned long long)height.mip_fallbacks());
	if (has_datum) {
		const DemDatumSampler geoid(height, radius);
		BakeConfig a = config_from(p_args, name + "_datum.rgp", name.c_str(), intervals, 3, 1.0 / 16.0, radius);
		a.min_level = 0;
		a.max_level = 3;
		a.header.layer = PACK_LAYER_AREOID;
		a.attribution = p_args.get("attribution");
		BakeStats as;
		if (!bake_pack(geoid, a, as)) {
			return 1;
		}
		report((name + "_datum").c_str(), as);
	}
	return 0;
}

// A pack's levels up to --max-level, tiles copied as they are:
// a small pack for demos, its finer detail left to synthesis.
int run_slice(const Args &p_args) {
	PackReader in;
	if (!in.open(p_args.get("in"))) {
		std::fprintf(stderr, "slice: cannot open %s\n", p_args.get("in").c_str());
		return 1;
	}
	const int max_level = std::atoi(p_args.get("max-level", "2").c_str());
	PackHeader h = in.header();
	h.max_level = uint32_t(std::min<int>(max_level, int(h.max_level)));
	PackWriter out;
	if (!out.open(p_args.get("out"), h, in.attribution())) {
		return 1;
	}
	std::vector<uint8_t> blob;
	size_t n = 0;
	for (const PackEntry &e : in.entries()) {
		if (e.key.level() > max_level) {
			continue;
		}
		if (!in.read_blob(e, blob) || !out.add(e.key, blob, e.min, e.max)) {
			std::fprintf(stderr, "slice: tile failed\n");
			return 1;
		}
		++n;
	}
	if (!out.finish()) {
		return 1;
	}
	std::printf("slice: %zu tiles, levels %u-%u, %.1f MB\n", n, h.min_level, h.max_level,
			double(out.bytes_written()) / 1e6);
	return 0;
}

int run_phobos(const Args &p_args) {
	IcqShape shape;
	std::printf("loading ICQ shape\n");
	if (!shape.open(p_args.get("icq"))) {
		return 1;
	}
	std::printf("  %zu triangles\n", shape.triangle_count());
	const PhobosHeightSampler height(shape);
	if (p_args.has("trial")) {
		codec_trial(height, { 256, 2 }, trial_keys(1));
		return 0;
	}
	BakeConfig h = config_from(p_args, "phobos_height.rgp", "phobos", 256, 1, 1.0 / 64.0, PhobosHeightSampler::R_REF);
	h.header.layer = PACK_LAYER_HEIGHT;
	h.attribution = "Phobos SPC shape model, R. Gaskell (PDS Small Bodies Node, "
					"gaskell.phobos.shape-model, ICQ q=512).";
	BakeStats hs;
	if (!bake_pack(height, h, hs)) {
		return 1;
	}
	report("phobos_height", hs);
	std::printf("  multi-hit rays: %llu, misses: %llu\n", (unsigned long long)height.multi_hits(),
			(unsigned long long)height.misses());
	return 0;
}

// Horizons CSV vectors ($$SOE..$$EOE rows: JD, date, x, y, z,
// vx, vy, vz in km and km/s) into an RGE1 table.
int run_orbit(const Args &p_args) {
	struct Row {
		double jd;
		OrbitState s;
	};
	std::vector<Row> rows;
	std::string list = p_args.get("horizons");
	size_t from = 0;
	while (from <= list.size()) {
		const size_t comma = list.find(',', from);
		const std::string path = list.substr(from, comma == std::string::npos ? std::string::npos : comma - from);
		from = comma == std::string::npos ? list.size() + 1 : comma + 1;
		FILE *f = std::fopen(path.c_str(), "r");
		if (!f) {
			std::fprintf(stderr, "orbit: cannot open %s\n", path.c_str());
			return 1;
		}
		char line[512];
		bool inside = false;
		while (std::fgets(line, sizeof(line), f)) {
			const std::string l(line);
			if (l.rfind("$$SOE", 0) == 0) {
				inside = true;
				continue;
			}
			if (l.rfind("$$EOE", 0) == 0) {
				inside = false;
				continue;
			}
			if (!inside) {
				continue;
			}
			Row r;
			char date[64];
			double v[6];
			if (std::sscanf(line, "%lf, %63[^,], %lf, %lf, %lf, %lf, %lf, %lf", &r.jd, date, &v[0], &v[1], &v[2], &v[3],
						&v[4], &v[5]) == 8) {
				r.s = { DVec3(v[0], v[1], v[2]) * 1000.0, DVec3(v[3], v[4], v[5]) * 1000.0 };
				rows.push_back(r);
			}
		}
		std::fclose(f);
	}
	std::sort(rows.begin(), rows.end(), [](const Row &p_a, const Row &p_b) { return p_a.jd < p_b.jd; });
	rows.erase(std::unique(rows.begin(), rows.end(), [](const Row &p_a, const Row &p_b) { return std::abs(p_a.jd - p_b.jd) < 1e-9; }),
			rows.end());
	if (rows.size() < 2) {
		std::fprintf(stderr, "orbit: no samples\n");
		return 1;
	}
	OrbitTable table;
	table.t0_ = (rows.front().jd - 2451545.0) * 86400.0;
	// From the whole span: one JD difference carries ~0.1 ms of
	// rounding, which would add up to seconds over the table.
	table.step_ = std::round((rows.back().jd - rows.front().jd) * 86400.0 / double(rows.size() - 1) * 1e6) * 1e-6;
	for (size_t k = 1; k < rows.size(); ++k) {
		if (std::abs((rows[k].jd - rows[k - 1].jd) * 86400.0 - table.step_) > 1e-3) {
			std::fprintf(stderr, "orbit: uneven step at row %zu\n", k);
			return 1;
		}
		table.states_.push_back(rows[k - 1].s);
	}
	table.states_.push_back(rows.back().s);
	// the parent: Mars unless given (GM and J2 from JPL, the IAU
	// pole at J2000)
	table.parent_gm_ = number(p_args, "parent-gm", 4.282837e13);
	table.parent_j2_ = number(p_args, "parent-j2", 1.96045e-3);
	table.parent_radius_ = number(p_args, "parent-radius", 3396200.0);
	table.pole_ra_ = number(p_args, "pole-ra", 317.68143);
	table.pole_dec_ = number(p_args, "pole-dec", 52.88650);
	if (!table.save(p_args.get("out"))) {
		return 1;
	}
	std::printf("orbit: %zu samples every %.0f s from J2000%+.0f s to %+.0f s\n", table.states_.size(), table.step_,
			table.t0_, table.end());
	return 0;
}

} // namespace

int main(int p_argc, char **p_argv) {
	Args args;
	if (!parse(p_argc, p_argv, args)) {
		std::fputs(USAGE, stderr);
		return 2;
	}
	if (args.mode == "dem") {
		return run_dem(args);
	}
	if (args.mode == "slice") {
		return run_slice(args);
	}
	if (args.mode == "mars") {
		return run_mars(args);
	}
	if (args.mode == "phobos") {
		return run_phobos(args);
	}
	if (args.mode == "orbit") {
		return run_orbit(args);
	}
	std::fputs(USAGE, stderr);
	return 2;
}
