#!/usr/bin/env python3
# This Source Code Form is subject to the terms of the
# Mozilla Public License, v. 2.0. If a copy of the MPL was
# not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.
"""Independent check of RGP1 packs against their sources.

Implements docs/rgp1.md from scratch and shares no code with
regolith_bake, so a mismatch means one of the two misreads the spec.

Usage:
  verify_pack.py mars PACK --dem TIF --areoid IMG [--samples N] [--seed S]
  verify_pack.py areoid PACK --areoid IMG [--samples N]
  verify_pack.py phobos PACK --icq TAB [--samples N]
"""
import argparse
import math
import random
import struct
import sys
import zlib
from compression import zstd

import numpy as np

FACES = [
    ((1, 0, 0), (0, 1, 0), (0, 0, 1)),
    ((-1, 0, 0), (0, -1, 0), (0, 0, 1)),
    ((0, 1, 0), (-1, 0, 0), (0, 0, 1)),
    ((0, -1, 0), (1, 0, 0), (0, 0, 1)),
    ((0, 0, 1), (0, 1, 0), (-1, 0, 0)),
    ((0, 0, -1), (0, 1, 0), (1, 0, 0)),
]


# Pack reading.

class Pack:
    def __init__(self, path):
        self.f = open(path, "rb")
        h = self.f.read(256)
        if h[:4] != b"RGP1":
            raise ValueError("not an RGP1 pack")
        (self.version, hsize, self.layer) = struct.unpack_from("<III", h, 4)
        self.body = h[16:48].split(b"\0")[0].decode()
        (self.n, self.b, self.projection, self.fmt) = struct.unpack_from("<IIII", h, 48)
        (self.step, self.r_ref) = struct.unpack_from("<dd", h, 64)
        (self.codec, self.codec_level, self.min_level, self.max_level) = struct.unpack_from("<IIII", h, 80)
        (self.count, self.index_offset, self.attr_offset) = struct.unpack_from("<QQQ", h, 96)
        (self.attr_size,) = struct.unpack_from("<I", h, 120)
        self.side = self.n + 1 + 2 * self.b
        self.f.seek(self.attr_offset)
        self.attribution = self.f.read(self.attr_size).decode()
        self.f.seek(self.index_offset)
        raw = self.f.read(32 * self.count)
        dt = np.dtype([("key", "<u8"), ("off", "<u8"), ("size", "<u4"), ("crc", "<u4"),
                       ("min", "<f4"), ("max", "<f4")])
        self.index = np.frombuffer(raw, dtype=dt)
        self.lookup = {int(k): i for i, k in enumerate(self.index["key"])}
        self.cache = {}

    def has(self, key):
        return key in self.lookup

    def tile(self, key):
        """Decoded int32 tile as a (side, side) array, j rows."""
        if key in self.cache:
            return self.cache[key]
        e = self.index[self.lookup[key]]
        self.f.seek(int(e["off"]))
        blob = self.f.read(int(e["size"]))
        if zlib.crc32(blob) != int(e["crc"]):
            raise ValueError("crc mismatch for key %x" % key)
        raw = zstd.decompress(blob)
        n = self.side * self.side
        if self.codec in (1, 2):
            planes = np.frombuffer(raw, dtype=np.uint8).reshape(4, n).astype(np.uint32)
            z = planes[0] | (planes[1] << 8) | (planes[2] << 16) | (planes[3] << 24)
        else:
            z = np.frombuffer(raw, dtype="<u4").astype(np.uint32)
        e_res = ((z >> 1).astype(np.int64) ^ -(z & 1).astype(np.int64)).tolist()
        side = self.side
        v = [0] * n
        for j in range(side):
            for i in range(side):
                k = j * side + i
                if j == 0:
                    pred = 0 if i == 0 else v[k - 1]
                elif i == 0:
                    pred = v[k - side]
                else:
                    a = v[k - 1]
                    b = v[k - side]
                    if self.codec == 1:
                        pred = a
                    else:
                        c = v[k - side - 1]
                        if c >= max(a, b):
                            pred = min(a, b)
                        elif c <= min(a, b):
                            pred = max(a, b)
                        else:
                            pred = a + b - c
                v[k] = pred + e_res[k]
        arr = np.array(v, dtype=np.int64).reshape(side, side)
        interior = arr[self.b:self.b + self.n + 1, self.b:self.b + self.n + 1] * self.step
        if np.float32(interior.min()) != e["min"] or np.float32(interior.max()) != e["max"]:
            raise ValueError("index min/max disagree with tile %x" % key)
        self.cache[key] = arr
        return arr


def make_key(face, level, x, y):
    return (face << 61) | (level << 56) | (x << 28) | y


def split_key(key):
    return key >> 61, (key >> 56) & 31, (key >> 28) & 0xFFFFFFF, key & 0xFFFFFFF


# Geometry, written from the spec.

def warp(c):
    if c == 1.0:
        return 1.0
    if c == -1.0:
        return -1.0
    return math.tan(math.pi / 4.0 * c)


def fold(face, gi, gj, m):
    f, u, v = FACES[face]
    cu, cv, cf = 2 * gi - m, 2 * gj - m, m
    if cu > m:
        cf -= cu - m
        cu = m
    elif cu < -m:
        cf -= -m - cu
        cu = -m
    if cv > m:
        cf -= cv - m
        cv = m
    elif cv < -m:
        cf -= -m - cv
        cv = -m
    return tuple(f[k] * cf + u[k] * cu + v[k] * cv for k in range(3))


def canonical(p, m):
    for face, (f, u, v) in enumerate(FACES):
        if sum(f[k] * p[k] for k in range(3)) == m:
            gi = (sum(u[k] * p[k] for k in range(3)) + m) // 2
            gj = (sum(v[k] * p[k] for k in range(3)) + m) // 2
            return face, gi, gj
    raise ValueError("point off the cube")


def normalize(x, y, z):
    n = math.sqrt(x * x + y * y + z * z)
    return x / n, y / n, z / n


def centre_dir(p, m):
    return normalize(warp(p[0] / m), warp(p[1] / m), warp(p[2] / m))


def face_dir(face, s, t):
    f, u, v = FACES[face]
    ws, wt = warp(s), warp(t)
    return normalize(*(f[k] + u[k] * ws + v[k] * wt for k in range(3)))


def lat_lon(d):
    return (math.degrees(math.atan2(d[2], math.hypot(d[0], d[1]))),
            math.degrees(math.atan2(d[1], d[0])))


def subsample_dirs(face, gi, gj, m):
    for b in (-1, 0, 1):
        for a in (-1, 0, 1):
            s = ((2 * gi - m) + (2.0 / 3.0) * a) / m
            t = ((2 * gj - m) + (2.0 / 3.0) * b) / m
            yield face_dir(face, s, t)


def round_away(x):
    return int(math.copysign(math.floor(abs(x) + 0.5), x))


# Mars sources.

class RasterDem:
    """A raw global raster: row 0 at +90, column 0 at -180."""

    def __init__(self, path, width, height, dtype="<f4", offset=0, deg=None, nodata=None, mips=8):
        self.W, self.H = width, height
        self.DEG = deg if deg else 360.0 / width
        self.nodata = nodata
        self.raw = np.memmap(path, dtype=dtype, mode="r", offset=offset, shape=(self.H, self.W))
        self.dims = [(self.W, self.H)]
        for _ in range(mips):
            w, h = self.dims[-1]
            self.dims.append(((w + 1) // 2, (h + 1) // 2))
        self.pixels = {}
        self.north = self.row_mean(0)
        self.south = self.row_mean(self.H - 1)

    def masked(self, block):
        block = np.asarray(block).astype(np.float64)
        if self.nodata is not None:
            block[block == self.nodata] = np.nan
        return block

    def row_mean(self, y):
        row = self.masked(self.raw[y])
        row = row[~np.isnan(row)]
        return float(row.mean()) if row.size else 0.0

    def pixel(self, k, x, y):
        key = (k, x, y)
        if key in self.pixels:
            return self.pixels[key]
        span = 1 << k
        x0, y0 = x * span, y * span
        x1, y1 = min(x0 + span, self.W), min(y0 + span, self.H)
        block = self.masked(self.raw[y0:y1, x0:x1])
        for _ in range(k):
            h, w = block.shape
            padded = np.full((h + h % 2, w + w % 2), np.nan)
            padded[:h, :w] = block
            q = padded.reshape(padded.shape[0] // 2, 2, padded.shape[1] // 2, 2)
            valid = ~np.isnan(q)
            total = np.where(valid, q, 0.0).sum(axis=(1, 3))
            count = valid.sum(axis=(1, 3))
            block = np.where(count > 0, total / np.maximum(count, 1), np.nan)
        value = float(block[0, 0])
        self.pixels[key] = value
        return value

    def sample(self, k, lat, lon):
        w, h = self.dims[k]
        deg = self.DEG * (1 << k)
        period = 360.0 / deg
        fx = math.fmod((lon + 180.0) / deg - 0.5, period)
        if fx < 0:
            fx += period
        if fx <= w - 1:
            x0 = int(math.floor(fx))
            x1 = min(x0 + 1, w - 1)
            tx = fx - x0
        else:
            x0, x1 = w - 1, 0
            tx = (fx - (w - 1)) / (period - (w - 1))
        fy = min(max((90.0 - lat) / deg - 0.5, 0.0), h - 1)
        y0 = int(math.floor(fy))
        y1 = min(y0 + 1, h - 1)
        ty = fy - y0
        taps = [((1 - tx) * (1 - ty), x0, y0), (tx * (1 - ty), x1, y0),
                ((1 - tx) * ty, x0, y1), (tx * ty, x1, y1)]
        total = weight = 0.0
        for wgt, x, y in taps:
            v = self.pixel(k, x, y)
            if not math.isnan(v):
                total += wgt * v
                weight += wgt
        return total / weight if weight > 1e-12 else math.nan


def MarsDem(path):
    return RasterDem(path, 106694, 53347, dtype="<i2", offset=854272, deg=0.003374120830641, nodata=-32768)


class Areoid:
    def __init__(self, path):
        self.r = np.fromfile(path, dtype=">i2").reshape(720, 1440).astype(np.float64) + 3396000.0

    def radius(self, lat, lon):
        lon360 = lon + 360.0 if lon < 0 else lon
        fx = lon360 * 4.0 - 0.5
        fy = min(max((90.0 - lat) * 4.0 - 0.5, 0.0), 719.0)
        xf = math.floor(fx)
        tx = fx - xf
        x0 = int(xf) % 1440
        x1 = (x0 + 1) % 1440
        y0 = int(math.floor(fy))
        y1 = min(y0 + 1, 719)
        ty = fy - y0
        r = self.r
        return ((1 - tx) * (1 - ty) * r[y0, x0] + tx * (1 - ty) * r[y0, x1]
                + (1 - tx) * ty * r[y1, x0] + tx * ty * r[y1, x1])


def smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


class Catalogue:
    """Every crater and pit, checked by brute force (no index)."""
    BODY_R = 3389500.0

    def __init__(self, crater_path, pit_path):
        def table(path, magic, stride):
            raw = open(path, "rb").read()
            m, n = struct.unpack_from("<II", raw, 0)
            if m != magic:
                raise ValueError("bad table " + path)
            return np.frombuffer(raw, dtype="<f4", offset=8, count=n * stride).reshape(n, stride).astype(np.float64)
        c = table(crater_path, 0x3152434D, 6)
        p = table(pit_path, 0x3156434D, 4)
        px = np.concatenate([c[:, 0], p[:, 0]])
        py = np.concatenate([c[:, 1], p[:, 1]])
        lon = np.radians(px / 106496.0 * 360.0 - 180.0)
        lat = np.radians(90.0 - py / 53248.0 * 180.0)
        self.dirs = np.stack([np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)], axis=1)
        self.radius = np.concatenate([c[:, 3], p[:, 2]]) * 200.0
        self.amount = np.concatenate([c[:, 4], p[:, 3]])
        self.flags = np.concatenate([c[:, 5].astype(np.int64) & 3, np.full(len(p), 4)])
        self.n_craters = len(c)
        self.stamped = []

    def stamp(self, d, min_diameter):
        d = np.array(d)
        ang = np.arctan2(np.linalg.norm(np.cross(self.dirs, d), axis=1), self.dirs @ d)
        rr = ang * self.BODY_R / self.radius
        use = (2.0 * self.radius >= min_diameter) & (rr < 1.5)
        rr, amt, fl = rr[use], self.amount[use], self.flags[use]
        pit = (fl & 4) != 0
        out = np.where(pit & (rr < 1.3), -amt * (1.0 - smoothstep(0.7, 1.15, rr)), 0.0)
        cx = (fl & 1) != 0
        tw = np.clip((rr - 0.45) / 0.55, 0.0, 1.0)
        dh = np.where(cx, -amt * (1.0 - tw * tw * (3.0 - 2.0 * tw)),
                      np.where(rr < 1.0, -amt * (1.0 - rr * rr), 0.0))
        dh = dh + np.where(cx & ((fl & 2) != 0), 0.35 * amt * np.exp(-(rr * rr) / 0.0324), 0.0)
        dh = dh + 0.28 * amt * np.exp(-((rr - 1.0) / 0.16) ** 2)
        crater = dh * (1.0 - smoothstep(1.1, 1.5, rr))
        return float(np.sum(np.where(pit, out, crater)))


def mars_rho(dem, areoid, cat, level, face, gi, gj, m, p, r_ref):
    dirs = list(subsample_dirs(face, gi, gj, m))
    k = max(0, 7 - level)
    h = math.nan
    while math.isnan(h) and k < 8:
        vals = [dem.sample(k, *lat_lon(d)) for d in dirs]
        vals = [v for v in vals if not math.isnan(v)]
        h = sum(vals) / len(vals) if vals else math.nan
        k += 1
    lat, lon = lat_lon(centre_dir(p, m))
    u = math.radians(90.0 - abs(lat)) * 3396190.0 / 12000.0
    if u < 1.0:
        w = 1.0 - u * u * (3.0 - 2.0 * u)
        h = h * (1.0 - w) + (dem.north if lat > 0 else dem.south) * w
    texel = math.pi / 2.0 * 3389500.0 / (208 << level)
    stamp = sum(cat.stamp(d, texel) for d in dirs) / 9.0
    if abs(stamp) > 1.0:
        cat.stamped.append(stamp)
    return h + stamp + areoid.radius(lat, lon) - r_ref


def dem_rho(dem, data_level, datum_radius, level, face, gi, gj, m, p, r_ref):
    """docs/rgp1.md for any body: Mars's method without craters."""
    dirs = list(subsample_dirs(face, gi, gj, m))
    k = max(0, data_level - level)
    h = math.nan
    while math.isnan(h) and k < len(dem.dims):
        vals = [dem.sample(k, *lat_lon(d)) for d in dirs]
        vals = [v for v in vals if not math.isnan(v)]
        h = sum(vals) / len(vals) if vals else math.nan
        k += 1
    lat, lon = lat_lon(centre_dir(p, m))
    blend = min(12000.0, 0.02 * datum_radius)
    u = math.radians(90.0 - abs(lat)) * datum_radius / blend
    if u < 1.0:
        w = 1.0 - u * u * (3.0 - 2.0 * u)
        h = h * (1.0 - w) + (dem.north if lat > 0 else dem.south) * w
    return h + datum_radius - r_ref


def dir_to_face(d):
    best = max(range(6), key=lambda f: (sum(FACES[f][0][k] * d[k] for k in range(3)), -f))
    f, u, v = FACES[best]
    w = sum(f[k] * d[k] for k in range(3))
    s = math.atan(sum(u[k] * d[k] for k in range(3)) / w) / (math.pi / 4.0)
    t = math.atan(sum(v[k] * d[k] for k in range(3)) / w) / (math.pi / 4.0)
    return best, s, t


def feature_picks(pack, cat, count, rng):
    """Samples at and around catalogued craters and pits."""
    level = pack.max_level
    m = pack.n << level
    tiles = 1 << level
    picks = []
    pits = list(range(cat.n_craters, len(cat.radius)))
    craters = list(range(cat.n_craters))
    chosen = rng.sample(craters, count) + rng.sample(pits, min(len(pits), count // 3))
    for idx in chosen:
        face, s, t = dir_to_face(cat.dirs[idx])
        step = 2.0 / m
        reach = cat.radius[idx] / 3389500.0 / (math.pi / 4.0)
        for frac in (0.0, 0.6, 1.0):
            ds = frac * reach * math.cos(idx)
            dt = frac * reach * math.sin(idx)
            gi = min(max(int(round((s + ds + 1.0) / step)), 0), m)
            gj = min(max(int(round((t + dt + 1.0) / step)), 0), m)
            x = min(gi // pack.n, tiles - 1)
            y = min(gj // pack.n, tiles - 1)
            picks.append((make_key(face, level, x, y), gi - x * pack.n, gj - y * pack.n))
    return picks


# Phobos source.

class Icq:
    def __init__(self, path):
        with open(path) as f:
            q = int(f.readline())
            v = np.loadtxt(f).reshape(6, q + 1, q + 1, 3) * 1000.0
        a = v[:, :-1, :-1].reshape(-1, 3)
        b = v[:, :-1, 1:].reshape(-1, 3)
        c = v[:, 1:, 1:].reshape(-1, 3)
        d = v[:, 1:, :-1].reshape(-1, 3)
        self.v0 = np.concatenate([a, a])
        self.e1 = np.concatenate([b, c]) - self.v0
        self.e2 = np.concatenate([c, d]) - self.v0
        cen = self.v0 + (self.e1 + self.e2) / 3.0
        self.cdir = cen / np.linalg.norm(cen, axis=1)[:, None]

    def radius(self, d):
        d = np.array(d)
        near = self.cdir @ d > math.cos(math.radians(3.0))
        v0, e1, e2 = self.v0[near], self.e1[near], self.e2[near]
        pvec = np.cross(d, e2)
        det = np.einsum("ij,ij->i", e1, pvec)
        ok = np.abs(det) > 1e-12
        inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
        tvec = -v0
        u = np.einsum("ij,ij->i", tvec, pvec) * inv
        qvec = np.cross(tvec, e1)
        v = (qvec @ d) * inv
        t = np.einsum("ij,ij->i", e2, qvec) * inv
        hit = ok & (u >= -1e-9) & (u <= 1 + 1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9) & (t > 0)
        ts = np.sort(t[hit])
        distinct = [x for k, x in enumerate(ts) if k == 0 or x - ts[k - 1] > 1e-6]
        return (max(distinct) if distinct else math.nan), len(distinct)


# Checks.

def owner_value(pack, level, p, m):
    """Value of lattice point p from the tile that owns it."""
    face, gi, gj = canonical(p, m)
    tiles = 1 << level
    x = min(gi // pack.n, tiles - 1)
    y = min(gj // pack.n, tiles - 1)
    arr = pack.tile(make_key(face, level, x, y))
    return int(arr[gj - y * pack.n + pack.b, gi - x * pack.n + pack.b])


def pick_samples(pack, count, rng):
    """Random tiles and positions, plus poles, a corner and lon 180."""
    keys = [int(k) for k in pack.index["key"]]
    picks = []
    for _ in range(count):
        key = rng.choice(keys)
        picks.append((key, rng.randint(-pack.b, pack.n + pack.b), rng.randint(-pack.b, pack.n + pack.b)))
    top = pack.max_level
    half = (pack.n << top) // 2
    for face, gi, gj in ((4, half, half), (5, half, half), (1, half, half), (0, 0, 0), (2, 0, half)):
        x = min(gi // pack.n, (1 << top) - 1)
        y = min(gj // pack.n, (1 << top) - 1)
        picks.append((make_key(face, top, x, y), gi - x * pack.n, gj - y * pack.n))
    return picks


def run(pack, picks, expected):
    worst = 0.0
    bad = 0
    border_checked = 0
    tol = pack.step / 2.0 + 0.02
    for n, (key, i, j) in enumerate(picks):
        face, level, x, y = split_key(key)
        m = pack.n << level
        p = fold(face, x * pack.n + i, y * pack.n + j, m)
        got = int(pack.tile(key)[j + pack.b, i + pack.b])
        if got != owner_value(pack, level, p, m):
            bad += 1
            print("  SHARED MISMATCH key %x i %d j %d" % (key, i, j))
        border_checked += 1
        cface, cgi, cgj = canonical(p, m)
        want = expected(level, cface, cgi, cgj, m, p)
        err = abs(got * pack.step - want)
        worst = max(worst, err)
        if not err <= tol:
            bad += 1
            lat, lon = lat_lon(centre_dir(p, m))
            print("  VALUE MISMATCH L%d lat %.4f lon %.4f got %.3f want %.3f" % (level, lat, lon, got * pack.step, want))
        if (n + 1) % 100 == 0:
            print("  %d/%d samples, worst %.4f m" % (n + 1, len(picks), worst), flush=True)
    print("checked %d samples (%d shared-value checks): worst |pack - source| = %.4f m, tolerance %.4f m"
          % (len(picks), border_checked, worst, tol))
    return bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["mars", "areoid", "phobos", "dem"])
    ap.add_argument("pack")
    ap.add_argument("--dem")
    ap.add_argument("--raster")
    ap.add_argument("--width", type=int)
    ap.add_argument("--height", type=int)
    ap.add_argument("--datum-radius", type=float)
    ap.add_argument("--areoid")
    ap.add_argument("--icq")
    ap.add_argument("--craters")
    ap.add_argument("--pits")
    ap.add_argument("--features", type=int, default=60)
    ap.add_argument("--samples", type=int, default=400)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()

    pack = Pack(a.pack)
    print("%s layer %d: N=%d B=%d step=%g r_ref=%g codec=%d levels %d-%d, %d tiles"
          % (pack.body, pack.layer, pack.n, pack.b, pack.step, pack.r_ref, pack.codec,
             pack.min_level, pack.max_level, pack.count))
    print("attribution:", pack.attribution)
    keys = pack.index["key"]
    if np.any(keys[1:] <= keys[:-1]):
        print("FAIL: index not strictly sorted")
        return 1
    expect_count = sum(6 * 4 ** L for L in range(pack.min_level, pack.max_level + 1))
    if pack.count != expect_count:
        print("FAIL: %d tiles, expected %d" % (pack.count, expect_count))
        return 1

    rng = random.Random(a.seed)
    picks = pick_samples(pack, a.samples, rng)
    if a.mode == "mars":
        dem, areoid = MarsDem(a.dem), Areoid(a.areoid)
        cat = Catalogue(a.craters, a.pits)
        print("pole caps: north %.1f south %.1f; %d craters, %d pits"
              % (dem.north, dem.south, cat.n_craters, len(cat.radius) - cat.n_craters))
        picks += feature_picks(pack, cat, a.features, rng)
        expected = lambda L, f, gi, gj, m, p: mars_rho(dem, areoid, cat, L, f, gi, gj, m, p, pack.r_ref)
    elif a.mode == "dem":
        # the baker's choice: the level nearest the pixel size
        pixel = math.radians(360.0 / a.width)
        data_level = min(max(round(math.log2((math.pi / 2.0) / (pack.n * pixel))), 0), 24)
        dem = RasterDem(a.raster, a.width, a.height, mips=data_level)
        radius = a.datum_radius or pack.r_ref
        print("data level %d; pole caps: north %.1f south %.1f" % (data_level, dem.north, dem.south))
        expected = lambda L, f, gi, gj, m, p: dem_rho(dem, data_level, radius, L, f, gi, gj, m, p, pack.r_ref)
    elif a.mode == "areoid":
        areoid = Areoid(a.areoid)
        expected = lambda L, f, gi, gj, m, p: areoid.radius(*lat_lon(centre_dir(p, m))) - pack.r_ref
    else:
        icq = Icq(a.icq)
        def expected(L, f, gi, gj, m, p):
            rs = []
            for d in subsample_dirs(f, gi, gj, m):
                r, hits = icq.radius(d)
                if hits != 1:
                    print("  ray with %d crossings" % hits)
                if not math.isnan(r):
                    rs.append(r)
            return sum(rs) / len(rs) - pack.r_ref
    bad = run(pack, picks, expected)
    if a.mode == "mars":
        s = cat.stamped
        print("samples with catalogue relief over 1 m: %d (range %.1f to %.1f m)"
              % (len(s), min(s, default=0.0), max(s, default=0.0)))
    print("VERIFY_PASSED" if bad == 0 else "VERIFY_FAILED (%d)" % bad)
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
