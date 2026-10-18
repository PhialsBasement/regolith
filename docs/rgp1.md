# RGP1 pack format and bake rules

This file is the contract between `regolith_bake` (C++) and
`tools/verify_pack.py` (Python). The two share no code: each implements
these rules on its own, so a bug in one shows up as a mismatch.

All integers are little-endian.

## Geometry

- **Body frame:** IAU body-fixed. +Z is the north pole, +X the prime
  meridian, +Y 90 degrees east.
- **Faces:** in the order +X, -X, +Y, -Y, +Z, -Z, with axes `(f, u, v)`:

| Face | f | u | v |
|---|---|---|---|
| 0 +X | ( 1, 0, 0) | ( 0, 1, 0) | ( 0, 0, 1) |
| 1 -X | (-1, 0, 0) | ( 0,-1, 0) | ( 0, 0, 1) |
| 2 +Y | ( 0, 1, 0) | (-1, 0, 0) | ( 0, 0, 1) |
| 3 -Y | ( 0,-1, 0) | ( 1, 0, 0) | ( 0, 0, 1) |
| 4 +Z | ( 0, 0, 1) | ( 0, 1, 0) | (-1, 0, 0) |
| 5 -Z | ( 0, 0,-1) | ( 0, 1, 0) | ( 1, 0, 0) |

- **Warp:** `warp(c) = tan(pi/4 * c)`, except that `warp(+-1) = +-1`
  exactly.
- **Continuous direction:**
  `face_dir(face, s, t) = normalize(f + u*warp(s) + v*warp(t))`.
- **Lattice:** a level-L tile grid has `m = N * 2^L` intervals per face
  edge, where N is the tile's interval count. A face sample `(gi, gj)`
  has integer lattice point `P = f*m + u*(2gi - m) + v*(2gj - m)`.
- **Fold:** samples past a face edge (gi or gj outside `[0, m]`) fold onto
  the neighbour face:
  1. Start with `cu = 2gi - m`, `cv = 2gj - m`, `cf = m`.
  2. If `cu > m`: `cf -= cu - m` and `cu = m`. If `cu < -m`:
     `cf -= -m - cu` and `cu = -m`. Do the same for `cv`.
  3. `P = f*cf + u*cu + v*cv`.
- **Canonical sample:** the lowest-index face with `dot(face.f, P) == m`,
  with `gi = (dot(u, P) + m)/2` and `gj = (dot(v, P) + m)/2`. Every value
  is computed from the canonical sample, so shared edges, corners and
  borders are identical in every tile that holds them.
- **Centre direction of P:** `normalize(warp(Px/m), warp(Py/m),
  warp(Pz/m))`, summed in x, y, z order.
- **Latitude and longitude:** in degrees.
  `lat = atan2(z, hypot(x, y))` and `lon = atan2(y, x)`.

## Tiles

- **Size:** a tile holds `(N + 1 + 2B)^2` samples, with tile-local
  `i, j` in `[-B, N+B]`. Sample `(i, j)` of tile `(face, L, x, y)` is face
  sample `(x*N + i, y*N + j)`, folded if needed.
- **Storage order:** row-major, j outer and i inner. Index is
  `(j + B)*side + (i + B)`.
- **Values:** int32 `v = round(rho / step)`, where rho is in metres. Ties
  round away from zero. `step` is global to the pack.

## Height value (layer 0)

`rho = radius - r_ref`, in metres.

### Supersampling

Each sample averages a 3 x 3 subsample grid at plus or minus 1/3 texel,
taken on the canonical face. With `a, b` in {-1, 0, 1}:

```
s = ((2gi - m) + (2/3)*a) / m
t = ((2gj - m) + (2/3)*b) / m
```

The subsample direction is `face_dir(face, s, t)`, where s and t may run
slightly past +-1.

The value is the mean of the subsamples that are valid. If none are
valid, Mars retries at the next coarser mip.

### Mars

**DEM sampling.** Source is `Mars_HRSC_MOLA_BlendDEM_Global_200mp_v2.tif`:
106694 x 53347 int16, NODATA -32768, pixel size
D = 0.003374120830641 degrees, PixelIsArea, top-left corner at
(-180, 90).

- **Mip pyramid:** mip 0 is the raster. Mip k+1 pixel `(x, y)` is the mean
  of the valid mip-k pixels `(2x..2x+1, 2y..2y+1)` that exist. Its size is
  `ceil(W/2) x ceil(H/2)` and its pixel size is `2*D_k`.
- **Mip choice:** level L reads mip `k = 7 - L`, clamped at 0 or more.
  NODATA falls back to `k + 1`, then further up.
- **Bilinear sample at mip k:**
  1. `fx = (lon + 180)/D_k - 0.5` and `fy = (90 - lat)/D_k - 0.5`.
  2. Period `p = 360/D_k`. Bring fx into `[0, p)`.
  3. If `fx <= W_k - 1`: `x0 = floor(fx)`, `x1 = x0 + 1`, `tx = fx - x0`.
     Otherwise `x0 = W_k - 1`, `x1 = 0`,
     `tx = (fx - (W_k - 1)) / (p - (W_k - 1))`. This covers the raster's
     sub-pixel gap at the antimeridian.
  4. Clamp fy to `[0, H_k - 1]`. Then `y0 = floor(fy)`,
     `y1 = min(y0 + 1, H_k - 1)`, `ty = fy - y0`.
  5. NODATA taps are dropped and the remaining weights renormalised.

**Pole caps.**
- The cap height is the mean of the valid pixels in raster row 0 (north)
  and row H-1 (south).
- With `u = colat_m / 12000`, where `colat_m = (90 - |lat|)` in radians
  times 3396190, the cap weight inside `u < 1` is `w = 1 - u^2(3 - 2u)`.
- `h = h*(1 - w) + cap*w`, using the lat of the centre direction.

**Catalogue stamps.** These come after the pole caps. The stamp term is
the mean over the 9 subsample directions of the summed feature profiles.

- **Tables:** mors's `crater_restore.bin` and `cave_sites.bin`.
  - `crater_restore.bin` is "MCR1": u32 0x3152434D, u32 count, then count
    x 6 f32 records `px, py, rx, ry, deepen, flags`.
  - `cave_sites.bin` is u32 0x3156434D, u32 count, then count x 4 f32
    records `px, py, r, depth`.
- **Position:** `lon = px/106496*360 - 180` and `lat = 90 - py/53248*180`.
- **Radius:** `ry*200 m` for craters and `r*200 m` for pits.
- **Distance:** `dist = 3389500 * atan2(|c x d|, c . d)`, where c is the
  feature direction and d the subsample direction.
- **Inclusion:** a feature counts only if its diameter is at least
  `texel_L = pi/2 * 3389500 / (208 * 2^L)`.

With `rr = dist/radius`, `smoothstep(e0, e1, x) = t^2(3 - 2t)` and
`t = clamp((x - e0)/(e1 - e0), 0, 1)`:

- **Pit** (where `rr < 1.3`): `-depth * (1 - smoothstep(0.7, 1.15, rr))`.
- **Crater** (where `rr < 1.5`), with `D = deepen`:
  - Complex (flag 1): `tw = clamp((rr - 0.45)/0.55, 0, 1)` and
    `dh = -D*(1 - tw^2(3 - 2tw))`. With a peak (flag 2), add
    `0.35*D*exp(-rr^2/0.0324)`.
  - Simple: `dh = -D*(1 - rr^2)` for `rr < 1`, and 0 otherwise.
  - Rim, for both: add `0.28*D*exp(-((rr - 1)/0.16)^2)`.
  - The result is `dh * (1 - smoothstep(1.1, 1.5, rr))`.

**Areoid.** `rho = h + (A - r_ref)`.
- A is the areoid radius, bilinear in `mega90n000cb.img` at the centre
  direction.
- That file is 1440 x 720 big-endian int16. Radius is value + 3396000 m.
- Pixel `(c, r)` has its centre at `lon = (c + 0.5)/4` (0 to 360 east) and
  `lat = 90 - (r + 0.5)/4`.
- Longitude is periodic (1440 columns) and latitude is clamped.

**Constants:** `r_ref = 3389500`, N = 208, B = 2, levels 0 to 7.

### Phobos

- **Source:** the Gaskell ICQ q=512 shape, in km.
  - Line 1 is q. Then come 6 faces of `(q+1)^2` vertices, with i fastest.
  - Each face's grid quads `(i,j) (i+1,j) (i+1,j+1) (i,j+1)` split into
    triangles `(i,j)(i+1,j)(i+1,j+1)` and `(i,j)(i+1,j+1)(i,j+1)`.
- **Radius:** a subsample's radius is the distance along the ray from the
  origin to its intersection with the mesh. The shape is star-shaped, so
  each ray has one intersection.
- **Constants:** `rho = radius_m - r_ref`, with `r_ref = 11080`, N = 256,
  B = 2, levels 0 to 1.

## Areoid value (layer 1)

`rho = A - r_ref` at the centre direction, without supersampling. Mars
only, levels 0 to 3.

## File layout

### Header (256 bytes)

| Offset | Type | Field |
|---|---|---|
| 0 | char[4] | "RGP1" |
| 4 | u32 | version = 1 |
| 8 | u32 | header size = 256 |
| 12 | u32 | layer (0 height, 1 areoid) |
| 16 | char[32] | body name, NUL padded |
| 48 | u32 | N (intervals) |
| 52 | u32 | B (border) |
| 56 | u32 | projection = 1 (this document) |
| 60 | u32 | sample format = 1 (int32 times step) |
| 64 | f64 | step, metres |
| 72 | f64 | r_ref, metres |
| 80 | u32 | codec |
| 84 | u32 | codec level (informational) |
| 88 | u32 | min level |
| 92 | u32 | max level |
| 96 | u64 | tile count |
| 104 | u64 | index offset |
| 112 | u64 | attribution offset |
| 120 | u32 | attribution size |

The remaining bytes up to 256 are zero.

### Index

`tile count` entries of 32 bytes, sorted by key:

| Type | Field |
|---|---|
| u64 | key = face<<61 \| level<<56 \| x<<28 \| y |
| u64 | blob offset |
| u32 | blob size |
| u32 | CRC-32 (ISO-HDLC, as zlib) of the blob |
| f32 | min rho of the tile's interior `[0, N]^2` |
| f32 | max rho of the tile's interior `[0, N]^2` |

### Blob (codecs)

1. Predict each value from its decoded neighbours: a = left, b = up,
   c = up-left.
   - **Codec 1 (row delta):** `pred = a`. In column 0, `pred = b`. At
     (0, 0), `pred = 0`.
   - **Codecs 2 and 3 (MED):** in row 0, `pred = a`, or 0 at (0, 0). In
     column 0, `pred = b`. Otherwise, if `c >= max(a, b)` then
     `pred = min(a, b)`; else if `c <= min(a, b)` then `pred = max(a, b)`;
     else `pred = a + b - c`.
2. `e = v - pred` (int32), then zigzag: `z = (e << 1) ^ (e >> 31)`.
3. Codecs 1 and 2 split z into 4 byte planes, low byte first: plane k
   holds byte k of every sample. Codec 3 keeps z as u32 little-endian.
4. zstd frame.
