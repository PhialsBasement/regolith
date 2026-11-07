#!/usr/bin/env python3
"""Any global DEM into a Regolith body.

Reads any elevation raster GDAL can open (GeoTIFF, PDS IMG,
JPEG 2000 and more), takes the body's radius from its coordinate
system, warps it onto the global grid regolith_bake reads, bakes
NAME_height.rgp and prints how to point a RegolithBody at it.

  make_body.py DEM --name vesta --out res://data/packs

Needs Python 3.10+ with rasterio and numpy:
  pip install rasterio numpy
"""
import argparse
import math
import os
import shutil
import subprocess
import sys

try:
    import numpy as np
    import rasterio
    from rasterio.crs import CRS
    from rasterio.enums import Resampling
    from rasterio.transform import from_bounds
    from rasterio.vrt import WarpedVRT
    from rasterio.warp import transform as warp_points
except ImportError:
    sys.exit("make_body: needs rasterio and numpy (pip install rasterio numpy)")

ROWS_PER_CHUNK = 256


def body_radius(crs):
    """The body's mean radius (m) from its coordinate system."""
    d = crs.to_dict()
    if "R" in d:
        return float(d["R"])
    a = d.get("a")
    b = d.get("b", a)
    if a is None:
        return None
    # a sphere of the ellipsoid's volume
    return float((a * a * b) ** (1.0 / 3.0))


def native_pixel_deg(src, radius):
    """The source's pixel size in degrees of arc at the equator."""
    if src.crs.is_geographic:
        return abs(src.transform.a)
    return math.degrees(abs(src.transform.a) / radius)


def longitude_origin(src):
    """-180 when the source runs from the antimeridian, else 0."""
    cx = (src.bounds.left + src.bounds.right) / 2.0
    cy = (src.bounds.bottom + src.bounds.top) / 2.0
    xs, _ = warp_points(src.crs, geographic(src.crs), [src.bounds.left + 1e-6, cx], [cy, cy])
    return 0.0 if xs[0] > -90.0 else -180.0


def geographic(crs):
    """The same body, in plain latitude and longitude."""
    d = crs.to_dict()
    keep = {k: d[k] for k in ("R", "a", "b", "rf", "ellps") if k in d}
    keep.update({"proj": "longlat", "no_defs": True})
    return CRS.from_dict(keep)


def warp_to_grid(src, width, origin, out_path, scale, add):
    """Writes the global grid: float32 LE, NaN missing, rows from
    +90, columns from -180. Returns the share of valid pixels."""
    height = width // 2
    dst_crs = geographic(src.crs)
    transform = from_bounds(origin, -90.0, origin + 360.0, 90.0, width, height)
    valid = 0
    with WarpedVRT(src, crs=dst_crs, transform=transform, width=width, height=height,
                   resampling=Resampling.bilinear, src_nodata=src.nodata, nodata=np.nan,
                   dtype="float32") as vrt, open(out_path, "wb") as out:
        for row in range(0, height, ROWS_PER_CHUNK):
            n = min(ROWS_PER_CHUNK, height - row)
            window = rasterio.windows.Window(0, row, width, n)
            block = vrt.read(1, window=window).astype(np.float32)
            block = block * np.float32(scale) + np.float32(add)
            if origin == 0.0:
                # 0..360 onto -180..180
                block = np.roll(block, width // 2, axis=1)
            valid += int(np.count_nonzero(~np.isnan(block)))
            block.astype("<f4").tofile(out)
            print("\r  warping %d%%" % (100 * (row + n) // height), end="", flush=True)
    print(flush=True)
    return valid / float(width * height)


def find_baker(given):
    if given:
        return given
    here = os.path.dirname(os.path.abspath(__file__))
    for rel in ("../build/regolith_bake", "../build-double/regolith_bake", "../build/regolith_bake.exe"):
        p = os.path.normpath(os.path.join(here, rel))
        if os.path.isfile(p):
            return p
    return shutil.which("regolith_bake")


def main():
    ap = argparse.ArgumentParser(description="Any global DEM into a Regolith body.")
    ap.add_argument("dem", help="the elevation raster (any format GDAL reads)")
    ap.add_argument("--name", required=True, help="body name: files are NAME_height.rgp")
    ap.add_argument("--out", default=".", help="output folder (default: here)")
    ap.add_argument("--radius", type=float, help="body radius (m); default: from the DEM")
    ap.add_argument("--values", choices=["auto", "height", "radius"], default="auto",
                    help="heights over the datum, or radii from the centre (default: guess)")
    ap.add_argument("--width", type=int, help="grid width in pixels (default: the DEM's own)")
    ap.add_argument("--scale", type=float, help="value scale (default: the DEM's, else 1)")
    ap.add_argument("--offset", type=float, help="value offset (default: the DEM's, else 0)")
    ap.add_argument("--attribution", help="credit stored in the pack (default: the file name)")
    ap.add_argument("--max-level", type=int, help="deepest level (default: the data's own)")
    ap.add_argument("--keep-grid", action="store_true", help="keep the intermediate grid")
    ap.add_argument("--baker", help="path to regolith_bake")
    a = ap.parse_args()

    baker = find_baker(a.baker)
    if not baker:
        sys.exit("make_body: regolith_bake not found; build Regolith or pass --baker")
    os.makedirs(a.out, exist_ok=True)

    with rasterio.open(a.dem) as src:
        radius = a.radius or (body_radius(src.crs) if src.crs else None)
        if not radius:
            sys.exit("make_body: the DEM has no coordinate system; pass --radius")
        pixel = native_pixel_deg(src, radius)
        width = a.width or int(round(360.0 / pixel))
        width += width % 2
        origin = longitude_origin(src)
        scale = a.scale if a.scale is not None else (src.scales[0] or 1.0)
        add = a.offset if a.offset is not None else (src.offsets[0] or 0.0)
        print("%s: %d x %d %s, radius %.1f km, %.1f m pixels" % (os.path.basename(a.dem), src.width,
              src.height, src.dtypes[0], radius / 1000.0, math.radians(pixel) * radius))
        if a.values == "auto":
            # radii sit near the radius; heights near zero
            sample = src.read(1, out_shape=(1, 64, 128), masked=True).astype(np.float64) * scale + add
            median = float(np.ma.median(sample))
            values = "radius" if median > 0.5 * radius else "height"
            kind = "radii from the centre" if values == "radius" else "heights over the datum"
            print("  values look like %s (median %.0f m)" % (kind, median))
        else:
            values = a.values
        if values == "radius":
            add -= radius
        grid = os.path.join(a.out, a.name + "_grid.f32")
        print("  global grid %d x %d" % (width, width // 2))
        coverage = warp_to_grid(src, width, origin, grid, scale, add)
    if coverage < 0.98:
        print("  warning: only %.1f%% of the globe has data. Small gaps fill from coarser data;"
              " wide ones bake flat at the datum radius." % (100 * coverage))

    cmd = [baker, "dem", "--name", a.name, "--raster", grid, "--width", str(width), "--height", str(width // 2),
           "--datum-radius", repr(radius), "--out", a.out,
           "--attribution", a.attribution or os.path.basename(a.dem)]
    if a.max_level is not None:
        cmd += ["--max-level", str(a.max_level)]
    print("baking:", " ".join(cmd), flush=True)
    done = subprocess.run(cmd).returncode
    if not a.keep_grid:
        os.remove(grid)
    if done != 0:
        sys.exit("make_body: regolith_bake failed")

    pack = os.path.join(a.out, a.name + "_height.rgp")
    print()
    print("Done: %s" % pack)
    print("In Godot, on a RegolithBody:")
    print('  pack_path = "<res:// path to>/%s_height.rgp"' % a.name)
    print("  (radius %.1f km is in the pack; no datum_path needed for a sphere)" % (radius / 1000.0))


if __name__ == "__main__":
    main()
