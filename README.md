# Regolith

True-scale planetary terrain for Godot 4.7, as a C++ GDExtension. Every
body is a cube-sphere displaced by its real elevation data, from orbit down
to the ground. See `TERRAIN_PLAN.md` in the morsified project for the
design.

## The engine

Regolith is made for Godot 4.7.2 with a few renderer patches and
double precision. `engine/build_godot.sh` (or `build_godot.ps1` on
Windows) builds that Godot, editor and export templates. On a stock
Godot it still runs, slowly, and warns you. See `engine/README.md`.

## Bodies from DEMs

`tools/make_body.py` turns any global elevation raster GDAL can read
(GeoTIFF, PDS IMG, JPEG 2000 and more) into a body:

```
pip install rasterio numpy
tools/make_body.py Vesta_Dawn_HAMO_DTM_DLR_Global_48ppd.tif --name vesta --out data/packs
```

It takes the radius from the DEM's coordinate system, tells heights
from radii, handles projected or geographic grids and 0..360 or
-180..180 longitudes, warps in chunks (rasters bigger than memory
are fine), and bakes `NAME_height.rgp` down to the level the data
supports. Point a `RegolithBody`'s `pack_path` at it.

Underneath is `regolith_bake dem`, which reads a plain global float32
grid and also takes a datum raster (an areoid or selenoid) for
bodies whose heights are over one. `tools/verify_pack.py dem`
checks a baked pack against its grid independently.

## Sky and atmosphere

`RegolithAtmosphere` draws the sky, the haze and the light as one
physically based model (Hillaire 2020): no sky shader, no painted
colours. Set `air` to a preset (`Mars`, `Earth`, `None` for an
airless body) or to `Custom` and give the air's gas (Rayleigh),
aerosol (Mie) and ozone coefficients per km. From those it
renders, as a compositor pass costing about 0.25 ms:

- the sky from anywhere: the ground, the stratosphere, orbit;
- aerial perspective on all geometry (Godot's fog is switched off);
- the sun's colour and strength through the air, and the ambient
  light from the sky and sunlit ground, set on the scene each frame;
- the night sky: the 9,096 stars of the Yale Bright Star Catalogue
  at their true positions, brightness and colour, and NASA's
  Milky Way, on a dark-adapting eye (`night_vision` caps it);
- the planets, the Moon and any moons on orbit tables (`moons`) as
  lit disks with their real phases, or points when unresolved.

Mars's dust follows `dust_tau`, which `RegolithWeather` sets. Ask
`get_sky_direction("Earth")` where something is in the sky.

Data: Yale Bright Star Catalogue, 5th ed. (Hoffleit and Warren,
1991). Milky Way: NASA/Goddard Space Flight Center Scientific
Visualization Studio, Deep Star Maps 2020. `tools/make_sky_data.gd`
rebuilds `regolith_stars.res` and `regolith_milkyway.res` from them.

## Bodies

Each `RegolithBody` takes a `preset` (Mercury to Neptune, the Moon,
Phobos, Deimos, Vesta, Ceres) filling its rotation, orbit and
gravity, all of which can then be set by hand.

## Layout

- `core/`: maths, projection, tile keys, frames. No Godot dependency;
  shared by the extension and the baker.
- `gdext/`: the Godot extension.
- `tests/`: core unit tests.
- `addon/`: the `.gdextension` file, copied next to the built library.
- `thirdparty/godot-cpp/`: submodule, pinned to `10.0.0-stable` (Godot
  4.7 API).

## Build (Linux)

```
git submodule update --init
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 16
./build/regolith_core_tests
```

The library and `regolith.gdextension` land in `../addons/regolith/`
(override with `-DREGOLITH_ADDON_DIR=...`).
