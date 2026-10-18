# Regolith

True-scale planetary terrain for Godot 4.7, as a C++ GDExtension. Every
body is a cube-sphere displaced by its real elevation data, from orbit down
to the ground. See `TERRAIN_PLAN.md` in the morsified project for the
design.

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
