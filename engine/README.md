# The Regolith engine build

Regolith runs on stock Godot 4.7.2, but slowly. Its terrain is made
for a Godot with a few renderer changes, and for double precision.
This folder builds that Godot from the official source.

## What the patches add

Applied in order to the `4.7.2-stable` tag:

| Patch | What it does |
|---|---|
| `0001` forest performance | GPU depth occlusion and shadow-receiver culling in the forward clustered renderer |
| `0002` forest switches | turns those on and off: `rendering/forest/gpu_occlusion`, `rendering/forest/shadow_receivers` |
| `0003` GPU profiling and terrain support | `motion_vectors_rigid` (a vertex shader runs once per frame instead of twice for motion vectors), `IN_DEPTH_PREPASS` (shaders skip shading in depth-only passes), and an opt-in GPU section profiler (`profile_clock`, `profile_add`; free unless `GODOT_SECTION_PROFILE` is set) |

Regolith's terrain shaders depend on all three. Together they're the
difference between a smooth frame rate and a slideshow.

## Build it

Needs git, Python 3, SCons and Godot's own build dependencies:
https://docs.godotengine.org/en/stable/engine_details/development/compiling/

Linux or macOS:

```
regolith/engine/build_godot.sh            # into regolith/engine/godot
regolith/engine/build_godot.sh ~/godot-regolith
```

Windows (PowerShell, with Godot's Windows build tools installed):

```
regolith\engine\build_godot.ps1
```

The script:

1. clones Godot `4.7.2-stable` (shallow), or reuses an existing checkout of that tag;
2. checks each patch applies, then applies it (already-applied patches are skipped, so it's safe to rerun);
3. builds the editor and the debug and release export templates with `precision=double`;
4. installs the templates where the editor looks for them (`export_templates/4.7.2.stable.double/`).

You need git, Python 3, SCons and Godot's usual build dependencies:
see [Compiling Godot](https://docs.godotengine.org/en/stable/contributing/development/compiling/).
A full build takes 20 minutes to an hour, depending on the machine.
Set `JOBS` to limit parallel compile jobs.

Then open your project with the editor it printed.

## On a stock Godot

Regolith still works: it loads slower versions of its terrain shaders
(`*_stock.gdshader`, generated from the same source at build time)
and says so loudly, with a dialog when the project opens, a red
notice in the 3D editor toolbar, and an error in the log every run.
It detects the patched engine by the defaults it registers, so a
project saved by the patched editor doesn't fool a stock one.
