# Terrain synthesis: mors's detail on the cube-sphere

Regolith ports mors's terrain look from `scripts/planet_streamer.gd`,
`scripts/terra_fine.gd` and `mars_ground.gdshader`. Each mors pass runs at
the Regolith level whose spacing matches the grid mors ran it on.

| mors pass | mors grid | Regolith level (Mars) |
|---|---|---|
| FBM detail (26 m, 0.35x to 2.2x by slope), ridged dunes inside the dune mask | 200 m source px | L7, 200 m (`detail`) |
| Roughness band (1.6 m, x(1 + 6 grade)) | 50 m | L9, 50 m (`roughness`) |
| Sub-catalogue craters 250 m to 1 km | 12.5 m | L9 (band 0) |
| terra_fine relief: ripples, hummocks, strata | 12.5 m | L11, 12.5 m (`relief`) |
| terra_fine thermal, 40 steps | 12.5 m, padded | L11, padded by 48 samples (`thermal`) |
| Sub-catalogue craters 40 to 250 m | 12.5 m | L11 (band 1) |
| Borehole, 3.20 N 20.20 E | 200 m | data levels, analytic |
| Colour map: gravel, bedrock, ice, Viking darkness | per region | geology page per tile, RGBA8, quarter resolution |

Levels between and below these subdivide their parent, so every pass carries
down to the finest level.

## What changed on the way to a sphere

- **Noise is sampled in space.** mors's FastNoiseLite fields were 2D in
  equirect pixels. Here they are 3D at `direction * r_ref`, with the same
  seeds and the same wavelengths in metres.
  - terra_fine's gradient noise gets a 3D extension. The 2D ripple noise
    stays 2D, along wind axes taken in the local tangent plane.
  - The 16-bit hashed angles use tables: bit-identical, no trig.
- **No pole fade or folds.** mors faded noise near the poles because the
  equirect grid packs a whole row onto one point. A cube-sphere has no such
  point.
- **Craters are hashed per face.** Sub-catalogue crater cells are squares
  of face metres, keyed by face. Every crater therefore belongs to exactly
  one face, and samples on any side of an edge see the same craters.
- **Thermal gathers its own padding.** It runs on a window gathered from
  the neighbouring tiles' pre-thermal relief, folding across face edges,
  and the padded border is cropped away afterwards. This matches mors,
  which padded a region and cropped it.

## Seams

`tests/test_synth.cpp` checks every shared edge sample:

- **Within a face:** every pass agrees. The thermal level is bit-identical
  and the others differ by under 1 mm.
- **Across faces:** the one-sample border beyond a face edge is
  extrapolated, not shared. That border only feeds shading normals.
- **Cube corners:** the folded thermal window is distorted, so thermal
  differs at the 8 cube corners.

## Shading

The ground shader is mors's `mars_ground.gdshader` logic.

- **Texture coordinates** are face metres, not world XZ.
  `RegolithBody::push_texture_domains` computes in double, per node, the
  fractional texture coordinate and the hex-tiling lattice position mod 256
  (the hash period). Tiles and hex cells are therefore exact across node
  edges at any distance from the origin.
- **Noise lookups** use face metres mod 20 km. Their scales are rounded so
  that `scale * 20 km` is a whole number (0.00041 to 0.0004, 0.0009 to
  0.001, 0.0041 to 0.004, 0.00137 to 0.00125, 0.00709 to 0.007, golden
  ratio 1.618 to 1.6), so they tile with no seam.
