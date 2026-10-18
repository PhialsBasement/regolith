# Level of detail: why there are no cracks

Selection follows CDLOD on each cube face's quadtree.

## Selection

- A node at level L has range `range(L) = k * edge(L)`, where
  `edge(L) = pi/2 * r_ref / 2^L`.
- Its vertices morph toward the parent grid between `m * range(L)` and
  `range(L)`, with `k = range_factor` and `m = morph_start`. Morphing moves
  them in position and blends them in height toward the parent page.
- A node splits into all four children when its bounding sphere comes
  within `range(L+1)` of the camera.

## No cracks between levels

Take a level-(L+1) node A next to a level-L node B. The shared edge is
crack-free when two things hold on it:

- every vertex of A is fully morphed (`morph_A = 1`), so A matches its
  parent's grid and heights;
- every vertex of B is unmorphed (`morph_B = 0`).

**A is fully morphed.** B was not split, so its whole sphere, and with it
the shared edge, lies at least `range(L+1)` from the camera. A's morph ends
exactly there.

**B is not morphing.** A's parent P was split, so P's sphere came within
`range(L+1)`. Every point of P, including the shared edge, is therefore at
most `range(L+1) + 2 * radius(P)` away. B starts morphing at
`m * range(L) = 2m * range(L+1)`. So we need

```
2m * range(L+1) > range(L+1) + 2 * radius(P)
```

which, with `range(L+1) = k/2 * edge(L)` and `2 * radius(P)` about
`1.5 * edge(L)` (diagonal plus relief), becomes `k * (2m - 1) > 3`.

The defaults are `k = 8` and `m = 0.8`, giving 4.8. That margin covers
steep relief, which inflates the bounding spheres.

## Testing

`tests/test_select.cpp` checks both conditions on every shared edge, for
random cameras from 2 m to 6,000 km, across face edges included.

## Streaming

If a node's children are not resident yet, the node is drawn whole. Its
neighbours may already be split, so a crack can show for a few frames.
Skirts hide that until the children arrive.
