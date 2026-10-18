# RGE1 orbit table

A satellite's state relative to its parent, sampled at a fixed step. It is
baked from JPL Horizons vectors by `regolith_bake orbit`. All values are
little-endian.

## Header (80 bytes)

| Offset | Type | Field |
|---|---|---|
| 0 | char[4] | "RGE1" |
| 4 | u32 | version = 1 |
| 8 | u32 | sample count |
| 12 | u32 | 0 |
| 16 | f64 | t0: TDB seconds since J2000 of the first sample |
| 24 | f64 | step, seconds |
| 32 | f64 | parent GM, m^3/s^2 |
| 40 | f64 | parent J2 |
| 48 | f64 | parent J2 reference radius, m |
| 56 | f64 | parent pole right ascension, degrees (ICRF) |
| 64 | f64 | parent pole declination, degrees (ICRF) |
| 72 | 8 bytes | zero |

## Samples

`count` records of six f64: position x, y, z (m) and velocity vx, vy, vz
(m/s), parent-centred, ICRF.

## Use

**Inside the table:** cubic Hermite on position and velocity between the
two neighbouring samples. At a 10-minute step for Phobos the error is
about 8 m.

**Outside the table:**
1. Two-body Kepler from the nearest end.
2. The node and periapsis drift at the J2 secular rates.
3. Elements are taken in the parent's equatorial frame, which is the
   frame J2 acts in.
