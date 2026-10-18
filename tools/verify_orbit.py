#!/usr/bin/env python3
"""Check an RGE1 orbit table against held-out JPL Horizons vectors.

Interpolates the table independently (cubic Hermite, as docs/rge1.md
specifies) at every Horizons epoch and reports the position error and
the angular error as seen from the parent's centre.

Usage: verify_orbit.py TABLE.rge HORIZONS.txt
"""
import math
import struct
import sys

import numpy as np


def load_table(path):
    raw = open(path, "rb").read()
    if raw[:4] != b"RGE1":
        raise ValueError("not an RGE1 table")
    count = struct.unpack_from("<I", raw, 8)[0]
    t0, step = struct.unpack_from("<dd", raw, 16)
    states = np.frombuffer(raw, dtype="<f8", offset=80, count=count * 6).reshape(count, 6)
    return t0, step, states


def load_horizons(path):
    rows, inside = [], False
    for line in open(path):
        if line.startswith("$$SOE"):
            inside = True
        elif line.startswith("$$EOE"):
            inside = False
        elif inside:
            f = line.split(",")
            jd = float(f[0])
            rows.append(((jd - 2451545.0) * 86400.0, [float(x) * 1000.0 for x in f[2:8]]))
    return rows


def hermite(t0, step, states, t):
    u = (t - t0) / step
    i = int(math.floor(u))
    s = u - i
    a, b = states[i], states[i + 1]
    h00 = 2 * s**3 - 3 * s**2 + 1
    h10 = s**3 - 2 * s**2 + s
    h01 = -2 * s**3 + 3 * s**2
    h11 = s**3 - s**2
    return h00 * a[:3] + h10 * step * a[3:] + h01 * b[:3] + h11 * step * b[3:]


def main():
    t0, step, states = load_table(sys.argv[1])
    rows = load_horizons(sys.argv[2])
    worst_m, worst_deg = 0.0, 0.0
    for t, truth in rows:
        got = hermite(t0, step, states, t)
        want = np.array(truth[:3])
        worst_m = max(worst_m, float(np.linalg.norm(got - want)))
        cosang = float(np.dot(got, want) / (np.linalg.norm(got) * np.linalg.norm(want)))
        worst_deg = max(worst_deg, math.degrees(math.acos(min(1.0, cosang))))
    print("%d epochs: worst position error %.2f m, worst direction error %.6f deg from Mars's centre"
          % (len(rows), worst_m, worst_deg))
    ok = worst_deg < 0.1
    print("VERIFY_PASSED" if ok else "VERIFY_FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
