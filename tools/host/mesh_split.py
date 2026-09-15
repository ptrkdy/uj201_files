#!/usr/bin/env python3
"""
mesh_split -- cut a link's mesh with a plane and derive each half's mass properties.

A body carrying two joints cannot exist in a URDF tree. Splitting it in CAD is
the honest fix, but the cut is a plane through a triangle soup, so it can be
done exactly here instead: clip the STL, then integrate mass, centre of mass and
the inertia tensor over each half.

Straddling triangles are clipped on the plane rather than assigned whole, so the
cut is exact and each half is closed except for a flat opening. Putting the
integration origin ON the cut plane makes that opening free: every tetrahedron
formed from an apex on the plane with a cap triangle is degenerate and
contributes nothing, so volume, centroid and inertia all come out exact without
having to triangulate the cap.

Self-check: integrating an unsplit mesh must reproduce what the CAD package
reported for the same body. verify() does exactly that.
"""

import math
import struct


# --------------------------------------------------------------------------
# STL

def read_stl(path):
    """Triangles as [(v0, v1, v2), ...] in the file's own units (mm)."""
    with open(path, 'rb') as f:
        data = f.read()
    if len(data) < 84:
        raise ValueError('%s is too short to be an STL' % path)
    count = struct.unpack('<I', data[80:84])[0]
    if len(data) != 84 + count * 50:
        return read_ascii_stl(path)
    tris = []
    for i in range(count):
        off = 84 + i * 50
        v = struct.unpack('<12f', data[off:off + 48])
        tris.append((v[3:6], v[6:9], v[9:12]))
    return tris


def read_ascii_stl(path):
    tris, current = [], []
    with open(path, 'r', errors='ignore') as f:
        for line in f:
            parts = line.split()
            if len(parts) == 4 and parts[0] == 'vertex':
                current.append(tuple(float(x) for x in parts[1:]))
                if len(current) == 3:
                    tris.append(tuple(current))
                    current = []
    return tris


def write_stl(path, tris):
    out = bytearray(b'\0' * 80)
    out += struct.pack('<I', len(tris))
    for a, b, c in tris:
        n = normal(a, b, c)
        out += struct.pack('<12fH', n[0], n[1], n[2],
                           a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], 0)
    with open(path, 'wb') as f:
        f.write(bytes(out))


# --------------------------------------------------------------------------
# vector helpers

def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def normalise(a):
    n = math.sqrt(dot(a, a))
    return scale(a, 1.0 / n) if n > 1e-12 else (0.0, 0.0, 1.0)


def normal(a, b, c):
    return normalise(cross(sub(b, a), sub(c, a)))


# --------------------------------------------------------------------------
# clipping

def clip(tris, point, unit_normal):
    """Split triangles into (negative side, positive side) of a plane.

    Triangles crossing the plane are cut on it, so neither half inherits
    geometry from the other.
    """
    negative, positive = [], []

    def signed(v):
        return dot(sub(v, point), unit_normal)

    def halfspace(poly, keep_positive):
        """Sutherland-Hodgman against one side of the plane."""
        out = []
        n = len(poly)
        for i in range(n):
            cur, nxt = poly[i], poly[(i + 1) % n]
            dc, dn = signed(cur), signed(nxt)
            cur_in = dc >= 0 if keep_positive else dc <= 0
            nxt_in = dn >= 0 if keep_positive else dn <= 0
            if cur_in:
                out.append(cur)
            if cur_in != nxt_in and abs(dn - dc) > 1e-12:
                t = dc / (dc - dn)
                out.append(tuple(cur[k] + (nxt[k] - cur[k]) * t for k in range(3)))
        return out

    for tri in tris:
        d = [signed(v) for v in tri]
        if all(x >= -1e-9 for x in d):
            positive.append(tri)
            continue
        if all(x <= 1e-9 for x in d):
            negative.append(tri)
            continue
        for keep_positive, bucket in ((True, positive), (False, negative)):
            poly = halfspace(list(tri), keep_positive)
            # Fan-triangulate the 3- or 4-sided remainder.
            for i in range(1, len(poly) - 1):
                bucket.append((poly[0], poly[i], poly[i + 1]))
    return negative, positive


# --------------------------------------------------------------------------
# mass properties

# Second moment of the canonical tetrahedron (0, e1, e2, e3).
_CANON = ((2.0, 1.0, 1.0), (1.0, 2.0, 1.0), (1.0, 1.0, 2.0))


def mass_properties(tris, origin, unit_scale=0.001):
    """Volume, centroid and inertia of a closed (or plane-capped) triangle set.

    `origin` must lie on the cut plane for a clipped half, so the missing cap
    integrates to zero. Coordinates are scaled by `unit_scale` (mm -> m).
    Returns volume in m^3, centroid in m relative to `origin`, and the inertia
    tensor about that centroid for unit density.
    """
    volume = 0.0
    centroid = [0.0, 0.0, 0.0]
    cov = [[0.0] * 3 for _ in range(3)]

    for tri in tris:
        a, b, c = (scale(sub(v, origin), unit_scale) for v in tri)
        det = dot(a, cross(b, c))          # 6 * signed tet volume
        if det == 0.0:
            continue
        volume += det / 6.0
        for k in range(3):
            centroid[k] += (a[k] + b[k] + c[k]) * det / 24.0

        # Second moment: det * A * C_canonical * A^T, with A = [a b c].
        cols = (a, b, c)
        for i in range(3):
            for j in range(3):
                s = 0.0
                for p in range(3):
                    for q in range(3):
                        s += _CANON[p][q] * cols[p][i] * cols[q][j]
                cov[i][j] += det * s / 120.0

    if abs(volume) < 1e-15:
        return 0.0, (0.0, 0.0, 0.0), [0.0] * 6

    centroid = [v / volume for v in centroid]

    # Inertia about the integration origin, then shift to the centroid.
    trace = cov[0][0] + cov[1][1] + cov[2][2]
    inertia = [[(trace if i == j else 0.0) - cov[i][j] for j in range(3)]
               for i in range(3)]
    cx, cy, cz = centroid
    shift = (((cy * cy + cz * cz), -cx * cy, -cx * cz),
             (-cx * cy, (cx * cx + cz * cz), -cy * cz),
             (-cx * cz, -cy * cz, (cx * cx + cy * cy)))
    for i in range(3):
        for j in range(3):
            inertia[i][j] -= volume * shift[i][j]

    # URDF order: ixx iyy izz ixy iyz ixz
    return volume, tuple(centroid), [inertia[0][0], inertia[1][1], inertia[2][2],
                                     inertia[0][1], inertia[1][2], inertia[0][2]]


def verify(path, reported_mass, reported_inertia):
    """Check the integrator against values the CAD package reported."""
    tris = read_stl(path)
    volume, centroid, inertia = mass_properties(tris, (0.0, 0.0, 0.0))
    if volume <= 0:
        return None
    density = reported_mass / volume
    scaled = [v * density for v in inertia]
    errors = [abs(s - r) / max(abs(r), 1e-9) for s, r in zip(scaled, reported_inertia)]
    return {
        'volume_m3': volume,
        'density': density,
        'centroid': centroid,
        'inertia_computed': scaled,
        'inertia_reported': list(reported_inertia),
        'worst_relative_error': max(errors) if errors else 0.0,
    }


# --------------------------------------------------------------------------
# pre-split meshes

def connected_shells(tris, tol=1e-4):
    """Group triangles into connected components by shared (quantised) vertices.

    A CAD package asked to split a body and export both halves writes them into
    one STL as disconnected shells. This recovers them as separate parts, which
    beats any plane cut: the parting surface is whatever the CAD actually used,
    interlocking geometry included.
    """
    parent = {}

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    def key(v):
        return (round(v[0] / tol), round(v[1] / tol), round(v[2] / tol))

    for tri in tris:
        ks = [key(v) for v in tri]
        for k in ks:
            parent.setdefault(k, k)
        union(ks[0], ks[1])
        union(ks[1], ks[2])

    groups = {}
    for tri in tris:
        groups.setdefault(find(key(tri[0])), []).append(tri)
    return sorted(groups.values(), key=len, reverse=True)


def solve_translation(tris, reference_vertices, samples=300, places=2):
    """Find the translation putting `tris` back onto the reference geometry.

    Exported halves are often moved apart for clarity, which loses their place
    in the parent's frame. Since a split preserves the original surfaces, the
    true offset is the one that lands the most vertices exactly on the
    reference; spurious offsets from repeated features (hole patterns) never
    accumulate as many votes.

    Returns (translation, fraction_of_vertices_matched).
    """
    verts = sorted({(round(p[0], 3), round(p[1], 3), round(p[2], 3))
                    for tri in tris for p in tri})
    if not verts or not reference_vertices:
        return (0.0, 0.0, 0.0), 0.0

    step = max(1, len(verts) // samples)
    votes = {}
    for v in verts[::step]:
        for o in reference_vertices:
            d = (round(o[0] - v[0], places),
                 round(o[1] - v[1], places),
                 round(o[2] - v[2], places))
            votes[d] = votes.get(d, 0) + 1

    best, score = (0.0, 0.0, 0.0), 0.0
    for d, _ in sorted(votes.items(), key=lambda kv: -kv[1])[:8]:
        moved = {(round(p[0] + d[0], 3), round(p[1] + d[1], 3), round(p[2] + d[2], 3))
                 for tri in tris for p in tri}
        hit = len(moved & reference_vertices) / float(len(moved))
        if hit > score:
            best, score = d, hit
    return best, score


def translate(tris, delta):
    return [tuple(tuple(p[a] + delta[a] for a in range(3)) for p in tri)
            for tri in tris]


def vertex_set(tris):
    return {(round(p[0], 3), round(p[1], 3), round(p[2], 3))
            for tri in tris for p in tri}
