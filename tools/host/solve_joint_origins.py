#!/usr/bin/env python3
"""
solve_joint_origins -- work out which transform actually yields a joint's world
position, by testing candidates against the real geometry.

The origin formula inherited from fusion360-urdf-ros2 only holds for joints near
the base; further out it drifts by hundreds of millimetres. Rather than guess at
the Fusion API's coordinate conventions, urdf_extract records the raw inputs and
this scores every plausible combination against the one thing we can measure:
a revolute joint's axis must pass through both the parts it connects, so its
origin should sit on or very near both meshes.

    python solve_joint_origins.py arm_assembly_model.json -p out/arm_assembly_description
"""

import argparse
import json
import math
import os
import struct


def stl_bounds(path):
    with open(path, 'rb') as f:
        head = f.read(84)
        if len(head) < 84:
            return None
        count = struct.unpack('<I', head[80:84])[0]
        if os.path.getsize(path) != 84 + count * 50:
            return None
        lo = [1e9] * 3
        hi = [-1e9] * 3
        for _ in range(count):
            tri = f.read(50)
            if len(tri) < 50:
                break
            v = struct.unpack('<12f', tri[:48])
            for corner in range(3):
                for axis in range(3):
                    c = v[3 + corner * 3 + axis]
                    lo[axis] = min(lo[axis], c)
                    hi[axis] = max(hi[axis], c)
        return lo, hi


def apply(matrix, point_cm):
    """Row-major 4x4 (Fusion cm) applied to a point in cm, returning metres."""
    r = [[matrix[0], matrix[1], matrix[2]],
         [matrix[4], matrix[5], matrix[6]],
         [matrix[8], matrix[9], matrix[10]]]
    t = [matrix[3], matrix[7], matrix[11]]
    return [(sum(r[a][c] * point_cm[c] for c in range(3)) + t[a]) / 100.0
            for a in range(3)]


def world_aabb(link, pkg_root):
    # A JSON-only re-extract carries no mesh paths, so fall back to the STLs
    # already sitting in the built package -- they are what we measure against.
    rel = link.get('mesh') or os.path.join('meshes', link['name'] + '.stl')
    path = os.path.join(pkg_root, rel)
    if not os.path.isfile(path):
        return None
    bounds = stl_bounds(path)
    matrix = link.get('world_transform')
    if not bounds or not matrix:
        return None
    lo, hi = bounds
    wlo = [1e9] * 3
    whi = [-1e9] * 3
    for i in (0, 1):
        for j in (0, 1):
            for k in (0, 1):
                # STL is in mm; apply() wants cm.
                corner = [(lo[0] if i == 0 else hi[0]) / 10.0,
                          (lo[1] if j == 0 else hi[1]) / 10.0,
                          (lo[2] if k == 0 else hi[2]) / 10.0]
                w = apply(matrix, corner)
                for a in range(3):
                    wlo[a] = min(wlo[a], w[a])
                    whi[a] = max(whi[a], w[a])
    return wlo, whi


def distance_to_box(point, box):
    lo, hi = box
    return math.sqrt(sum(max(lo[a] - point[a], 0.0, point[a] - hi[a]) ** 2
                         for a in range(3)))


def candidates(raw):
    """Every plausible reading of where a joint origin lives."""
    out = {}
    g1, g2 = raw.get('geometry_one'), raw.get('geometry_two')
    o1w, o2w = raw.get('occ_one_world'), raw.get('occ_two_world')
    o1l, o2l = raw.get('occ_one_local'), raw.get('occ_two_local')

    if g1:
        out['one_raw'] = [v / 100.0 for v in g1]
        if o1w:
            out['occ1_world * one'] = apply(o1w, g1)
        if o1l:
            out['occ1_local * one'] = apply(o1l, g1)
    if g2:
        out['two_raw'] = [v / 100.0 for v in g2]
        if o2w:
            out['occ2_world * two'] = apply(o2w, g2)
        if o2l:
            out['occ2_local * two'] = apply(o2l, g2)
        if o1w:
            out['occ1_world * two'] = apply(o1w, g2)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('model')
    ap.add_argument('-p', '--pkg-root', required=True,
                    help='built package directory (for the meshes)')
    args = ap.parse_args()

    with open(args.model) as f:
        model = json.load(f)

    links = {l['name']: l for l in model['links']}
    boxes = {n: world_aabb(l, args.pkg_root) for n, l in links.items()}

    scores = {}
    print('%-14s %-22s %9s %9s' % ('joint', 'candidate', 'd(parent)', 'd(child)'))
    print('-' * 62)
    for joint in model['joints']:
        raw = joint.get('raw')
        if not raw:
            continue
        pb, cb = boxes.get(joint['parent']), boxes.get(joint['child'])
        if not pb or not cb:
            continue
        print(joint['fusion_name'])
        for name, point in sorted(candidates(raw).items()):
            dp, dc = distance_to_box(point, pb), distance_to_box(point, cb)
            scores.setdefault(name, []).append(max(dp, dc))
            print('%-14s %-22s %8.0fmm %8.0fmm'
                  % ('', name, dp * 1000, dc * 1000))
        print()

    if not scores:
        print('No joint carries a "raw" block -- re-extract with the current '
              'urdf_extract.py.')
        return 1

    print('RANKING (worst-case distance to either part, averaged over joints)')
    print('-' * 62)
    ranked = sorted(scores.items(), key=lambda kv: sum(kv[1]) / len(kv[1]))
    for name, values in ranked:
        print('  %-24s mean %7.1f mm   worst %7.1f mm'
              % (name, 1000 * sum(values) / len(values), 1000 * max(values)))
    print()
    print('Best: %s' % ranked[0][0])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
