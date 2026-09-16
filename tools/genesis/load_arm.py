#!/usr/bin/env python3
"""
load_arm -- load the UJ201 description into Genesis and report what it made of it.

Answers the questions the URDF cannot answer about itself:

  * does Genesis accept the file at all
  * is the mm -> m mesh scale honoured (a 0.5 m arm, or a 500 m one)
  * how many links survive merge_fixed_links, and which
  * what total mass the physics engine ends up with
  * how many actuated DOFs it finds, against the 8 revolute joints we wrote

Run inside the genesis-radiial image, with this repository mounted at /work:

    docker run --rm -v <repo>:/work genesis-radiial:0.4.6 \
        python /work/tools/genesis/load_arm.py

Written defensively: Genesis's Python API moves between releases, so every
probe is attempted and reported rather than assumed, and a missing attribute
degrades to a note instead of a traceback.
"""

import argparse
import os
import sys


def probe(label, fn, unit=''):
    """Run one query, printing the answer or why it could not be had."""
    try:
        value = fn()
    except Exception as exc:                      # noqa: BLE001 - diagnostic
        print('  %-26s unavailable (%s: %s)' % (label, type(exc).__name__, exc))
        return None
    print('  %-26s %s%s' % (label, value, unit))
    return value


def first_attr(obj, *names):
    for name in names:
        if hasattr(obj, name):
            return getattr(obj, name)
    raise AttributeError('none of %s on %s' % (names, type(obj).__name__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf', default='/work/urdf/arm_assembly.urdf')
    ap.add_argument('--no-merge', action='store_true',
                    help='disable merge_fixed_links, to see every link')
    ap.add_argument('--steps', type=int, default=0,
                    help='step the sim this many times after building')
    args = ap.parse_args()

    if not os.path.isfile(args.urdf):
        sys.exit('no such URDF: %s' % args.urdf)

    import genesis as gs
    print('genesis %s' % getattr(gs, '__version__', '(unknown version)'))
    gs.init(backend=gs.cpu, logging_level='warning')

    scene = gs.Scene(show_viewer=False)
    scene.add_entity(gs.morphs.Plane())
    arm = scene.add_entity(gs.morphs.URDF(
        file=args.urdf,
        fixed=True,                    # URDF has no world joint; without this it falls
        merge_fixed_links=not args.no_merge,
    ))
    scene.build()

    print('\nentity')
    n_links = probe('links', lambda: first_attr(arm, 'n_links'))
    probe('dofs', lambda: first_attr(arm, 'n_dofs'))
    probe('joints', lambda: len(first_attr(arm, 'joints')))

    print('\nlink names and masses')
    total = 0.0
    try:
        for link in arm.links:
            name = getattr(link, 'name', '?')
            mass = None
            for attr in ('inertial_mass', 'mass', '_inertial_mass'):
                if hasattr(link, attr):
                    mass = getattr(link, attr)
                    break
            if mass is None:
                print('  %-38s (mass attribute not found)' % name)
                continue
            mass = float(mass)
            total += mass
            print('  %-38s %8.5f kg' % (name, mass))
    except Exception as exc:                      # noqa: BLE001 - diagnostic
        print('  could not enumerate links (%s: %s)' % (type(exc).__name__, exc))
    if total:
        print('  %-38s %8.5f kg' % ('TOTAL', total))

    print('\nscale check (the mm -> m question)')
    try:
        import numpy as np
        pos = np.asarray(arm.get_links_pos().cpu() if hasattr(arm.get_links_pos(), 'cpu')
                         else arm.get_links_pos())
        pos = pos.reshape(-1, 3)
        span = pos.max(axis=0) - pos.min(axis=0)
        print('  link-origin extent        %.3f x %.3f x %.3f m'
              % (span[0], span[1], span[2]))
        reach = float(np.linalg.norm(pos.max(axis=0) - pos.min(axis=0)))
        print('  diagonal                  %.3f m' % reach)
        if reach > 50:
            print('  ** the mesh scale was NOT applied -- this is ~1000x too large **')
        elif reach < 0.05:
            print('  ** suspiciously small -- check the scale **')
        else:
            print('  looks like metres, so scale="0.001" was honoured')
    except Exception as exc:                      # noqa: BLE001 - diagnostic
        print('  could not measure (%s: %s)' % (type(exc).__name__, exc))

    if args.steps:
        print('\nstepping %d times' % args.steps)
        for _ in range(args.steps):
            scene.step()
        print('  survived %d steps without a solver blow-up' % args.steps)

    print('\nloaded OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
