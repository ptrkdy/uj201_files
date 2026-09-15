#!/usr/bin/env python3
"""
urdf_build -- turn a urdf_extract JSON model into a ROS 2 description package.

    python urdf_build.py arm_assembly_model.json -o ./out
    python urdf_build.py arm_assembly_model.json -o ./out --check-meshes

This is the generation half of the SDK, deliberately split from extraction so
it runs on plain Python with no Fusion and no ROS. Iterate on URDF output here
in seconds instead of re-exporting from Fusion each time.

Geometry convention (matching the original exporter, which is known to work):
link frames are axis-aligned with world; a link's frame sits at the origin of
the joint that attaches it; meshes are exported in world position, so the
visual/collision origin is the negation of the link's world position.

--check-meshes tests that assumption instead of trusting it: it reads each
STL's bounding box and compares it against the link's world centre of mass.
If the meshes turn out to be in local coordinates the report says so and the
generated origins would need the world_transform correction instead.
"""

import argparse
import json
import math
import os
import shutil
import struct
import sys

PKG_SUFFIX = '_description'


def fmt(values):
    # Normalise -0.0 to 0 so the output does not read as "-0 -0 -0".
    return ' '.join('%g' % (v if v else 0.0) for v in values)


def sub(a, b):
    return [x - y for x, y in zip(a, b)]


def neg(a):
    return [-x for x in a]


# --------------------------------------------------------------------------
# model -> URDF

def link_origins(model, report):
    """World position of each link's frame. base_link is the world origin.

    A link's frame sits at the origin of the joint that attaches it, so a link
    that is never any joint's child has no frame under this convention. Rather
    than drop it (which leaves joints pointing at links that do not exist), fall
    back to the occurrence's own world transform and say so.
    """
    origins = {}
    for link in model['links']:
        if link.get('is_base'):
            origins[link['name']] = [0.0, 0.0, 0.0]
    for joint in model['joints']:
        origins[joint['child']] = list(joint['origin'])

    for link in model['links']:
        if link['name'] in origins:
            continue
        matrix = link.get('world_transform')
        if matrix and len(matrix) >= 12:
            # Fusion works internally in cm; the model is in metres.
            origins[link['name']] = [round(matrix[i] / 100.0, 6) for i in (3, 7, 11)]
            placed = 'placed at its own world position'
        else:
            origins[link['name']] = [0.0, 0.0, 0.0]
            placed = 'placed at the world origin (no transform available)'
        report.append('link "%s" is never any joint\'s child, so it is a root of its '
                      'own disconnected sub-tree -- %s. It needs a joint attaching it '
                      'to its parent.' % (link['name'], placed))
    return origins


def rpy_from_matrix(matrix):
    """Fixed-axis XYZ angles from a row-major 4x4, matching URDF's rpy."""
    m00, m01, m02 = matrix[0], matrix[1], matrix[2]
    m10, m11, m12 = matrix[4], matrix[5], matrix[6]
    m20, m21, m22 = matrix[8], matrix[9], matrix[10]

    sy = math.sqrt(m00 * m00 + m10 * m10)
    if sy < 1e-9:  # gimbal lock: yaw and roll are degenerate, fold into roll
        return [math.atan2(-m12, m11), math.atan2(-m20, sy), 0.0]
    return [math.atan2(m21, m22), math.atan2(-m20, sy), math.atan2(m10, m00)]


def mesh_pose(link, origin, mesh_frame):
    """Where to put the mesh relative to the link frame.

    'world': the STL already carries the part's world position, so the origin is
    just the negation of the link frame (the original exporter's convention,
    which its copy_occs() mutation creates).

    'local': the STL is in the occurrence's own frame -- which is what a
    read-only export of an occurrence actually produces -- so the occurrence's
    full world transform has to be applied, rotation included.
    """
    if mesh_frame != 'local':
        return neg(origin), [0.0, 0.0, 0.0]
    matrix = link.get('world_transform')
    position = world_position(link)
    if not matrix or position is None:
        return neg(origin), [0.0, 0.0, 0.0]
    return sub(position, origin), rpy_from_matrix(matrix)


def render_link(link, origin, pkg, mesh_rel, mesh_frame='world', plain=False):
    com = sub(link['center_of_mass'], origin)
    ixx, iyy, izz, ixy, iyz, ixz = link['inertia']
    visual, visual_rpy = mesh_pose(link, origin, mesh_frame)

    if mesh_rel:
        # A plain .urdf carries a relative path so any browser viewer can fetch
        # it; the .xacro keeps the $(find) form that ROS expects.
        target = mesh_rel if plain else 'file://$(find %s)/%s' % (pkg, mesh_rel)
        shape = '<mesh filename="%s" scale="0.001 0.001 0.001"/>' % target
    else:
        # No STL for this link: a small box keeps the URDF loadable in RViz.
        shape = '<box size="0.01 0.01 0.01"/>'

    out = ['<link name="%s">' % link['name'],
           '  <inertial>',
           '    <origin xyz="%s" rpy="0 0 0"/>' % fmt(com),
           '    <mass value="%g"/>' % link['mass'],
           '    <inertia ixx="%g" iyy="%g" izz="%g" ixy="%g" iyz="%g" ixz="%g"/>'
           % (ixx, iyy, izz, ixy, iyz, ixz),
           '  </inertial>']
    for tag in ('visual', 'collision'):
        out.append('  <%s>' % tag)
        out.append('    <origin xyz="%s" rpy="%s"/>' % (fmt(visual), fmt(visual_rpy)))
        out.append('    <geometry>')
        out.append('      %s' % shape)
        out.append('    </geometry>')
        if tag == 'visual':
            if plain:
                out.append('    <material name="silver">')
                out.append('      <color rgba="0.700 0.700 0.700 1.000"/>')
                out.append('    </material>')
            else:
                out.append('    <material name="silver"/>')
        out.append('  </%s>' % tag)
    out.append('</link>')
    return '\n'.join(out)


def render_joint(joint, origins):
    parent, child = joint['parent'], joint['child']
    xyz = sub(origins.get(child, [0, 0, 0]), origins.get(parent, [0, 0, 0]))

    out = ['<joint name="%s" type="%s">' % (joint['name'], joint['type'])]
    out.append('  <origin xyz="%s" rpy="0 0 0"/>' % fmt(xyz))
    out.append('  <parent link="%s"/>' % parent)
    out.append('  <child link="%s"/>' % child)
    if joint['type'] in ('revolute', 'continuous', 'prismatic'):
        out.append('  <axis xyz="%s"/>' % fmt(joint['axis']))
    if joint['type'] in ('revolute', 'prismatic'):
        out.append('  <limit upper="%g" lower="%g" effort="%g" velocity="%g"/>'
                   % (joint['upper'], joint['lower'],
                      joint.get('effort', 100), joint.get('velocity', 100)))
    out.append('</joint>')
    return '\n'.join(out)


def render_robot(model, pkg, report, plain=False):
    origins = link_origins(model, report)
    parts = ['<?xml version="1.0" ?>',
             '<robot name="%s" xmlns:xacro="http://www.ros.org/wiki/xacro">' % model['robot_name'],
             '',
             '' if plain else
             '<xacro:include filename="$(find %s)/urdf/materials.xacro" />' % pkg,
             '']

    mesh_frame = model.get('mesh_frame', 'world')
    # Every link in the model gets written, base first, then the rest in order.
    for link in sorted(model['links'], key=lambda l: 0 if l.get('is_base') else 1):
        parts.append(render_link(link, origins[link['name']], pkg, link.get('mesh'),
                                 mesh_frame, plain))
        parts.append('')
    for joint in model['joints']:
        parts.append(render_joint(joint, origins))
        parts.append('')
    parts.append('</robot>')
    return '\n'.join(parts) + '\n'


MATERIALS = '''<?xml version="1.0" ?>
<robot name="%s" xmlns:xacro="http://www.ros.org/wiki/xacro">

<material name="silver">
  <color rgba="0.700 0.700 0.700 1.000"/>
</material>

</robot>
'''

DISPLAY_LAUNCH = '''import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('%(pkg)s')
    xacro_file = os.path.join(pkg, 'urdf', '%(robot)s.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui', output='screen'),
        ExecuteProcess(cmd=['rviz2'], output='screen'),
    ])
'''

PACKAGE_XML = '''<?xml version="1.0"?>
<package format="3">
  <name>%(pkg)s</name>
  <version>0.0.1</version>
  <description>URDF description for %(robot)s</description>
  <maintainer email="you@example.com">you</maintainer>
  <license>MIT</license>

  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>xacro</exec_depend>

  <export><build_type>ament_python</build_type></export>
</package>
'''

SETUP_PY = '''from glob import glob
import os
from setuptools import setup

package_name = '%(pkg)s'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='URDF description for %(robot)s',
    license='MIT',
)
'''


# --------------------------------------------------------------------------
# mesh frame verification

def stl_bounds(path):
    """Bounding box of a binary or ASCII STL, in the file's own units."""
    with open(path, 'rb') as f:
        head = f.read(84)
        if len(head) < 84:
            return None
        count = struct.unpack('<I', head[80:84])[0]
        expected = 84 + count * 50
        size = os.path.getsize(path)
        if size != expected:
            return ascii_stl_bounds(path)

        lo = [float('inf')] * 3
        hi = [float('-inf')] * 3
        for _ in range(count):
            tri = f.read(50)
            if len(tri) < 50:
                break
            vals = struct.unpack('<12f', tri[:48])
            for v in range(3):
                for axis in range(3):
                    c = vals[3 + v * 3 + axis]
                    lo[axis] = min(lo[axis], c)
                    hi[axis] = max(hi[axis], c)
        return lo, hi


def ascii_stl_bounds(path):
    lo = [float('inf')] * 3
    hi = [float('-inf')] * 3
    with open(path, 'r', errors='ignore') as f:
        for line in f:
            parts = line.split()
            if len(parts) == 4 and parts[0] == 'vertex':
                try:
                    c = [float(x) for x in parts[1:]]
                except ValueError:
                    continue
                for axis in range(3):
                    lo[axis] = min(lo[axis], c[axis])
                    hi[axis] = max(hi[axis], c[axis])
    if lo[0] == float('inf'):
        return None
    return lo, hi


def check_meshes(model, out_dir):
    """Is each STL in world coordinates, as the origin maths assumes?"""
    print('mesh frame check')
    print('-' * 70)
    verdicts = []
    for link in model['links']:
        rel = link.get('mesh')
        if not rel:
            print('  %-28s no mesh' % link['name'])
            continue
        path = os.path.join(out_dir, rel)
        if not os.path.isfile(path):
            print('  %-28s MISSING %s' % (link['name'], path))
            continue
        bounds = stl_bounds(path)
        if not bounds:
            print('  %-28s unreadable' % link['name'])
            continue
        lo, hi = bounds
        # STL is in mm, the model is in m.
        centre = [(lo[i] + hi[i]) / 2000.0 for i in range(3)]
        span = max((hi[i] - lo[i]) / 1000.0 for i in range(3)) or 1e-6
        com = link['center_of_mass']
        err = max(abs(centre[i] - com[i]) for i in range(3))
        world = err < max(span, 0.02)
        verdicts.append(world)
        print('  %-28s %s  bbox centre %s vs world CoM %s (err %.3f m, size %.3f m)'
              % (link['name'], 'WORLD ' if world else 'LOCAL?',
                 fmt(centre), fmt(com), err, span))
    print()
    if verdicts and not all(verdicts):
        print('WARNING: some meshes do not sit at their world centre of mass. The '
              'generated visual origins assume world-positioned meshes; if RViz shows '
              'parts scattered, the meshes are in local coordinates and the origins '
              'need the world_transform correction.')
    elif verdicts:
        print('All meshes sit at their world centre of mass -- the world-frame '
              'assumption holds and the generated origins are correct.')
    print()


# --------------------------------------------------------------------------

def apply_overrides(model, path, report, model_dir=None):
    """Patch link mass/inertia and joint limits from a JSON file.

    Stand-in geometry (a surface-modelled servo, say) reports zero mass from
    Fusion. Correcting that here beats modelling fake solids, and keeps the
    datasheet values in version control next to the model.

        {"links":  {"servo_base": {"mass": 0.055}},
         "joints": {"revolute_5": {"effort": 3.0, "velocity": 4.8}}}
    """
    with open(path) as f:
        overrides = json.load(f)

    by_name = {l['name']: l for l in model['links']}
    for name, patch in overrides.get('links', {}).items():
        link = by_name.get(name)
        if link is None:
            report.append('override for unknown link "%s" ignored' % name)
            continue
        for key, value in patch.items():
            link[key] = value
        report.append('link "%s": applied %s' % (name, ', '.join(sorted(patch))))

    by_joint = {j['name']: j for j in model['joints']}
    for name, patch in overrides.get('joints', {}).items():
        joint = by_joint.get(name)
        if joint is None:
            report.append('override for unknown joint "%s" ignored' % name)
            continue
        for key, value in patch.items():
            joint[key] = value
        report.append('joint "%s": applied %s' % (name, ', '.join(sorted(patch))))

    # Detach before the extra groups are added, so an override group can claim a
    # link that a Fusion rigid group would otherwise have grabbed first. Needed
    # where the CAD grouping and the kinematic truth disagree -- an actuator
    # bolted into one housing but carried by another link's motion.
    detach = set(overrides.get('detach_links', []))
    if detach:
        removed = 0
        for group in model.get('rigid_groups') or []:
            before = len(group.get('member_names', []))
            group['member_names'] = [m for m in group.get('member_names', [])
                                     if m not in detach]
            removed += before - len(group['member_names'])
        report.append('detached %d link membership(s) from Fusion rigid groups: %s'
                      % (removed, ', '.join(sorted(detach))))

    split_links(model, overrides.get('split_links', []),
                model_dir or os.path.dirname(os.path.abspath(path)), report)

    # After the split, so a reversed joint refers to the half that survived.
    reverse_joints(model, overrides.get('reverse_joints', []), report)

    split_universal_joints(model, overrides.get('universal_joints', []), report)

    # Extra rigid groups that cannot come from Fusion -- typically ones whose
    # member is a link synthesised above.
    extra = overrides.get('rigid_groups', [])
    if extra:
        for group in extra:
            group.setdefault('member_names', group.get('members', []))
        model.setdefault('rigid_groups', []).extend(extra)
        report.append('overrides: added %d extra rigid group(s).' % len(extra))


def reverse_joints(model, names, report):
    """Flip a joint's parent and child.

    Which side of a Fusion joint is Component1 is a modelling choice, not a
    kinematic fact. When it points against the tree -- the moving assembly
    chosen as the parent and the thing it hangs from as the child -- the joint
    drives the wrong body, and a link can end up the child of two joints, which
    a URDF tree forbids.

    Reversing swaps parent and child and negates the limits, since rotating A
    relative to B by theta is rotating B relative to A by -theta. The axis and
    the world-space origin are unchanged.
    """
    wanted = set(names)
    for joint in model['joints']:
        if joint['fusion_name'] not in wanted and joint['name'] not in wanted:
            continue
        joint['parent'], joint['child'] = joint['child'], joint['parent']
        joint['parent_id'], joint['child_id'] = (joint.get('child_id'),
                                                 joint.get('parent_id'))
        lower, upper = joint.get('lower', 0.0), joint.get('upper', 0.0)
        joint['lower'], joint['upper'] = -upper, -lower
        report.append('reversed joint "%s": now "%s" -> "%s", limits [%g, %g]'
                      % (joint['fusion_name'], joint['parent'], joint['child'],
                         joint['lower'], joint['upper']))


def split_links(model, specs, model_dir, report):
    """Cut a body that carries two joints into two links, so it can be a tree.

    A serial two-axis wrist is native URDF; a single body with two parent joints
    is not. The cut plane is perpendicular to the line joining the two joint
    origins, so each half keeps one axis, and each half's mass, centre of mass
    and inertia are integrated from its own geometry rather than guessed.

        {"split_links": [{"link": "handles_prototype",
                          "between_joints": ["Revolute 17", "Revolute 18"],
                          "parts": ["uj_yoke_in", "uj_yoke_out"],
                          "fraction": 0.5}]}
    """
    if not specs:
        return
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import mesh_split as ms

    links = {l['name']: l for l in model['links']}
    joints = {j['fusion_name']: j for j in model['joints']}

    for spec in specs:
        link = links.get(spec['link'])
        if link is None:
            report.append('split_links: unknown link "%s"' % spec['link'])
            continue
        first, second = (joints.get(n) for n in spec.get('between_joints', [])[:2])
        if not first or not second:
            report.append('split_links: "%s" needs two known joints' % spec['link'])
            continue

        matrix = link.get('world_transform')
        mesh_rel = link.get('mesh') or 'meshes/%s.stl' % link['name']
        mesh_path = os.path.join(model_dir, mesh_rel)
        if not matrix or not os.path.isfile(mesh_path):
            report.append('split_links: no mesh or transform for "%s"' % spec['link'])
            continue

        rot = [[matrix[0], matrix[1], matrix[2]],
               [matrix[4], matrix[5], matrix[6]],
               [matrix[8], matrix[9], matrix[10]]]
        trans = [matrix[3] / 100.0, matrix[7] / 100.0, matrix[11] / 100.0]

        def to_local_point(p):          # world metres -> local millimetres
            d = [p[a] - trans[a] for a in range(3)]
            return tuple(sum(rot[k][a] * d[k] for k in range(3)) * 1000.0
                         for a in range(3))

        def to_world(v):                # local metres -> world metres
            return [sum(rot[a][k] * v[k] for k in range(3)) + trans[a]
                    for a in range(3)]

        def to_local_dir(v):
            return tuple(sum(rot[k][a] * v[k] for k in range(3)) for a in range(3))

        tris = ms.read_stl(mesh_path)
        names = spec.get('parts') or [link['name'] + '_a', link['name'] + '_b']
        o_first, o_second = first['origin'], second['origin']

        if spec.get('from_mesh'):
            # The halves were split in CAD and exported together, so take the
            # parting surface the CAD actually used rather than inventing a
            # plane -- which is the only option when yoke arms interlock.
            source = os.path.join(model_dir, spec['from_mesh'])
            if not os.path.isfile(source):
                report.append('split_links: no such mesh "%s"' % spec['from_mesh'])
                continue
            pieces = ms.connected_shells(ms.read_stl(source))
            if len(pieces) != len(names):
                report.append('split_links: "%s" holds %d shell(s) but %d part name(s) '
                              'were given' % (spec['from_mesh'], len(pieces), len(names)))
                continue

            # Exported halves are usually moved apart, losing their place in the
            # parent frame. Recover it by landing each back on the original.
            reference = ms.vertex_set(tris)
            restored = []
            for piece in pieces:
                delta, score = ms.solve_translation(piece, reference)
                restored.append(ms.translate(piece, delta))
                report.append('  shell of %d triangles restored by %s mm '
                              '(%.0f%% of its vertices land on the original)'
                              % (len(piece), [round(v, 2) for v in delta], score * 100))

            # Whichever half carries the first axis is the one that joint drives.
            def near(half, world_point):
                p = to_local_point(world_point)
                return min(math.dist(p, v) for tri in half for v in tri)

            halves = (restored if near(restored[0], o_first) <= near(restored[1], o_first)
                      else list(reversed(restored)))
            integration_origin = (0.0, 0.0, 0.0)   # shells are closed solids
        else:
            fraction = float(spec.get('fraction', 0.5))
            cut_world = [o_first[a] + (o_second[a] - o_first[a]) * fraction
                         for a in range(3)]
            normal_world = ms.normalise(tuple(o_second[a] - o_first[a]
                                              for a in range(3)))
            plane_point = to_local_point(cut_world)
            plane_normal = ms.normalise(to_local_dir(normal_world))
            halves = list(ms.clip(tris, plane_point, plane_normal))
            integration_origin = plane_point

        total_volume, _, _ = ms.mass_properties(tris, (0.0, 0.0, 0.0))
        if total_volume <= 0:
            report.append('split_links: "%s" has no usable volume' % spec['link'])
            continue
        density = link['mass'] / total_volume

        new_links = []
        for name, half in zip(names, halves):
            volume, com_local, inertia_local = ms.mass_properties(half, integration_origin)
            # Shift from the integration origin back into the mesh's own frame
            # before converting to world coordinates.
            offset = [integration_origin[a] / 1000.0 for a in range(3)]
            com_mesh = [com_local[a] + offset[a] for a in range(3)]

            i_local = [[inertia_local[0], inertia_local[3], inertia_local[5]],
                       [inertia_local[3], inertia_local[1], inertia_local[4]],
                       [inertia_local[5], inertia_local[4], inertia_local[2]]]
            i_world = [[sum(rot[i][p] * i_local[p][q] * rot[j][q]
                            for p in range(3) for q in range(3))
                        for j in range(3)] for i in range(3)]

            rel = 'meshes/%s.stl' % name
            ms.write_stl(os.path.join(model_dir, rel), half)

            part = dict(link)
            part.update({
                'id': link['id'] + '#' + name,
                'name': name,
                'component': link['component'] + ' (split)',
                'mass': volume * density,
                'center_of_mass': [round(v, 9) for v in to_world(com_mesh)],
                'inertia': [round(v * density, 12) for v in
                            (i_world[0][0], i_world[1][1], i_world[2][2],
                             i_world[0][1], i_world[1][2], i_world[0][2])],
                'mesh': rel,
                'is_base': False,
            })
            new_links.append(part)
            report.append('split "%s" -> "%s": %d triangles, %.5f kg'
                          % (link['name'], name, len(half), part['mass']))

        model['links'] = [l for l in model['links'] if l['name'] != link['name']]
        model['links'].extend(new_links)

        # First joint drives the near half; second now hangs off it, which is
        # what makes the wrist serial instead of a closed loop.
        first['child'] = names[0]
        if spec.get('bind'):
            # The halves are one rigid body -- the split exists only to keep the
            # CAD geometry -- so bond them and leave the second joint to drive
            # whatever actually hangs off the assembly.
            for side in ('parent', 'child'):
                if second[side] == link['name']:
                    second[side] = names[1]
            model['joints'].append({
                'fusion_name': 'split bond', 'name': '%s_to_%s' % (names[0], names[1]),
                'type': 'fixed', 'parent': names[0], 'child': names[1],
                'parent_id': '', 'child_id': '',
                'axis': [0.0, 0.0, 1.0], 'origin': list(second['origin']),
                'lower': 0.0, 'upper': 0.0,
            })
            report.append('bonded "%s" to "%s" with a fixed joint; %s keeps driving '
                          '"%s"' % (names[0], names[1], second['fusion_name'],
                                    second['child']))
        else:
            second['parent'] = names[0]
            second['child'] = names[1]
            report.append('rewired: %s -> "%s", then %s -> "%s" (serial two-axis wrist)'
                          % (first['fusion_name'], names[0],
                             second['fusion_name'], names[1]))

        for group in model.get('rigid_groups') or []:
            group['member_names'] = [names[0] if m == link['name'] else m
                                     for m in group.get('member_names', [])]


def correct_joint_origins(model, report):
    """Use the raw world-space joint geometry when the extract recorded it.

    Lets a model produced by an older urdf_extract be corrected here rather than
    needing another round trip through Fusion.
    """
    fixed = 0
    for joint in model['joints']:
        point = (joint.get('raw') or {}).get('geometry_two')
        if not point:
            continue
        corrected = [round(v / 100.0, 6) for v in point]
        if max(abs(a - b) for a, b in zip(corrected, joint['origin'])) > 1e-6:
            joint['origin'] = corrected
            fixed += 1
    if fixed:
        report.append('corrected %d joint origin(s) from the raw world-space joint '
                      'geometry -- the inherited heuristic double-transformed them.'
                      % fixed)


def reattach_meshes(model, model_dir, report):
    """A JSON-only extract carries no mesh paths; reuse STLs already on disk."""
    found = 0
    for link in model['links']:
        if link.get('mesh'):
            continue
        rel = 'meshes/' + link['name'] + '.stl'
        if os.path.isfile(os.path.join(model_dir, rel)):
            link['mesh'] = rel
            found += 1
    if found:
        report.append('reattached %d mesh(es) from a previous STL export.' % found)


def fix_degenerate_com(model, report):
    """Give massless links a believable centre of mass.

    Fusion returns centreOfMass (0,0,0) for a component with no mass -- which is
    every surface-modelled stand-in. Left alone, overriding the mass then places
    that link's whole inertia at the world origin instead of at the part. Fall
    back to the occurrence's own world position, which for a small servo block
    is within a few millimetres of the truth.
    """
    fixed = 0
    for link in model['links']:
        if link['mass'] > 1e-9:
            continue
        if max(abs(v) for v in link['center_of_mass']) > 1e-9:
            continue
        position = world_position(link)
        if position is None:
            continue
        link['center_of_mass'] = position
        fixed += 1
    if fixed:
        report.append('%d massless link(s) reported centre of mass (0,0,0); moved each '
                      'to its own world position so an overridden mass lands on the part '
                      'rather than at the world origin.' % fixed)


def prune_wrapper_links(model, report):
    """Drop container occurrences whose children are already links.

    Fusion's getPhysicalProperties() on an occurrence includes everything below
    it, so when a wrapper and its contents are both links the wrapper's mass is
    the children's mass counted a second time. A wrapper with no bodies of its
    own contributes no geometry either, so it is pure double-counting and is
    removed. One that does have bodies is kept -- dropping it would lose
    geometry -- but the inflated mass is reported.
    """
    links = model['links']
    ids = {l['id']: l for l in links}
    jointed = set()
    for joint in model['joints']:
        jointed.add(joint['parent'])
        jointed.add(joint['child'])

    drop = []
    for link in links:
        descendants = [other for other in links
                       if other is not link and other['id'].startswith(link['id'] + '+')]
        if not descendants:
            continue
        child_mass = sum(d['mass'] for d in descendants)
        if link['body_count'] == 0 and link['name'] not in jointed:
            drop.append(link)
            report.append('dropped wrapper link "%s": no bodies of its own, and its '
                          'mass (%.5f kg) is %s counted again.'
                          % (link['name'], link['mass'],
                             ' + '.join(d['name'] for d in descendants)))
        elif child_mass > 0:
            report.append('link "%s" (%.5f kg) contains link(s) %s (%.5f kg), which are '
                          'counted twice -- Fusion reports a container\'s mass including '
                          'its contents. Subtract in overrides.json if this matters for '
                          'dynamics.'
                          % (link['name'], link['mass'],
                             ', '.join(d['name'] for d in descendants), child_mass))

    if not drop:
        return
    dropped = {l['name'] for l in drop}
    model['links'] = [l for l in links if l not in drop]
    for group in model.get('rigid_groups') or []:
        group['member_names'] = [m for m in group.get('member_names', [])
                                 if m not in dropped]


def world_position(link):
    matrix = link.get('world_transform')
    if matrix and len(matrix) >= 12:
        return [round(matrix[i] / 100.0, 6) for i in (3, 7, 11)]
    return None


def expand_rigid_groups(model, report):
    """Turn Fusion Rigid Groups into URDF fixed joints.

    A rigid group is an unordered set of "these move together", but URDF needs a
    direction: one member is the parent and the rest hang off it. So each group
    is anchored on a member that is already attached to the tree, and groups are
    processed repeatedly until no more can be anchored -- that way a chain of
    groups resolves outward from base_link regardless of the order they were
    created in.

    A member that already has a parent joint is left alone: giving it a second
    one would make the model a graph rather than a tree.
    """
    groups = model.get('rigid_groups') or []
    if not groups:
        return

    by_name = {l['name']: l for l in model['links']}
    origins = {}
    connected = set()
    for link in model['links']:
        if link.get('is_base'):
            connected.add(link['name'])
            origins[link['name']] = [0.0, 0.0, 0.0]
    for joint in model['joints']:
        connected.add(joint['child'])
        origins[joint['child']] = list(joint['origin'])

    pending = [g for g in groups]
    added = 0

    def attach(group, anchor, members):
        nonlocal added
        for member in members:
            if member == anchor or member in connected:
                continue
            link = by_name.get(member)
            position = world_position(link) if link else None
            if position is None:
                position = origins.get(anchor, [0.0, 0.0, 0.0])
            model['joints'].append({
                'fusion_name': group.get('name', 'rigid group'),
                'name': '%s_fixed' % member,
                'type': 'fixed',
                'parent': anchor,
                'child': member,
                'parent_id': '', 'child_id': link['id'] if link else '',
                'axis': [0.0, 0.0, 1.0],
                'origin': position,
                'lower': 0.0, 'upper': 0.0,
            })
            origins[member] = position
            connected.add(member)
            added += 1

    progress = True
    while pending and progress:
        progress = False
        for group in list(pending):
            members = [m for m in group.get('member_names', []) if m in by_name]
            if len(members) < 2:
                pending.remove(group)
                continue
            anchors = [m for m in members if m in connected]
            if not anchors:
                continue  # wait for another group to connect it to the tree
            attach(group, anchors[0], members)
            pending.remove(group)
            progress = True

    # Anything still unanchored is a floating cluster: keep it internally rigid
    # so at least it moves as one piece, and say that it is not attached.
    for group in pending:
        members = [m for m in group.get('member_names', []) if m in by_name]
        if len(members) < 2:
            continue
        attach(group, members[0], members)
        report.append('rigid group "%s" is not connected to base_link by any joint; '
                      'its members were linked to "%s", which is still a floating root.'
                      % (group.get('name', '?'), members[0]))

    report.append('rigid groups: %d group(s) became %d fixed joint(s).'
                  % (len(groups), added))


def split_universal_joints(model, specs, report):
    """Insert the intermediate link a 2-DOF joint needs to exist in URDF.

    URDF is a tree: a link has exactly one parent joint, so a universal joint --
    two perpendicular revolutes acting on one part -- cannot be expressed
    directly. The standard idiom is a cross (spider) link between them:

        yoke --[rev A]--> cross --[rev B]--> output

    The cross does not have to exist in CAD. Declared in the overrides file as:

        {"universal_joints": [{"joints": ["revolute_17", "revolute_18"],
                               "link_name": "uj201_cross", "mass": 0.02}]}

    List the joints base-first. A real universal joint has both axes meeting at
    the cross centre, so their origins should coincide; if they do not, this
    says so, because that means one revolute is snapped to the wrong feature or
    a genuine structural member is missing between them.
    """
    by_joint = {j['name']: j for j in model['joints']}

    for spec in specs:
        names = spec.get('joints', [])
        if len(names) != 2:
            report.append('universal_joint needs exactly 2 joints, got %d' % len(names))
            continue
        first, second = (by_joint.get(n) for n in names)
        if first is None or second is None:
            missing = [n for n in names if n not in by_joint]
            report.append('universal_joint references unknown joint(s): %s'
                          % ', '.join(missing))
            continue
        if first['child'] != second['child']:
            report.append('universal_joint joints "%s" and "%s" do not share a child '
                          '("%s" vs "%s"); nothing to split'
                          % (names[0], names[1], first['child'], second['child']))
            continue

        gap = math.sqrt(sum((a - b) ** 2
                            for a, b in zip(first['origin'], second['origin'])))
        if gap > 0.001:
            report.append('universal_joint "%s"/"%s": axes meet %.1f mm apart. A real '
                          'universal joint has both axes through the cross centre -- '
                          'check what each revolute is snapped to in Fusion.'
                          % (names[0], names[1], gap * 1000.0))

        dot = sum(a * b for a, b in zip(first['axis'], second['axis']))
        if abs(dot) > 0.01:
            report.append('universal_joint "%s"/"%s": axes are not perpendicular '
                          '(dot=%.3f).' % (names[0], names[1], dot))

        cross_name = spec.get('link_name', first['child'] + '_cross')
        mass = spec.get('mass', 0.01)
        inertia = spec.get('inertia', [1e-6, 1e-6, 1e-6, 0.0, 0.0, 0.0])

        model['links'].append({
            'id': 'synthetic:' + cross_name,
            'name': cross_name,
            'component': '(synthesised)',
            'occurrence': '',
            'parent_path': '',
            'world_transform': None,
            'depth': 0,
            'body_count': 0,
            'mass': mass,
            # The cross frame sits at the joint that attaches it, so its centre
            # of mass in world coordinates is that joint's origin.
            'center_of_mass': list(first['origin']),
            'inertia': inertia,
            'is_base': False,
            'mesh': None,
        })

        output = first['child']
        first['child'] = cross_name
        second['parent'] = cross_name
        report.append('universal_joint: inserted "%s" between "%s" and "%s" so "%s" '
                      'has one parent joint.'
                      % (cross_name, names[0], names[1], output))


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(text)
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('model', help='the *_model.json written by urdf_extract')
    ap.add_argument('-o', '--out', default='.', help='where to write the package')
    ap.add_argument('--check-meshes', action='store_true',
                    help='verify STL coordinate frames against the model')
    ap.add_argument('--mesh-frame', choices=['world', 'local'],
                    help="override the model's mesh_frame")
    ap.add_argument('--overrides',
                    help='JSON file patching link mass/inertia and joint limits')
    args = ap.parse_args()

    with open(args.model) as f:
        model = json.load(f)

    if args.mesh_frame:
        model['mesh_frame'] = args.mesh_frame

    report = []
    report.append('meshes treated as %s-frame.' % model.get('mesh_frame', 'world'))
    correct_joint_origins(model, report)
    reattach_meshes(model, os.path.dirname(os.path.abspath(args.model)), report)
    # Overrides first, so a rigid group may name a link that only exists after
    # the universal-joint split (the synthesised cross), and so extra groups
    # declared in the overrides file are expanded alongside Fusion's own.
    fix_degenerate_com(model, report)
    if args.overrides:
        apply_overrides(model, args.overrides, report,
                        os.path.dirname(os.path.abspath(args.model)))
    prune_wrapper_links(model, report)
    expand_rigid_groups(model, report)

    robot = model['robot_name']
    pkg = robot + PKG_SUFFIX
    out_dir = os.path.abspath(os.path.join(args.out, pkg))

    problems = model.get('problems', [])
    errors = [p for p in problems if p['severity'] == 'error']
    for p in problems:
        print('  [%s] %s' % (p['severity'], p['message']))
    if problems:
        print()

    subs = {'pkg': pkg, 'robot': robot}
    xacro_text = render_robot(model, pkg, report)
    for line in report:
        print('  [build] %s' % line)
    if report:
        print()

    # A flat .urdf next to the .xacro: no includes, no $(find), relative mesh
    # paths -- loadable by browser viewers and by check_urdf without xacro.
    plain_text = render_robot(model, pkg, [], plain=True)
    viewer_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               'viewer_template.html')
    viewer_html = ''
    if os.path.isfile(viewer_path):
        with open(viewer_path, encoding='utf-8') as vf:
            viewer_html = (vf.read().replace('__ROBOT__', robot)
                                    .replace('__URDF__', 'urdf/' + robot + '.urdf'))

    written = [
        write(os.path.join(out_dir, 'urdf', robot + '.xacro'), xacro_text),
        write(os.path.join(out_dir, 'urdf', robot + '.urdf'), plain_text),
        write(os.path.join(out_dir, 'urdf', 'materials.xacro'), MATERIALS % robot),
        write(os.path.join(out_dir, 'launch', 'display.launch.py'), DISPLAY_LAUNCH % subs),
        write(os.path.join(out_dir, 'package.xml'), PACKAGE_XML % subs),
        write(os.path.join(out_dir, 'setup.py'), SETUP_PY % subs),
        write(os.path.join(out_dir, 'resource', pkg), ''),
        write(os.path.join(out_dir, pkg, '__init__.py'), ''),
    ]
    if viewer_html:
        written.append(write(os.path.join(out_dir, 'viewer.html'), viewer_html))

    # Meshes live next to the model JSON, where urdf_extract put them.
    mesh_src = os.path.join(os.path.dirname(os.path.abspath(args.model)), 'meshes')
    if os.path.isdir(mesh_src):
        mesh_dst = os.path.join(out_dir, 'meshes')
        os.makedirs(mesh_dst, exist_ok=True)
        copied = 0
        for name in os.listdir(mesh_src):
            if name.lower().endswith('.stl'):
                shutil.copy2(os.path.join(mesh_src, name), os.path.join(mesh_dst, name))
                copied += 1
        print('copied %d mesh(es) from %s' % (copied, mesh_src))

    print('wrote %s' % out_dir)
    for path in written:
        print('  %s' % os.path.relpath(path, out_dir))
    print()

    # Meshes come from the extractor's own output directory, next to the JSON.
    if args.check_meshes:
        check_meshes(model, os.path.dirname(os.path.abspath(args.model)))

    print('%d links, %d joints, %d extraction error(s)'
          % (len(model['links']), len(model['joints']), len(errors)))
    print('next: python urdf_lint.py %s'
          % os.path.join(out_dir, 'urdf', robot + '.xacro'))
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
