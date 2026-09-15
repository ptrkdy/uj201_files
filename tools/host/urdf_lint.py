#!/usr/bin/env python3
"""
urdf_lint -- validate a generated URDF/xacro without ROS installed.

Stands in for `xacro ... | check_urdf` while there is no ROS 2 on this machine,
and checks several things check_urdf does not: missing mesh files, degenerate
inertia tensors, names that are legal XML but illegal as ROS/TF frame names.

    python urdf_lint.py <path/to/robot.xacro> [--pkg-root DIR]

The xacro handling is deliberately minimal: it resolves <xacro:include> and
$(find <pkg>) and nothing else, which is all the fusion360-urdf-ros2 output
uses. A file with real xacro macros or properties needs the real xacro.
Exit code is 0 when there are no errors, 1 otherwise.
"""

import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET

XACRO_NS = 'http://www.ros.org/wiki/xacro'
VALID_JOINT_TYPES = {'revolute', 'continuous', 'prismatic', 'fixed', 'floating', 'planar'}
# ROS name rules in practice: no whitespace, no '/' (TF frame separator).
BAD_NAME = re.compile(r'[\s/]')


class Findings:
    def __init__(self):
        self.errors = []
        self.warnings = []

    def error(self, msg):
        self.errors.append(msg)

    def warn(self, msg):
        self.warnings.append(msg)


def resolve_find(path, pkg_root, pkg_name):
    """Turn 'file://$(find pkg)/meshes/x.stl' into a local filesystem path."""
    p = path
    if p.startswith('file://'):
        p = p[len('file://'):]
    if p.startswith('package://'):
        p = p[len('package://'):]
        parts = p.split('/', 1)
        return os.path.join(pkg_root, parts[1]) if len(parts) == 2 else None
    m = re.match(r'\$\(find\s+([^)]+)\)(.*)', p)
    if m:
        found_pkg, rest = m.group(1).strip(), m.group(2)
        if pkg_name and found_pkg != pkg_name:
            return None  # a different package; we cannot resolve it
        return os.path.normpath(os.path.join(pkg_root, rest.lstrip('/\\')))
    if os.path.isabs(p):
        return p
    return os.path.normpath(os.path.join(pkg_root, p))


def load(path, pkg_root, pkg_name, find, seen=None):
    """Parse a xacro/urdf file, splicing in <xacro:include>d robots."""
    seen = seen if seen is not None else set()
    real = os.path.realpath(path)
    if real in seen:
        return []
    seen.add(real)

    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as e:
        find.error('%s is not well-formed XML: %s' % (os.path.basename(path), e))
        return []
    except OSError as e:
        find.error('cannot read %s: %s' % (path, e))
        return []

    if root.tag != 'robot':
        find.error('%s: root element is <%s>, expected <robot>'
                   % (os.path.basename(path), root.tag))

    elements = []
    for child in root:
        if child.tag == '{%s}include' % XACRO_NS:
            target = child.get('filename', '')
            resolved = resolve_find(target, pkg_root, pkg_name)
            if resolved is None:
                find.warn('include %s points outside this package; not checked' % target)
            elif not os.path.isfile(resolved):
                find.error('<xacro:include> target does not exist: %s (from %s)'
                           % (target, os.path.basename(path)))
            else:
                elements.extend(load(resolved, pkg_root, pkg_name, find, seen))
        elif child.tag.startswith('{%s}' % XACRO_NS):
            find.warn('%s uses %s, which this linter does not evaluate -- run the real '
                      'xacro to check it' % (os.path.basename(path),
                                             child.tag.split('}')[1]))
        else:
            elements.append(child)
    return elements


def check_inertial(link_name, link, find):
    inertial = link.find('inertial')
    if inertial is None:
        find.warn('link "%s" has no <inertial>; Gazebo will treat it as massless'
                  % link_name)
        return

    mass_el = inertial.find('mass')
    mass = float(mass_el.get('value', 0)) if mass_el is not None else 0.0
    if mass <= 0:
        find.error('link "%s" has mass %g; a non-fixed joint on a zero-mass link makes '
                   'Gazebo reject the model' % (link_name, mass))
    elif mass < 0.001:
        find.warn('link "%s" has mass %.6g kg (<1 g) -- likely a missing material'
                  % (link_name, mass))

    i = inertial.find('inertia')
    if i is None:
        find.error('link "%s" has <inertial> but no <inertia>' % link_name)
        return
    try:
        ixx, iyy, izz = (float(i.get(k, 0)) for k in ('ixx', 'iyy', 'izz'))
    except ValueError:
        find.error('link "%s" has a non-numeric inertia value' % link_name)
        return

    if min(ixx, iyy, izz) <= 0:
        find.error('link "%s" has a non-positive principal moment '
                   '(ixx=%g iyy=%g izz=%g)' % (link_name, ixx, iyy, izz))
        return
    # A physical rigid body must satisfy the triangle inequality on its
    # principal moments; violating it usually means a bad mesh or units.
    for a, b, c, names in ((ixx, iyy, izz, 'ixx+iyy >= izz'),
                           (iyy, izz, ixx, 'iyy+izz >= ixx'),
                           (izz, ixx, iyy, 'izz+ixx >= iyy')):
        if a + b < c * (1 - 1e-6):
            find.warn('link "%s" violates the inertia triangle inequality (%s): '
                      'ixx=%g iyy=%g izz=%g' % (link_name, names, ixx, iyy, izz))


def check_meshes(link_name, link, pkg_root, pkg_name, find):
    for tag in ('visual', 'collision'):
        for el in link.findall(tag):
            for mesh in el.iter('mesh'):
                fn = mesh.get('filename', '')
                resolved = resolve_find(fn, pkg_root, pkg_name)
                if resolved is None:
                    find.warn('link "%s" %s mesh %s is in another package; not checked'
                              % (link_name, tag, fn))
                elif not os.path.isfile(resolved):
                    find.error('link "%s" %s mesh is missing on disk: %s'
                               % (link_name, tag, resolved))


def check_joint(joint, links, find):
    name = joint.get('name', '<unnamed>')
    jtype = joint.get('type', '')

    if BAD_NAME.search(name):
        find.error('joint name "%s" contains whitespace or "/". It is legal XML but '
                   'breaks ros2_control and TF lookups -- rename it in Fusion to '
                   'snake_case and re-export.' % name)

    if jtype not in VALID_JOINT_TYPES:
        find.error('joint "%s" has type "%s", which is not a URDF joint type. Valid: %s'
                   % (name, jtype, ', '.join(sorted(VALID_JOINT_TYPES))))

    parent_el, child_el = joint.find('parent'), joint.find('child')
    parent = parent_el.get('link') if parent_el is not None else None
    child = child_el.get('link') if child_el is not None else None

    for role, link_name in (('parent', parent), ('child', child)):
        if link_name is None:
            find.error('joint "%s" has no <%s link="...">' % (name, role))
        elif link_name not in links:
            find.error('joint "%s" references a %s link "%s" that is never defined. '
                       'This is the signature of a truncated export.'
                       % (name, role, link_name))

    if jtype in ('revolute', 'continuous', 'prismatic'):
        axis_el = joint.find('axis')
        if axis_el is None:
            find.error('joint "%s" is %s but has no <axis>' % (name, jtype))
        else:
            try:
                axis = [float(v) for v in axis_el.get('xyz', '').split()]
            except ValueError:
                axis = []
            if len(axis) != 3:
                find.error('joint "%s" has a malformed axis "%s"'
                           % (name, axis_el.get('xyz')))
            elif sum(a * a for a in axis) < 1e-12:
                find.error('joint "%s" has a zero-length axis; the Fusion joint axis '
                           'did not survive the export' % name)

    if jtype in ('revolute', 'prismatic'):
        limit = joint.find('limit')
        if limit is None:
            find.error('joint "%s" is %s and must have a <limit>' % (name, jtype))
        else:
            lower = float(limit.get('lower', 0))
            upper = float(limit.get('upper', 0))
            if lower > upper:
                find.error('joint "%s" has lower=%g > upper=%g' % (name, lower, upper))
            elif lower == upper:
                find.warn('joint "%s" has lower == upper == %g, so it cannot move'
                          % (name, lower))
            if float(limit.get('effort', 0)) <= 0:
                find.warn('joint "%s" has effort=%s (the exporter\'s placeholder is 100); '
                          'set it from the servo datasheet' % (name, limit.get('effort')))

    return parent, child


def check_tree(links, joints, find):
    """A URDF must be a tree: one root, every other link with exactly one parent."""
    parent_of = {}
    for name, parent, child in joints:
        if child is None:
            continue
        if child in parent_of:
            find.error('link "%s" is the child of both "%s" and "%s". A URDF link may '
                       'have only one parent joint.' % (child, parent_of[child][0], name))
        else:
            parent_of[child] = (name, parent)

    roots = [l for l in links if l not in parent_of]
    if not roots:
        find.error('no root link: every link has a parent, so the model contains a cycle')
        return None
    if len(roots) > 1:
        find.error('%d disconnected roots (%s). A URDF must have exactly one. Every link '
                   'except the base needs a joint attaching it to the tree.'
                   % (len(roots), ', '.join(sorted(roots))))
    if 'base_link' in links and 'base_link' not in roots:
        find.error('"base_link" is not the root; it is the child of joint "%s"'
                   % parent_of['base_link'][0])

    # Cycle detection on the parent chain.
    for link in links:
        seen, cur = set(), link
        while cur in parent_of:
            if cur in seen:
                find.error('cycle in the kinematic chain at link "%s"' % cur)
                break
            seen.add(cur)
            cur = parent_of[cur][1]

    return sorted(roots)[0] if roots else None


def render_tree(root, joints):
    children = {}
    for name, parent, child in joints:
        children.setdefault(parent, []).append((child, name))
    lines = [root]

    def walk(link, depth, seen):
        if link in seen:
            lines.append('%s(cycle)' % ('  ' * depth))
            return
        for child, jname in sorted(children.get(link, [])):
            lines.append('%s+- [%s] %s' % ('  ' * depth, jname, child))
            walk(child, depth + 1, seen | {link})

    walk(root, 1, set())
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('urdf', help='path to the .xacro or .urdf file')
    ap.add_argument('--pkg-root', help='package root (default: the parent of urdf/)')
    args = ap.parse_args()

    path = os.path.abspath(args.urdf)
    if not os.path.isfile(path):
        print('no such file: %s' % path)
        return 2

    pkg_root = args.pkg_root or os.path.dirname(os.path.dirname(path))
    pkg_root = os.path.abspath(pkg_root)
    pkg_name = os.path.basename(pkg_root)

    find = Findings()
    elements = load(path, pkg_root, pkg_name, find)

    links = {}
    for el in elements:
        if el.tag != 'link':
            continue
        name = el.get('name', '<unnamed>')
        if name in links:
            find.error('link "%s" is defined more than once' % name)
        if BAD_NAME.search(name):
            find.error('link name "%s" contains whitespace or "/"' % name)
        links[name] = el

    joints = []
    joint_names = set()
    for el in elements:
        if el.tag != 'joint':
            continue
        name = el.get('name', '<unnamed>')
        if name in joint_names:
            find.error('joint "%s" is defined more than once' % name)
        joint_names.add(name)
        parent, child = check_joint(el, links, find)
        joints.append((name, parent, child))

    for name, el in links.items():
        check_inertial(name, el, find)
        check_meshes(name, el, pkg_root, pkg_name, find)

    root_link = None
    if links:
        root_link = check_tree(set(links), joints, find)
    else:
        find.error('no <link> elements found')

    print('urdf_lint: %s' % path)
    print('package root: %s (package "%s")' % (pkg_root, pkg_name))
    print('%d links, %d joints' % (len(links), len(joints)))
    print()

    if root_link and joints:
        print('kinematic tree')
        print('-' * 60)
        print(render_tree(root_link, joints))
        print()

    for label, items in (('ERRORS', find.errors), ('WARNINGS', find.warnings)):
        print('%s: %d' % (label, len(items)))
        print('-' * 60)
        for i, item in enumerate(items, 1):
            print('  %d. %s' % (i, item))
        print()

    if find.errors:
        print('FAIL -- %d error(s)' % len(find.errors))
        return 1
    print('PASS -- %d warning(s)' % len(find.warnings))
    return 0


if __name__ == '__main__':
    sys.exit(main())
