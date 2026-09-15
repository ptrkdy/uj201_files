# -*- coding: utf-8 -*-
"""
urdf_extract -- read a Fusion assembly into a JSON robot model. Read-only.

Replaces the extraction half of Fusion_URDF_Exporter_ROS2. The differences that
matter:

  * Links are keyed by occurrence fullPathName, not by top-level component name.
    Nested components are therefore first-class links, and Copy/Paste siblings
    that share one component definition stay distinct. No flattening, no
    Paste New, no Break Link.
  * Joints come from root.allJoints, so joints inside sub-assemblies count.
  * Link names are generated (snake_case, de-versioned, de-duplicated) and can
    be overridden from a JSON file, so nothing has to be renamed in Fusion.
  * Nothing is mutated. The original's copy_occs() rebuilds your design; this
    only reads. STL export is opt-in and writes outside the document.

Output: <save_dir>/<robot>_model.json, consumed by tools/host/urdf_build.py.

Geometry convention matches the original exporter, which is known to work with
this toolchain: link frames stay axis-aligned with world, meshes are exported
in world position, and the visual/collision origin compensates. urdf_build.py
verifies that assumption against the actual STL bounds rather than trusting it.
"""

import json
import os
import re
import traceback

import adsk.core
import adsk.fusion

TITLE = 'URDF extract'
OVERRIDES_NAME = 'link_names.json'

# Fusion JointTypes index -> URDF type.
JOINT_TYPES = {0: 'fixed', 1: 'revolute', 2: 'prismatic'}
FUSION_JOINT_NAMES = ['Rigid', 'Revolute', 'Slider', 'Cylindrical',
                      'PinSlot', 'Planar', 'Ball']

VERSION_SUFFIX = re.compile(r'[ _-]v\d+$', re.IGNORECASE)
INSTANCE_SUFFIX = re.compile(r':\d+$')


def snake(name):
    """A ROS-safe snake_case name: no spaces, no punctuation, no version tail."""
    name = INSTANCE_SUFFIX.sub('', name)
    name = VERSION_SUFFIX.sub('', name)
    name = re.sub(r'[^0-9A-Za-z]+', '_', name)
    name = re.sub(r'_+', '_', name).strip('_').lower()
    return name or 'link'


def world_transform(occ):
    """World transform of an occurrence, valid at any depth."""
    try:
        return occ.transform2  # assembly-context aware where available
    except Exception:
        return occ.transform


def inertia_about_com(moments, com, mass):
    """Parallel-axis shift, same math as the original utils.origin2center_of_mass."""
    x, y, z = com
    shift = [y * y + z * z, x * x + z * z, x * x + y * y, -x * y, -y * z, -x * z]
    return [round(i - mass * t, 9) for i, t in zip(moments, shift)]


class Model(object):
    def __init__(self):
        self.links = {}      # fullPathName -> dict
        self.joints = []
        self.problems = []

    def problem(self, severity, msg):
        self.problems.append({'severity': severity, 'message': msg})


def load_overrides(save_dir):
    path = os.path.join(save_dir, OVERRIDES_NAME)
    if not os.path.isfile(path):
        return {}
    try:
        with open(path) as f:
            return json.load(f)
    except Exception:
        return {}


def assign_names(model, overrides):
    """Generate unique snake_case link names; explicit overrides always win."""
    used = set()

    # base_link first so it always keeps the name.
    order = sorted(model.links, key=lambda k: 0 if model.links[k]['is_base'] else 1)

    for key in order:
        link = model.links[key]
        if key in overrides:
            name = overrides[key]
        elif link['is_base']:
            name = 'base_link'
        else:
            name = snake(link['component'])

        if name in used:
            # Disambiguate shared definitions by their parent, then by index.
            parent = link['parent_path']
            candidate = '%s_%s' % (snake(parent.split('+')[-1]), name) if parent else name
            n = 1
            while candidate in used or candidate == name:
                n += 1
                candidate = '%s_%d' % (name, n)
            model.problem('info', 'Two occurrences both wanted the link name "%s"; '
                                  '"%s" became "%s". Override it in %s if you want '
                                  'something clearer.' % (name, key, candidate, OVERRIDES_NAME))
            name = candidate

        used.add(name)
        link['name'] = name


def add_link(model, occ, is_base=False):
    key = occ.fullPathName
    if key in model.links:
        return model.links[key]

    try:
        prop = occ.getPhysicalProperties(
            adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
        mass = prop.mass
        com = [v / 100.0 for v in prop.centerOfMass.asArray()]
        (_, xx, yy, zz, xy, yz, xz) = prop.getXYZMomentsOfInertia()
        moments = [v / 10000.0 for v in (xx, yy, zz, xy, yz, xz)]
        inertia = inertia_about_com(moments, com, mass)
    except Exception as e:
        model.problem('error', 'Could not read physical properties of "%s": %s'
                      % (occ.fullPathName, e))
        mass, com, inertia = 0.0, [0, 0, 0], [0] * 6

    # Bodies at this occurrence's own level plus everything below it: a link is
    # the whole subtree unless a descendant is itself a link (resolved later).
    body_count = occ.bRepBodies.count

    parent_path = ''
    try:
        if occ.assemblyContext is not None:
            parent_path = occ.assemblyContext.fullPathName
    except Exception:
        pass

    try:
        transform = list(world_transform(occ).asArray())
    except Exception:
        transform = None

    link = {
        'id': key,
        'name': None,
        'component': occ.component.name,
        'occurrence': occ.name,
        'parent_path': parent_path,
        'world_transform': transform,
        'depth': key.count('+'),
        'body_count': body_count,
        'mass': mass,
        'center_of_mass': [round(v, 9) for v in com],
        'inertia': inertia,
        'is_base': is_base,
    }
    model.links[key] = link
    model._occ_by_key[key] = occ

    if mass <= 1e-9:
        model.problem('error', 'Link "%s" has zero mass -- assign a physical material '
                               'in Fusion.' % key)
    if body_count == 0:
        model.problem('warn', 'Occurrence "%s" has no bodies of its own; its STL will '
                              'be empty unless its geometry sits deeper.' % key)
    return link


def read_joint(model, joint):
    ftype = joint.jointMotion.jointType
    entry = {'fusion_name': joint.name, 'name': snake(joint.name)}

    if ftype not in JOINT_TYPES:
        model.problem('error', 'Joint "%s" is a %s joint; URDF supports only Rigid, '
                               'Revolute and Slider here. Skipped.'
                      % (joint.name, FUSION_JOINT_NAMES[ftype]))
        return None
    entry['type'] = JOINT_TYPES[ftype]

    occ_one, occ_two = joint.occurrenceOne, joint.occurrenceTwo
    if occ_one is None or occ_two is None:
        model.problem('error', 'Joint "%s" is not between two components. Skipped.'
                      % joint.name)
        return None

    entry['child'] = occ_one.fullPathName
    entry['parent'] = occ_two.fullPathName
    entry['axis'] = [0.0, 0.0, 1.0]
    entry['lower'] = entry['upper'] = 0.0

    if ftype == 1:
        entry['axis'] = [round(v, 6) for v in
                         joint.jointMotion.rotationAxisVector.asArray()]
        limits = joint.jointMotion.rotationLimits
        hi, lo = limits.isMaximumValueEnabled, limits.isMinimumValueEnabled
        if hi and lo:
            entry['upper'] = round(limits.maximumValue, 6)
            entry['lower'] = round(limits.minimumValue, 6)
        else:
            # The original aborts the whole export here. Degrade instead: a
            # half-limited joint becomes continuous and is reported.
            entry['type'] = 'continuous'
            if hi or lo:
                model.problem('warn', 'Joint "%s" has only one rotation limit set, so '
                                      'it is exported as continuous. Set both limits in '
                                      'Fusion for a real revolute joint.' % joint.name)
    elif ftype == 2:
        entry['axis'] = [round(v, 6) for v in
                         joint.jointMotion.slideDirectionVector.asArray()]
        limits = joint.jointMotion.slideLimits
        if limits.isMaximumValueEnabled and limits.isMinimumValueEnabled:
            entry['upper'] = round(limits.maximumValue / 100.0, 6)
            entry['lower'] = round(limits.minimumValue / 100.0, 6)
        else:
            model.problem('error', 'Slider joint "%s" needs both limits set.' % joint.name)
            return None

    origin = joint_origin_world(joint)
    if origin is None:
        model.problem('error', 'Joint "%s" has no readable origin. Skipped.' % joint.name)
        return None
    entry['origin'] = origin

    # The origin heuristic above is inherited from the third-party exporter and
    # is only reliable near the base. Record every raw input so the correct
    # transform can be derived on the host, checked against real geometry,
    # without another round trip through Fusion.
    entry['raw'] = {
        'geometry_one': raw_point(joint.geometryOrOriginOne),
        'geometry_two': raw_point(joint.geometryOrOriginTwo),
        'occ_one_world': raw_matrix(world_transform(occ_one)),
        'occ_two_world': raw_matrix(world_transform(occ_two)),
        'occ_one_local': raw_matrix(occ_one.transform),
        'occ_two_local': raw_matrix(occ_two.transform),
    }
    return entry


def raw_point(geometry):
    """The unconverted origin of a joint geometry, in Fusion's cm."""
    if geometry is None:
        return None
    try:
        if isinstance(geometry, adsk.fusion.JointOrigin):
            return list(geometry.geometry.origin.asArray())
        return list(geometry.origin.asArray())
    except Exception:
        return None


def raw_matrix(transform):
    try:
        return list(transform.asArray())
    except Exception:
        return None


def joint_origin_world(joint):
    """Joint position in world metres.

    geometryOrOriginTwo.origin is ALREADY in root/world coordinates -- only the
    unit conversion is needed. The third-party exporter applied
    occurrenceTwo.transform on top of it, which double-transforms every joint
    whose occurrence is not at the origin; it survived only because a
    coincidence test skipped the transform for joints near the base.

    Verified empirically by tools/host/solve_joint_origins.py, which scored the
    raw value at a mean of 0.4 mm from both connected meshes against 284 mm for
    the transformed one.
    """
    for geometry in (joint.geometryOrOriginTwo, joint.geometryOrOriginOne):
        point = raw_point(geometry)
        if point is not None:
            return [round(v / 100.0, 6) for v in point]
    return None


def collect_rigid_groups(design, root, model):
    """Fusion keeps Rigid Groups outside root.joints, so read them separately.

    A rigid group says "these occurrences move as one body", which is exactly a
    set of URDF fixed joints. Members are recorded here and turned into joints
    by urdf_build.py, where the anchor can be chosen against the finished tree.
    """
    groups = []
    seen = set()

    try:
        components = list(design.allComponents)
    except Exception:
        components = [root]

    for comp in components:
        try:
            rigid_groups = comp.rigidGroups
        except Exception:
            continue
        for i in range(rigid_groups.count):
            rg = rigid_groups.item(i)
            try:
                members = [rg.occurrences.item(j) for j in range(rg.occurrences.count)]
            except Exception:
                continue
            if len(members) < 2:
                continue
            key = tuple(sorted(o.fullPathName for o in members))
            if key in seen:
                continue
            seen.add(key)
            for occ in members:
                add_link(model, occ)
            groups.append({'name': rg.name or ('rigid_group_%d' % len(groups)),
                           'members': [o.fullPathName for o in members]})
    return groups


def find_base(root, model):
    for occ in root.allOccurrences:
        if occ.component.name == 'base_link':
            return occ
    model.problem('error', 'No component named "base_link" anywhere in the assembly.')
    return None


def export_meshes(design, model, save_dir):
    """One STL per link, exported straight from the occurrence at any depth."""
    mesh_dir = os.path.join(save_dir, 'meshes')
    try:
        os.makedirs(mesh_dir)
    except Exception:
        pass

    mgr = design.exportManager
    for key, link in model.links.items():
        occ = model._occ_by_key.get(key)
        if occ is None:
            continue
        path = os.path.join(mesh_dir, link['name'] + '.stl')
        try:
            opts = mgr.createSTLExportOptions(occ, path)
            opts.sendToPrintUtility = False
            opts.isBinaryFormat = True
            opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
            mgr.execute(opts)
            link['mesh'] = 'meshes/%s.stl' % link['name']
        except Exception as e:
            model.problem('error', 'STL export failed for "%s": %s' % (key, e))
            link['mesh'] = None


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('No active Fusion design.', TITLE)
            return

        root = design.rootComponent
        robot_name = snake(root.name)

        folder = ui.createFolderDialog()
        folder.title = 'Where should the robot model be written?'
        if folder.showDialog() != adsk.core.DialogResults.DialogOK:
            return
        save_dir = folder.folder

        want_stl = ui.messageBox(
            'Export STL meshes as well?\n\nYes: full export (slower).\n'
            'No: JSON model only -- enough to validate the structure with '
            'urdf_build.py and urdf_lint.py.',
            TITLE, adsk.core.MessageBoxButtonTypes.YesNoButtonType) \
            == adsk.core.DialogResults.DialogYes

        model = Model()
        model._occ_by_key = {}

        base_occ = find_base(root, model)
        if base_occ is None:
            ui.messageBox('No base_link found. Name the root component of your robot '
                          '"base_link" and run again.', TITLE)
            return
        add_link(model, base_occ, is_base=True)

        # Links are exactly the occurrences the joints refer to, at any depth.
        raw_joints = []
        for joint in root.allJoints:
            entry = read_joint(model, joint)
            if entry is None:
                continue
            for occ in (joint.occurrenceOne, joint.occurrenceTwo):
                add_link(model, occ, is_base=(occ.fullPathName == base_occ.fullPathName))
            raw_joints.append(entry)

        # Rigid Groups live outside root.joints; their members become links too.
        rigid_groups = collect_rigid_groups(design, root, model)

        assign_names(model, load_overrides(save_dir))

        by_id = {k: v['name'] for k, v in model.links.items()}
        for entry in raw_joints:
            entry['parent_id'], entry['child_id'] = entry['parent'], entry['child']
            entry['parent'] = by_id.get(entry['parent_id'], entry['parent_id'])
            entry['child'] = by_id.get(entry['child_id'], entry['child_id'])
        model.joints = raw_joints

        # Bodied occurrences nobody jointed: real geometry that will not appear.
        jointed = set()
        for entry in model.joints:
            jointed.add(entry['parent_id'])
            jointed.add(entry['child_id'])
        for group in rigid_groups:
            jointed.update(group['members'])
        for occ in root.allOccurrences:
            if occ.bRepBodies.count and occ.fullPathName not in jointed \
                    and occ.fullPathName != base_occ.fullPathName:
                if any(occ.fullPathName.startswith(j + '+') for j in jointed):
                    continue  # geometry belonging to a link above it
                model.problem('warn', 'Occurrence "%s" has bodies but no joint, so it '
                                      'will not appear in the URDF.' % occ.fullPathName)

        if want_stl:
            export_meshes(design, model, save_dir)

        for group in rigid_groups:
            group['member_names'] = [by_id.get(m, m) for m in group['members']]

        doc = {
            'robot_name': robot_name,
            'rigid_groups': rigid_groups,
            'source_document': root.name,
            'units': {'length': 'm', 'mass': 'kg'},
            'mesh_frame': 'local',  # a read-only occurrence export is in the occurrence's own frame
            'links': [model.links[k] for k in model.links],
            'joints': model.joints,
            'problems': model.problems,
        }
        out_path = os.path.join(save_dir, '%s_model.json' % robot_name)
        with open(out_path, 'w') as f:
            json.dump(doc, f, indent=2)

        errors = [p for p in model.problems if p['severity'] == 'error']
        warns = [p for p in model.problems if p['severity'] == 'warn']
        summary = ('Extracted %d links, %d joints and %d rigid group(s).\n\n'
                   '%d error(s), %d warning(s).\n\n'
                   'Written to:\n%s\n\nNext: run tools/host/urdf_build.py on that file.'
                   % (len(model.links), len(model.joints), len(rigid_groups),
                      len(errors), len(warns), out_path))
        if errors:
            summary += '\n\nFirst errors:\n' + '\n'.join(
                '  - ' + p['message'] for p in errors[:5])
        ui.messageBox(summary, TITLE)

    except Exception:
        if ui:
            ui.messageBox('urdf_extract crashed:\n%s' % traceback.format_exc(), TITLE)
