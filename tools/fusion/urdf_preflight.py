# -*- coding: utf-8 -*-
"""
URDF pre-flight for Fusion_URDF_Exporter_ROS2 (runtimerobotics/fusion360-urdf-ros2).

Read-only. Mutates nothing. Run it in Fusion before every export attempt.

Rather than guessing at rules, this re-implements the exporter's own logic
(core/Joint.py make_joints_dict, core/Link.py make_inertial_dict,
core/Write.py write_link_urdf / write_joint_urdf) and dry-runs the export,
so it reports the exact link the real exporter will die on and the exact
dialog you will see.

Install: Utilities -> Add-Ins -> Scripts and Add-Ins -> Scripts -> "+" ->
create a script named urdf_preflight, then replace its .py with this file.

Output: a summary message box, plus the full report written to
  <home>/urdf_preflight_report.txt
"""

import os
import re
import traceback

import adsk.core
import adsk.fusion

# Fusion JointTypes enum index -> the URDF type string the exporter assigns.
# (core/Joint.py: joint_type_list[joint.jointMotion.jointType])
JOINT_TYPE_LIST = ['fixed', 'revolute', 'prismatic', 'Cylinderical',
                   'PinSlot', 'Planner', 'Ball']
FUSION_JOINT_NAMES = ['Rigid', 'Revolute', 'Slider', 'Cylindrical',
                      'PinSlot', 'Planar', 'Ball']
# Only these three produce valid URDF. The rest are written verbatim as a
# bogus joint type and the URDF will not parse downstream.
SUPPORTED = (0, 1, 2)

REPORT_NAME = 'urdf_preflight_report.txt'


def san(name):
    """The exporter's name sanitizer (core/Joint.py, core/Link.py)."""
    return re.sub('[ :()]', '_', name)


class Report(object):
    def __init__(self):
        self.blockers = []   # export will fail or produce unusable URDF
        self.warnings = []   # export succeeds, output is wrong or awkward
        self.info = []

    def blocker(self, msg):
        self.blockers.append(msg)

    def warn(self, msg):
        self.warnings.append(msg)

    def note(self, msg):
        self.info.append(msg)

    def render(self):
        out = []
        out.append('URDF PRE-FLIGHT REPORT')
        out.append('=' * 72)
        out.append('')
        for title, items in (('BLOCKERS (export will fail)', self.blockers),
                             ('WARNINGS (export succeeds, output is wrong)', self.warnings),
                             ('INFO', self.info)):
            out.append('%s: %d' % (title, len(items)))
            out.append('-' * 72)
            if not items:
                out.append('  (none)')
            for i, item in enumerate(items, 1):
                out.append('  %d. %s' % (i, item))
            out.append('')
        return '\n'.join(out)


def collect_top_level(root, rep):
    """Mirror make_inertial_dict: keys the exporter will have for links."""
    inertial_keys = {}      # exporter key -> occurrence
    definition_counts = {}  # component name -> [occurrence names]
    sanitize_collisions = {}

    for occ in root.occurrences:
        key = 'base_link' if occ.component.name == 'base_link' else san(occ.name)

        if key in inertial_keys:
            rep.blocker('Two top-level occurrences collapse to the same link key '
                        '"%s": "%s" and "%s". One will silently overwrite the other.'
                        % (key, inertial_keys[key].name, occ.name))
        inertial_keys[key] = occ
        sanitize_collisions.setdefault(san(occ.name), []).append(occ.name)
        definition_counts.setdefault(occ.component.name, []).append(occ.name)

        if occ.bRepBodies.count == 0:
            rep.warn('Top-level component "%s" has NO bodies. copy_occs() skips it, '
                     'so no STL is exported, but make_inertial_dict still gives it a '
                     'link entry with mass 0 -- if a joint uses it you get a link with '
                     'zero mass and a missing mesh.' % occ.name)

        if occ.component.occurrences.count > 0:
            rep.warn('Top-level component "%s" contains %d nested component(s). '
                     'Only its own bodies are exported; nested components are dropped '
                     'from the mesh and are not valid joint targets.'
                     % (occ.name, occ.component.occurrences.count))

    for comp_name, occ_names in definition_counts.items():
        if len(occ_names) > 1:
            rep.blocker('SHARED DEFINITION x%d: component "%s" is instanced as %s. '
                        'These are Copy/Paste siblings of one definition -- renaming any '
                        'one renames all of them, so you cannot give them distinct link '
                        'names. Replace with Paste New, then rename each.'
                        % (len(occ_names), comp_name, ', '.join(occ_names)))

    for sanitized, originals in sanitize_collisions.items():
        if len(originals) > 1:
            rep.blocker('Name sanitization collision: %s all become "%s".'
                        % (', '.join('"%s"' % o for o in originals), sanitized))

    return inertial_keys


def build_joints_dict(root, rep):
    """Mirror make_joints_dict, recording what the exporter would produce."""
    joints = []  # list of dicts in the exporter's iteration order

    root_joint_tokens = set()
    for j in root.joints:
        root_joint_tokens.add(j.entityToken if hasattr(j, 'entityToken') else j.name)

    # Joints nested inside sub-assemblies are invisible to the exporter.
    try:
        for j in root.allJoints:
            token = j.entityToken if hasattr(j, 'entityToken') else j.name
            if token not in root_joint_tokens:
                rep.warn('Joint "%s" lives inside a sub-assembly. The exporter reads '
                         'root.joints only, so this joint is silently ignored.' % j.name)
    except Exception:
        pass

    for joint in root.joints:
        fusion_type = joint.jointMotion.jointType
        entry = {
            'name': joint.name,
            'fusion_type': fusion_type,
            'urdf_type': JOINT_TYPE_LIST[fusion_type],
            'ok': True,
        }

        if fusion_type not in SUPPORTED:
            rep.blocker('Joint "%s" is a %s joint. Only Rigid, Revolute and Slider are '
                        'supported; this writes type="%s" into the URDF, which will not '
                        'parse. Replace it.'
                        % (joint.name, FUSION_JOINT_NAMES[fusion_type],
                           JOINT_TYPE_LIST[fusion_type]))
            entry['ok'] = False

        if ' ' in joint.name:
            rep.warn('Joint name "%s" contains a space. The exporter does NOT sanitize '
                     'joint names -- it writes <joint name="%s"> verbatim. Rename it in '
                     'Fusion to snake_case (e.g. shoulder_pan_joint) before exporting.'
                     % (joint.name, joint.name))

        if fusion_type == 1:  # Revolute
            limits = joint.jointMotion.rotationLimits
            hi, lo = limits.isMaximumValueEnabled, limits.isMinimumValueEnabled
            if hi and not lo:
                rep.blocker('Joint "%s" has a maximum but no minimum limit. The exporter '
                            'aborts with "%sis not set its lower limit."'
                            % (joint.name, joint.name))
                entry['ok'] = False
            elif lo and not hi:
                rep.blocker('Joint "%s" has a minimum but no maximum limit. The exporter '
                            'aborts with "%sis not set its upper limit."'
                            % (joint.name, joint.name))
                entry['ok'] = False
            elif not hi and not lo:
                entry['urdf_type'] = 'continuous'
                rep.warn('Joint "%s" has no rotation limits, so it is exported as '
                         '"continuous" (unbounded). Set limits in Fusion if this axis '
                         'should not spin freely.' % joint.name)

        if fusion_type == 2:  # Slider
            limits = joint.jointMotion.slideLimits
            hi, lo = limits.isMaximumValueEnabled, limits.isMinimumValueEnabled
            if hi != lo:
                rep.blocker('Slider joint "%s" has only one of its two limits enabled; '
                            'the exporter aborts.' % joint.name)
                entry['ok'] = False

        occ_one, occ_two = joint.occurrenceOne, joint.occurrenceTwo
        if occ_one is None or occ_two is None:
            rep.blocker('Joint "%s" is attached to the root body rather than to two '
                        'components. Both sides must be top-level components.' % joint.name)
            entry['ok'] = False
            entry['child'] = entry['parent'] = None
            joints.append(entry)
            continue

        entry['child'] = san(occ_one.name)
        entry['child_occ'] = occ_one
        if occ_two.component.name == 'base_link':
            entry['parent'] = 'base_link'
        else:
            entry['parent'] = san(occ_two.name)
        entry['parent_occ'] = occ_two

        # base_link as a child is a special trap: make_inertial_dict files the
        # base under the key 'base_link', but the joint child is san(occ.name).
        if occ_one.component.name == 'base_link':
            rep.blocker('Joint "%s" uses base_link as Component1 (the child). '
                        'make_joints_dict names the child "%s" while make_inertial_dict '
                        'files the base under "base_link", so the export dies with '
                        '"Failed: %s". base_link must always be Component2.'
                        % (joint.name, san(occ_one.name), san(occ_one.name)))
            entry['ok'] = False

        for role, occ in (('Component1 (child)', occ_one), ('Component2 (parent)', occ_two)):
            if occ.assemblyContext is not None:
                rep.blocker('Joint "%s" %s is "%s", which is nested inside "%s" rather '
                            'than being a top-level component. It never gets a link '
                            'entry, so the export dies with "Failed: %s".'
                            % (joint.name, role, occ.name,
                               occ.assemblyContext.name, san(occ.name)))
                entry['ok'] = False

        try:
            _ = joint.geometryOrOriginTwo.origin.asArray()
        except Exception:
            try:
                g = joint.geometryOrOriginTwo
                if isinstance(g, adsk.fusion.JointOrigin):
                    _ = g.geometry.origin.asArray()
                else:
                    _ = g.origin.asArray()
            except Exception:
                rep.blocker('Joint "%s" has no usable joint origin; the exporter aborts '
                            'with "%s doesn\'t have joint origin."' % (joint.name, joint.name))
                entry['ok'] = False

        joints.append(entry)

    return joints


def dry_run_write(joints, inertial_keys, rep):
    """Mirror write_link_urdf then write_joint_urdf, in that order."""
    # --- Pass 1: write_link_urdf. base_link first, then every joint's child.
    if 'base_link' not in inertial_keys:
        rep.blocker('No component named exactly "base_link" at the top level. The '
                    'exporter stops with "There is no base_link."')
        return []

    links_written = ['base_link']
    seen_children = {}
    stopped_after = None

    for entry in joints:
        child = entry.get('child')
        if child is None:
            continue
        if child not in inertial_keys:
            if stopped_after is None:
                stopped_after = list(links_written)
                rep.blocker('EXPORT STOPS HERE: joint "%s" has child "%s", which has no '
                            'link entry. write_link_urdf raises KeyError and you get the '
                            'dialog "Failed: %s". Everything after this joint is missing '
                            'from the .xacro. Links written before the stop: %s.'
                            % (entry['name'], child, child, ', '.join(stopped_after)))
            else:
                rep.blocker('WOULD FAIL NEXT: joint "%s" has child "%s", which also has '
                            'no link entry. You will hit this one ("Failed: %s") as soon '
                            'as the stop above is fixed.'
                            % (entry['name'], child, child))
            # Keep simulating rather than returning, so that one run reports every
            # problem instead of only the first. The checks below therefore describe
            # the export you get once the blockers above are resolved.
            continue
        if child in seen_children:
            rep.blocker('Joints "%s" and "%s" both use "%s" as their child. A URDF link '
                        'may have exactly one parent joint; this writes the <link> twice '
                        'and makes the kinematic tree invalid.'
                        % (seen_children[child], entry['name'], child))
        seen_children[child] = entry['name']
        links_written.append(child)

    # --- Pass 2: write_joint_urdf. Every parent must have been written above.
    for entry in joints:
        parent = entry.get('parent')
        if parent is None:
            continue
        if parent not in links_written:
            hint = ''
            if parent in inertial_keys:
                hint = (' "%s" is a valid top-level component but is never any joint\'s '
                        'child, so it is not part of the kinematic tree.' % parent)
            rep.blocker('Joint "%s": parent "%s" was never written as a link, so '
                        'write_joint_urdf raises KeyError and you get the "swap '
                        'component1<=>component2" dialog.%s'
                        % (entry['name'], parent, hint))

    # --- Orphans: real components that simply never make it into the URDF.
    orphans = [occ.name for key, occ in inertial_keys.items()
               if key not in links_written and occ.bRepBodies.count > 0]
    if orphans:
        rep.blocker('%d component(s) with bodies are never any joint\'s child, so each '
                    'is exported as an STL and then dropped from the URDF entirely -- no '
                    'error, they just will not exist in RViz: %s. Each needs a joint '
                    '(Rigid for a servo case) with the segment as Component2.'
                    % (len(orphans), ', '.join(sorted(orphans))))

    return links_written


def check_physics(inertial_keys, links_written, rep):
    for key in links_written:
        occ = inertial_keys.get(key)
        if occ is None:
            continue
        try:
            prop = occ.getPhysicalProperties(
                adsk.fusion.CalculationAccuracy.LowCalculationAccuracy)
        except Exception:
            continue
        if prop.mass <= 1e-9:
            rep.blocker('Link "%s" has mass %g kg. A zero-mass link with a non-fixed '
                        'joint makes Gazebo reject the model.' % (key, prop.mass))
        elif prop.mass < 0.001:
            rep.warn('Link "%s" has mass %.6f kg (<1 g) -- almost certainly a missing '
                     'physical material.' % (key, prop.mass))


def describe_tree(joints, links_written):
    lines = ['KINEMATIC TREE (as the exporter would write it)', '-' * 72]
    children = {}
    for entry in joints:
        if entry.get('parent') and entry.get('child'):
            children.setdefault(entry['parent'], []).append(
                (entry['child'], entry['name'], entry['urdf_type']))

    reached = set()

    def walk(link, depth, seen):
        if link in seen:
            lines.append('%s%s  <-- CYCLE' % ('  ' * depth, link))
            return
        seen = seen | {link}
        for child, jname, jtype in children.get(link, []):
            reached.add(child)
            lines.append('%s+- [%s %s] %s' % ('  ' * depth, jtype, jname, child))
            walk(child, depth + 1, seen)

    lines.append('base_link')
    walk('base_link', 1, set())

    # Anything hanging off a parent that base_link never reaches. These are the
    # sub-chains that make the model several disconnected trees instead of one.
    detached = sorted(p for p in children if p != 'base_link' and p not in reached)
    if detached:
        lines.append('')
        lines.append('DETACHED from base_link -- these parents are not in the tree:')
        for parent in detached:
            if parent in reached:
                continue  # already shown under an earlier detached sub-chain
            lines.append('%s  (not a link)' % parent)
            walk(parent, 1, set())
    return '\n'.join(lines)


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('No active Fusion design.', 'URDF pre-flight')
            return

        root = design.rootComponent
        rep = Report()

        robot_name = root.name.split()[0]
        rep.note('Robot name will be "%s" and the package "%s_description" '
                 '(taken from root.name.split()[0]).' % (robot_name, robot_name))
        if root.name != robot_name:
            rep.warn('The design is named "%s" but the exporter takes only the first '
                     'whitespace-separated token, so the robot becomes "%s". Rename the '
                     'design if that is not what you want.' % (root.name, robot_name))

        inertial_keys = collect_top_level(root, rep)
        rep.note('%d top-level occurrences, %d root joints.'
                 % (root.occurrences.count, root.joints.count))

        joints = build_joints_dict(root, rep)
        links_written = dry_run_write(joints, inertial_keys, rep)
        check_physics(inertial_keys, links_written, rep)

        body = rep.render()
        body += '\n' + describe_tree(joints, links_written) + '\n'
        label = ('LINKS THE EXPORTER WOULD WRITE ONCE THE BLOCKERS ABOVE ARE FIXED'
                 if rep.blockers else 'LINKS THE EXPORTER WOULD WRITE')
        body += '\n%s (%d): %s\n' % (label, len(links_written), ', '.join(links_written))

        path = os.path.join(os.path.expanduser('~'), REPORT_NAME)
        try:
            with open(path, 'w') as f:
                f.write(body)
            where = 'Full report written to:\n%s' % path
        except Exception as e:
            where = 'Could not write the report file: %s' % e

        if rep.blockers:
            head = 'NOT READY TO EXPORT -- %d blocker(s), %d warning(s).\n\n' % (
                len(rep.blockers), len(rep.warnings))
            head += '\n\n'.join('%d. %s' % (i, b) for i, b in
                                enumerate(rep.blockers[:4], 1))
            if len(rep.blockers) > 4:
                head += '\n\n(+%d more -- see the report file.)' % (len(rep.blockers) - 4)
        else:
            head = ('READY TO EXPORT -- no blockers, %d warning(s).\n\n'
                    'The exporter would write %d links: %s'
                    % (len(rep.warnings), len(links_written), ', '.join(links_written)))

        ui.messageBox(head + '\n\n' + where, 'URDF pre-flight')

    except Exception:
        if ui:
            ui.messageBox('Pre-flight itself crashed:\n%s' % traceback.format_exc(),
                          'URDF pre-flight')
