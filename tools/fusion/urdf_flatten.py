# -*- coding: utf-8 -*-
"""
urdf_flatten -- harvest nested bodies into flat, local, top-level components.

The exporter reads each top-level occurrence's own bRepBodies and nothing
deeper, so a component whose geometry lives one level down exports as an empty
link (or kills the export outright). Fusion's own fix for that is
Break Link -> Ungroup From Parent, but Ungroup is disabled across a linked
design boundary, which is where this project keeps getting stuck.

This sidesteps the link entirely. For each offending top-level occurrence it
creates a NEW local component at the root and copies every body found at any
depth into it, using the same BRepBody.copyToComponent call the exporter itself
relies on. The linked original is never modified, so Fusion has nothing to
refuse. Body copies land in world position, which is what the exporter expects.

What it does NOT do: recreate joints. A joint that referenced a nested
occurrence has to be rebuilt against the new flat component by hand. Run
"Select referencing Joints" on the original first so you know the list.

Run it on a BACKUP copy of the design. Originals are hidden, not deleted, so
you can rebuild joints with both visible and then delete the originals
yourself.

Install: copy to
  %appdata%/Autodesk/Autodesk Fusion 360/API/Scripts/urdf_flatten/
alongside a urdf_flatten.manifest.
"""

import re
import traceback

import adsk.core
import adsk.fusion

FLAT_SUFFIX = '_flat'


def san(name):
    """Match the exporter's sanitizer so the new names survive export intact."""
    return re.sub('[ :()]', '_', name)


def descendant_bodies(occ):
    """Every body at any depth below occ, excluding occ's own bodies."""
    found = []
    for child in occ.childOccurrences:
        for i in range(child.bRepBodies.count):
            found.append(child.bRepBodies.item(i))
        found.extend(descendant_bodies(child))
    return found


def find_wrappers(root):
    """Top-level occurrences whose geometry is (partly) one or more levels down."""
    wrappers = []
    for occ in root.occurrences:
        if occ.name.endswith(FLAT_SUFFIX + ':1'):
            continue  # already one of ours
        nested = descendant_bodies(occ)
        if nested:
            wrappers.append((occ, occ.bRepBodies.count, nested))
    return wrappers


def flatten(root, occ, own_count, nested, log):
    """Build a flat local component holding every body from occ's subtree."""
    target_name = san(occ.name).rstrip('_123456789').rstrip('_') + FLAT_SUFFIX

    new_occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    new_occ.component.name = target_name

    copied = 0
    # Snapshot first: copying mutates the collections being walked.
    own = [occ.bRepBodies.item(i) for i in range(own_count)]
    for body in own + nested:
        try:
            body.copyToComponent(new_occ)
            copied += 1
        except Exception as e:
            log.append('  ! could not copy body "%s": %s' % (body.name, e))

    if copied == 0:
        try:
            new_occ.deleteMe()
        except Exception:
            pass
        log.append('  ! nothing copied from "%s"; new component removed' % occ.name)
        return None

    try:
        occ.isLightBulbOn = False
    except Exception:
        pass

    log.append('  "%s" -> "%s"  (%d body/bodies: %d own + %d nested)'
               % (occ.name, target_name, copied, own_count, len(nested)))
    return new_occ


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('No active Fusion design.', 'URDF flatten')
            return

        root = design.rootComponent
        wrappers = find_wrappers(root)

        if not wrappers:
            ui.messageBox('No top-level component has bodies nested below it. '
                          'Nothing to flatten.', 'URDF flatten')
            return

        preview = '\n'.join(
            '  %s  -- %d own body/bodies, %d nested'
            % (occ.name, own, len(nested)) for occ, own, nested in wrappers)
        confirm = (
            'This MODIFIES the design. Run it on a backup copy only.\n\n'
            '%d component(s) have geometry nested below the top level:\n\n%s\n\n'
            'For each one a new local component "<name>%s" will be created at the '
            'root holding copies of every body, and the original will be hidden '
            '(not deleted).\n\n'
            'Joints are NOT recreated -- you will need to rebuild any joint that '
            'referenced a nested component.\n\nProceed?'
            % (len(wrappers), preview, FLAT_SUFFIX))

        if ui.messageBox(confirm, 'URDF flatten',
                         adsk.core.MessageBoxButtonTypes.OKCancelButtonType,
                         adsk.core.MessageBoxIconTypes.WarningIconType) \
                != adsk.core.DialogResults.DialogOK:
            return

        log = []
        made = 0
        for occ, own_count, nested in wrappers:
            if flatten(root, occ, own_count, nested, log) is not None:
                made += 1

        ui.messageBox(
            'Flattened %d of %d component(s).\n\n%s\n\n'
            'Next: rebuild the joints against the new *%s components, delete the '
            'hidden originals, then re-run urdf_preflight.'
            % (made, len(wrappers), '\n'.join(log), FLAT_SUFFIX),
            'URDF flatten')

    except Exception:
        if ui:
            ui.messageBox('urdf_flatten crashed:\n%s' % traceback.format_exc(),
                          'URDF flatten')
