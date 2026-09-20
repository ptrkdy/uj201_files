# -*- coding: utf-8 -*-
"""
fusion_export_all -- export every design in the active Fusion project to disk.

Exporting by hand is one File -> Export dialog per design; this walks the
project's folder tree, opens each Fusion design in turn, writes the formats
below, and closes it again. The project folder structure is mirrored in the
output so a nested project does not collapse into one flat pile.

Formats, and why all three rather than just the native one:

  .f3z   Fusion archive. Keeps history and bundles every referenced design, so
         one archive of an assembly carries its components with it. Only Fusion
         opens it -- fine as your own backup, useless to anyone without a
         licence.
  .step  Neutral interchange. FreeCAD, Onshape, SolidWorks and the rest read
         it. This is the one that makes published CAD actually modifiable.
  .stl   Mesh. Off by default: an STL of a whole assembly is a single merged
         shell, which is rarely what you want. Turn it on for single-part
         designs you intend to print.

Nothing is modified: each document is opened read-only as far as this script
is concerned and closed without saving.

Install: Utilities -> Add-Ins -> Scripts and Add-Ins -> Scripts -> "+",
create a script named fusion_export_all, then replace its .py with this file.
"""

import os
import re
import traceback

import adsk.core
import adsk.fusion

TITLE = 'Export all designs'

EXPORT_F3Z = True
EXPORT_STEP = True
EXPORT_STL = False          # see the note above

# Windows forbids these in filenames; Fusion allows them in design names.
ILLEGAL = re.compile(r'[<>:"/\\|?*\x00-\x1f]')


def safe_name(name):
    """A design name that survives being used as a filename."""
    cleaned = ILLEGAL.sub('_', name).strip().rstrip('.')
    return cleaned or 'unnamed'


def collect(folder, path, found, problems):
    """Depth-first walk of a project folder, gathering Fusion designs."""
    try:
        for i in range(folder.dataFiles.count):
            data_file = folder.dataFiles.item(i)
            # Projects hold uploaded meshes, images and PDFs too; only Fusion
            # designs can be opened as a Design product.
            if (data_file.fileExtension or '').lower() != 'f3d':
                continue
            found.append((path, data_file))
    except Exception as exc:
        problems.append('could not list files in %s: %s' % (path or '/', exc))

    try:
        for i in range(folder.dataFolders.count):
            child = folder.dataFolders.item(i)
            collect(child, os.path.join(path, safe_name(child.name)), found,
                    problems)
    except Exception as exc:
        problems.append('could not list folders in %s: %s' % (path or '/', exc))


def export_one(app, data_file, out_dir, written, problems):
    """Open one design, write the enabled formats, close it again."""
    document = None
    try:
        document = app.documents.open(data_file, True)
        if document is None:
            problems.append('%s: would not open' % data_file.name)
            return
        design = adsk.fusion.Design.cast(
            document.products.itemByProductType('DesignProductType'))
        if design is None:
            problems.append('%s: not a design' % data_file.name)
            return

        manager = design.exportManager
        stem = os.path.join(out_dir, safe_name(data_file.name))

        if EXPORT_F3Z:
            try:
                manager.execute(
                    manager.createFusionArchiveExportOptions(stem + '.f3z'))
                written.append(stem + '.f3z')
            except Exception as exc:
                problems.append('%s: f3z failed (%s)' % (data_file.name, exc))

        if EXPORT_STEP:
            try:
                try:
                    options = manager.createSTEPExportOptions(stem + '.step')
                except Exception:
                    # Some releases require the geometry argument explicitly.
                    options = manager.createSTEPExportOptions(
                        stem + '.step', design.rootComponent)
                manager.execute(options)
                written.append(stem + '.step')
            except Exception as exc:
                problems.append('%s: step failed (%s)' % (data_file.name, exc))

        if EXPORT_STL:
            try:
                options = manager.createSTLExportOptions(
                    design.rootComponent, stem + '.stl')
                options.sendToPrintUtility = False
                options.isBinaryFormat = True
                options.meshRefinement = \
                    adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
                manager.execute(options)
                written.append(stem + '.stl')
            except Exception as exc:
                problems.append('%s: stl failed (%s)' % (data_file.name, exc))

    except Exception as exc:
        problems.append('%s: %s' % (data_file.name, exc))
    finally:
        if document is not None:
            try:
                document.close(False)      # never save: this script only reads
            except Exception:
                pass


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        project = app.data.activeProject
        if project is None:
            ui.messageBox('No active project. Open one in the Data Panel first.',
                          TITLE)
            return

        formats = [name for name, on in (('.f3z', EXPORT_F3Z),
                                         ('.step', EXPORT_STEP),
                                         ('.stl', EXPORT_STL)) if on]
        if not formats:
            ui.messageBox('Every format is switched off at the top of the '
                          'script; nothing to do.', TITLE)
            return

        found, problems = [], []
        collect(project.rootFolder, '', found, problems)
        if not found:
            ui.messageBox('No Fusion designs found in project "%s".'
                          % project.name, TITLE)
            return

        folder_dialog = ui.createFolderDialog()
        folder_dialog.title = 'Where should the exports go?'
        if folder_dialog.showDialog() != adsk.core.DialogResults.DialogOK:
            return
        root_out = folder_dialog.folder

        confirm = ('Project "%s": %d design(s) found.\n\n'
                   'Each will be opened, exported as %s, and closed without '
                   'saving. Nothing in the project is modified.\n\n'
                   'Opening designs one by one is slow -- expect a few seconds '
                   'each.\n\nExport to:\n%s\n\nProceed?'
                   % (project.name, len(found), ', '.join(formats), root_out))
        if ui.messageBox(confirm, TITLE,
                         adsk.core.MessageBoxButtonTypes.OKCancelButtonType) \
                != adsk.core.DialogResults.DialogOK:
            return

        progress = ui.createProgressDialog()
        progress.isCancelButtonShown = True
        progress.show(TITLE, 'Exporting %v of %m: %p%', 0, len(found))

        written = []
        for index, (relative, data_file) in enumerate(found):
            if progress.wasCancelled:
                problems.append('cancelled after %d of %d' % (index, len(found)))
                break
            progress.message = ('Exporting %%v of %%m: %s' % data_file.name)
            out_dir = os.path.join(root_out, relative) if relative else root_out
            try:
                os.makedirs(out_dir)
            except Exception:
                pass
            export_one(app, data_file, out_dir, written, problems)
            progress.progressValue = index + 1
        progress.hide()

        summary = ['Wrote %d file(s) from %d design(s) into:\n%s'
                   % (len(written), len(found), root_out)]
        if problems:
            summary.append('\n%d problem(s):' % len(problems))
            summary.extend('  - ' + p for p in problems[:12])
            if len(problems) > 12:
                summary.append('  (+%d more)' % (len(problems) - 12))
        else:
            summary.append('\nNo problems.')
        ui.messageBox('\n'.join(summary), TITLE)

    except Exception:
        if ui:
            ui.messageBox('fusion_export_all crashed:\n%s'
                          % traceback.format_exc(), TITLE)
