# Fusion_URDF_Exporter_ROS2 internals

Read off the source at `../fusion360-urdf-ros2` (commit `e317c5e`), not from the
README. This is what the exporter actually does, in order, and exactly how each
failure mode is produced. It supersedes the guesswork in the project spec.

## Execution order (`Fusion_URDF_Exporter_ROS2.py:run`)

1. `robot_name = root.name.split()[0]` — **only the first whitespace token** of the
   design name. `package_name = robot_name + '_description'`.
2. Three dialogs: welcome, folder browse, Gazebo version.
   Harmonic → `write_urdf_sim` (emits `.ros2control`); Classic → `write_urdf`
   (emits `.trans`). *Our partial file includes `.ros2control`, so the failed run
   answered "Yes / Harmonic".*
3. `Joint.make_joints_dict(root, msg)` — may abort with a message.
4. `Link.make_inertial_dict(root, msg)` — then a check for the `base_link` key.
5. `Write.write_urdf_sim(...)` → writes the `.xacro`. **This is where it dies.**
6. …the remaining `Write.*` and `utils.*` calls, then `copy_occs` + `export_stl`.

Because step 5 runs before step 6, a crash there leaves a truncated `.xacro`
**and no meshes at all**.

## The two dictionaries

Everything hinges on these keys matching. `san(s) = re.sub('[ :()]', '_', s)`.

### `inertial_dict` — the set of things that can be a link
Built from `root.occurrences` (**top level only** — nested components are never
enumerated, which is why they "get silently skipped").

| condition | key |
|---|---|
| `occ.component.name == 'base_link'` | `'base_link'` |
| otherwise | `san(occ.name)` — the *occurrence* name, so `"p02_new_base_neck:1"` → `"p02_new_base_neck_1"` |

Note it does **not** skip body-less occurrences: those still get an entry, with
mass 0 and no STL.

### `joints_dict` — built from `root.joints`
- Key: `joint.name`, **unsanitized**. A Fusion joint called `Revolute 5` becomes
  `<joint name="Revolute 5">`.
- `child  = san(joint.occurrenceOne.name)` → Component1
- `parent = 'base_link'` if `occurrenceTwo.component.name == 'base_link'`,
  else `san(joint.occurrenceTwo.name)` → Component2
- Type comes from `['fixed','revolute','prismatic','Cylinderical','PinSlot','Planner','Ball'][jointMotion.jointType]`.
  Only indices 0–2 (Rigid/Revolute/Slider) are real URDF types; the other four are
  written verbatim and produce a URDF that will not parse.
- A Revolute joint with **neither** limit enabled is silently downgraded to
  `continuous`. With exactly **one** limit enabled the exporter aborts.
- `root.joints`, not `root.allJoints` — joints created inside a sub-assembly are
  invisible to the exporter.

## The five ways it fails

### 1. `There is no base_link.`
No top-level occurrence whose *component* is named exactly `base_link`.

### 2. `Failed: <link_name>`  ← *what bit us*
`write_link_urdf` writes `base_link`, then walks `joints_dict` doing
`inertial_dict[joints_dict[joint]['child']]`. A missing key raises a bare
`KeyError`, caught only by `run()`'s `except Exception as e:` →
`ui.messageBox(f'Failed:\n{str(e)}')`. `str(KeyError)` is just the key, so
**the dialog text is the name of the offending child occurrence**.

Causes: the child is nested inside another component; the child is `base_link`
itself used as Component1 (child key `base_link_1` ≠ stored key `base_link`).

The partial `.xacro` tells you how far it got: `base_link`,
`p02_new_base_neck_1`, `p10_new_elbow_v1_1` — so the **third** joint in dict
order has the bad child.

### 3. `There seems to be an error with the connection between…`
`write_joint_urdf` needs `links_xyz_dict[parent]`, which only contains
`base_link` plus every joint's child. So this fires when a joint's parent is
never any joint's child — i.e. parent/child swapped (Component1/Component2
backwards), or the parent is the root of a disconnected sub-chain. Note this
calls `quit()`, which raises `SystemExit` and is *not* caught by `run()`'s
`except Exception`.

### 4. Silent omission
A top-level component with bodies that is never a joint's **child** gets an STL
exported and no `<link>` in the URDF. This is what would happen to every servo
today — they have no joints at all.

### 5. Shared component definitions
Fusion occurrence names are `component.name + ':' + instance`, so renaming any
one instance renames the shared definition and therefore *all* of them. The
exporter reads occurrence names, so Copy→Paste siblings can only ever be
`servo_x_1`, `servo_x_2`, … Paste New creates an independent definition and
breaks the link. (`copy_occs` does give each occurrence its own new component
named `san(occ.name)`, so the export does not crash — you just cannot get
semantic names.)

## Things that are correct and should not be "fixed"

- Visual/collision origins are the negative of the link position
  (`Link.__init__`: `self.xyz = [-_ for _ in xyz]`) because meshes are exported
  in world position. Odd-looking offsets like `-0.158945 0.256118 -0.270818` are
  correct by construction.
- `scale="0.001 0.001 0.001"` — Fusion exports STL in mm, URDF is in m.
- `effort="100" velocity="100"` are hardcoded placeholders, not measurements.

## Model mutation

`utils.copy_occs(root)` rebuilds every bodied top-level occurrence as a new
component and renames the originals to `old_component`. `export_stl` then skips
anything whose component name contains `old_component`. This is destructive:
run on a backup copy and never save afterwards.
