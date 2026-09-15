# uj201 — Fusion 360 → ROS 2 URDF

Tooling built to export a custom robot arm (**UJ201**, a heavily modified S100)
from Autodesk Fusion 360 into a ROS 2 description package — after the usual
third-party exporter turned out to need the CAD reshaped around its limitations.

Rather than reshape the model, this reads the assembly as it actually is.

## Why not the existing exporter

[`runtimerobotics/fusion360-urdf-ros2`](https://github.com/runtimerobotics/fusion360-urdf-ros2)
requires every link to be a **top-level, bodies-only component**, silently skips
nested components, cannot give distinct names to Copy/Paste siblings sharing one
component definition, and rewrites your design in place while exporting.

All of that traces to one decision: it keys links off **component** names. Fusion
has a strict class/instance split — `Component` is the class, `Occurrence` is the
instance, and `occ.fullPathName` uniquely identifies an instance at any depth.
Key off the occurrence instead and nesting and shared definitions both stop being
problems. No flattening, no Break Link, no Paste New, no renaming.

## Pipeline

    # 1. in Fusion: Utilities -> Scripts and Add-Ins -> urdf_extract -> Run
    #    read-only; writes <robot>_model.json (+ meshes/ if you ask for STLs)

    # 2. anywhere, no Fusion and no ROS required:
    python tools/host/urdf_build.py arm_assembly_model.json -o ./out \
        --overrides overrides.json --mesh-frame local
    python tools/host/urdf_lint.py out/arm_assembly_description/urdf/arm_assembly.xacro

    # 3. look at it -- no ROS needed
    cd out/arm_assembly_description && python -m http.server 8765
    #    then open http://127.0.0.1:8765/viewer.html

Extraction and generation are deliberately separate: `urdf_build.py` is plain
Python, so URDF output can be iterated on in seconds instead of re-exporting from
CAD for every change.

## Tools

| tool | runs where | mutates? | what it does |
|---|---|---|---|
| `tools/fusion/urdf_extract.py` | Fusion | no | reads the assembly into a JSON robot model (+ STLs) |
| `tools/fusion/urdf_preflight.py` | Fusion | no | dry-runs the *third-party* exporter and reports every way it will fail, at once |
| `tools/fusion/urdf_flatten.py` | Fusion | **yes** | copies nested bodies into flat top-level components (only needed for the third-party path) |
| `tools/host/urdf_build.py` | anywhere | no | JSON model → ROS 2 package, plain `.urdf`, and a browser viewer |
| `tools/host/urdf_lint.py` | anywhere | no | validates a URDF/xacro without ROS installed |
| `tools/host/mesh_split.py` | anywhere | no | plane-cuts a mesh and integrates mass, centre of mass and inertia |
| `tools/host/solve_joint_origins.py` | anywhere | no | derives the correct joint-origin transform by scoring candidates against geometry |

Anything that mutates runs on a **backup copy** of the design, never the master.

## overrides.json

Corrections that belong in version control rather than in CAD — things Fusion
cannot know, or cannot express:

- **`links`** — per-link mass and inertia. Surface-modelled stand-ins report zero
  mass, so servo masses come from the datasheet here.
- **`reverse_joints`** — flip a joint whose Component1/Component2 point against
  the tree. Swaps parent and child and negates the limits.
- **`split_links`** — split a body that carries two joints. `from_mesh` takes a
  CAD split exported as two shells in one STL, separates them by connectivity and
  restores each to the parent frame automatically. `bind: true` bonds the halves
  rigidly instead of hinging them.
- **`rigid_groups`** / **`detach_links`** — extra fixed relationships, or
  corrections where the CAD grouping and the kinematics disagree.

## Things that cost real time to work out

- **STL export of an occurrence is in the occurrence's LOCAL frame**, not world.
  Proven by nine shared-definition servos exporting identical bounding boxes. The
  third-party exporter's meshes are world-positioned only because its `copy_occs()`
  mutation makes them so. Local meshes need real `rpy` from the world transform.
- **`geometryOrOriginTwo.origin` is already in world coordinates.** The existing
  exporter applies `occurrenceTwo.transform` on top, double-transforming every
  joint; it looks correct only near the base. Verified empirically — the raw value
  lands a mean of 0.4 mm from both connected meshes, the transformed one 284 mm.
- **Rigid Group ≠ Rigid Joint.** They live in different collections
  (`component.rigidGroups` vs `root.joints`). Alignment constraints are a third
  thing and reach the URDF not at all.
- **`getPhysicalProperties()` on a container includes its children**, so a wrapper
  and its contents both being links double-counts mass.
- **Fusion's default material is steel** (7849 kg/m³). Printed parts are ~6× lighter.
- **A link may be the parent of many joints, but the child of only one.** A body
  that appears to need splitting often just has a joint pointing the wrong way.

See [docs/exporter_internals.md](docs/exporter_internals.md) for the third-party
exporter's internals and its five failure modes.

## Status

Model is structurally complete: 23 links, 22 joints, one tree, lint clean, every
joint verified to drive the right body. Still to do — real materials instead of
Fusion's default steel, ROS 2 for RViz and Gazebo, collision hulls, `ros2_control`.
