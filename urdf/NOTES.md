# Modelling notes

Two things in this description do not correspond one-to-one with the Fusion
assembly. Both are recorded here because they are not guessable from the URDF.

## 1. The U-joint is the shoulder, and `Revolute 18` was reversed

In Fusion, `Revolute 18` had `handles_prototype` as Component1 (child) and
`servo_motor_initial_draft:4` as Component2 (parent). That servo lives inside
`p09_prototype_u_bridge` — and the bridge also carries the servo that drives the
elbow, so the entire rest of the arm hangs off it.

Read literally, that says *the handle hangs off the bridge*. The truth is the
reverse: **the bridge hangs off the handle**. Which side of a Fusion joint is
Component1 is a modelling choice, not a kinematic fact, and here it pointed
against the tree.

Reversing it (`reverse_joints` in `overrides.json`) swaps parent and child and
negates the limits, since rotating A relative to B by θ is rotating B relative
to A by −θ. The result is that the two U-joint axes become a **2-DOF shoulder**
carrying the whole arm:

    base_link
      └─ servo_motor_initial_draft ─[revolute_5]→ p02_new_base_neck
           └─ servo_motor_initial_draft_7 ─[revolute_17]→ uj_yoke_in
                └─ [fixed] uj_yoke_out
                     └─ [revolute_18]→ servo_motor_initial_draft_8
                          ├─ p09_prototype_u_bridge
                          └─ servo_motor_initial_draft_2 ─[revolute_11]→ p10_new_elbow
                               └─ … elbow → forearm → wrist → end effector → thumb

Before this, `revolute_18` visibly drove the wrong body — it swung the handle
instead of the housing.

## 2. `uj_yoke_in` / `uj_yoke_out` are one physical part

`handles_prototype` was a single CAD body carrying **both** shoulder axes. A URDF
is a tree: a link may be the *parent* of many joints, but the *child* of only
one. A body that is the child of two joints cannot be expressed at all.

It was split in Fusion (Split Body) and both halves exported into one STL as two
disconnected shells; `split_links.from_mesh` separates them by vertex
connectivity and restores each to the parent frame. The halves are **bonded with
a fixed joint** (`uj_yoke_in_to_uj_yoke_out`), so kinematically they remain one
rigid body.

Worth being straight about: once `Revolute 18` was reversed, the split was no
longer strictly necessary — the reversal alone removes the two-parents problem.
The split is kept because it exists in the CAD and the two halves carry accurate
independent mass properties, integrated from their own geometry:

| link | mass | share |
|---|---|---|
| `uj_yoke_in` | 0.22317 kg | 43% |
| `uj_yoke_out` | 0.29566 kg | 57% |
| original `handles_prototype` | 0.51884 kg | — |

If you would rather see one link, drop `split_links` from `overrides.json` and
rebuild; the kinematics are unchanged.

An earlier attempt cut the body with a computed plane at the midpoint between
the two axes. That was wrong twice over: the real parting line is at 43/57, not
50/50, and interlocking yoke arms cannot be separated by any plane. The CAD
split is the authority.

## 3. `base_footprint` is a massless dummy root

`base_link` has an inertia, and `kdl_parser` warns:

    The root link base_link has an inertia specified in the URDF, but KDL does
    not support a root link with an inertia.

KDL then *ignores* it — so every KDL consumer (IK, dynamics) would silently work
from a model whose base has no mass. The conventional fix is an empty link above
the real base, joined by a fixed joint:

    <link name="base_footprint"/>

It carries no inertial and no geometry, which is the whole point. With it in
place `robot_state_publisher` loads the description with zero warnings.

Added by `dummy_root` in `overrides.json`; drop that key for a description
rooted directly at `base_link`.

## Caveats carried by this description

- Masses come from Fusion's default material, **steel** (7849 kg/m³). Printed
  parts are roughly 6× lighter. Fine for RViz, wrong for dynamics.
- `effort` and `velocity` on every joint are the placeholder `100`, not measured.
- Collision geometry is the visual mesh. For Gazebo, substitute convex hulls.
- The servo stand-ins are surface bodies, so Fusion reports zero mass for them;
  their masses come from `overrides.json` at ~60 g (Feetech STS/SCS class).
