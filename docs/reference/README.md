# Reference artefacts

`partial-export-from-third-party-exporter.xacro` is the truncated output of the
failed `Fusion_URDF_Exporter_ROS2` run that started this project: 69 lines, three
links, no closing `</robot>`.

It stops after `base_link`, `p02_new_base_neck_1` and `p10_new_elbow_v1_1`
because `write_link_urdf` walks the joints dictionary looking each child up in
`inertial_dict`, and the third joint's child was an occurrence nested inside
another component — so it was never registered as a link. The resulting bare
`KeyError` surfaces as a dialog reading only `Failed: <name>`.

Kept as evidence for the failure analysis in `../exporter_internals.md`. It is
not a working description and is not built by anything.
