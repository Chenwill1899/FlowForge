# Flow-GVF validation scenes

These deterministic ASCII PCD scenes were generated through the
`map_generator/scene_editor` ROS node with:

- resolution: `0.1 m`
- wall thickness: `0.3 m`
- vertical extent: `z = -0.5 ... 2.5 m`
- nominal start: `(0, 0, 1)`
- nominal intent: world-frame `+x`

## Scenarios

| File | Purpose |
| --- | --- |
| `far_offset_door.pcd` | A wall at `x=8 m` with a `2.5 m` door centered at `y=5 m`; primary coarse-prior test. |
| `asymmetric_two_doors.pcd` | Narrow lower door and wide upper door; tests geometry-driven channel choice. |
| `asymmetric_two_doors_mirrored.pcd` | Mirrored asymmetric doors; detects a fixed left/right bias. |
| `symmetric_two_doors.pcd` | Symmetric equal-width doors; exposes separatrix/stagnation behavior. |
| `side_exit_room.pcd` | Closed room with one side exit outside the initial fine-sensing range. |
| `two_gate_s_corridor.pcd` | Two offset gates inside a bounded corridor; tests propagation of coarse guidance. |
| `door_width_0p8.pcd` | Centered `0.8 m` door for resolution/topology testing. |
| `door_width_1p2.pcd` | Centered `1.2 m` door for resolution/topology testing. |
| `door_width_2p0.pcd` | Centered `2.0 m` door for resolution/topology testing. |
| `door_width_3p0.pcd` | Centered `3.0 m` door for resolution/topology testing. |

Load one in the baseline benchmark with:

```bash
BENCHMARK_MAP="$PWD/src/Interface/uav_simulator/dynamic_map_generator/resource/flow-gvf/far_offset_door.pcd" \
  scripts/run_baseline_benchmark.sh
```

The PCD coordinates are published unchanged in the `world` frame by
`map_generator/map_pub`.
