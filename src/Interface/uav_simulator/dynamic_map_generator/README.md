# map_generator

`map_generator` provides the point-cloud map used by the simulator. In addition
to loading existing PCD files, it includes an RViz-based editor for building a
new scene from walls and square pillars.

## Build

Build the workspace and source it before launching the editor:

```bash
cd /path/to/GVF-Nav
catkin_make
source devel/setup.bash
```

## Start the scene editor

```bash
roslaunch map_generator draw_map.launch
```

The launch file starts only `scene_editor` and its dedicated RViz window. The
RViz fixed frame is `world`; the live scene is published on `/mock_map` and the
editing hints are published on `/scene_editor/markers`.

Run the editor separately from the simulator. The simulator's `map_pub` also
publishes `/mock_map`, so leaving both running would interleave the old map and
the editing preview.

The defaults can be overridden on the launch command line. All dimensions are
in metres:

```bash
roslaunch map_generator draw_map.launch \
  resolution:=0.1 \
  wall_thickness:=0.3 \
  pillar_width:=0.6 \
  base_z:=-1.0 \
  height:=4.0
```

`resolution` is fixed for the lifetime of the editor. The other dimensions can
also be changed while the editor is running.

## Draw a scene in RViz

### Wall

1. Select **2D Nav Goal** in the RViz toolbar.
2. Click and drag once to choose the first endpoint. The arrow orientation is
   ignored.
3. Click and drag a second time to choose the other endpoint. The editor then
   creates a wall between the two points.

Every pair of goals creates one wall. A marker identifies an unmatched first
endpoint while the editor is waiting for the second one.

### Pillar

1. Select **Publish Point** in the RViz toolbar.
2. Click once on the XY grid to place a square pillar.

Input endpoints and pillar centres are snapped to the configured grid. The
solid wall and pillar volumes are then sampled at that resolution, and
numerically coincident points are stored only once.

## Change dimensions while editing

The following private parameters apply to objects created after the change;
objects already in the scene keep their original geometry:

```bash
rosparam set /scene_editor/wall_thickness 0.4
rosparam set /scene_editor/pillar_width 0.8
rosparam set /scene_editor/base_z -0.5
rosparam set /scene_editor/height 3.5
```

## Undo, clear, and cancel

```bash
# Remove the most recently created wall or pillar.
rosservice call /scene_editor/undo

# Remove every object. One undo restores the cleared scene.
rosservice call /scene_editor/clear

# Discard an unmatched first wall endpoint without changing the scene.
rosservice call /scene_editor/cancel_wall
```

Undo on an empty scene and cancelling when no wall is pending are safe no-op
operations; the service response reports what happened.

## Save as PCD

Save the current scene with the `SaveScene` service:

```bash
rosservice call /scene_editor/save \
"name: 'warehouse_v1'
overwrite: false"
```

The `.pcd` suffix is optional. By default this creates:

```text
$(rospack find map_generator)/resource/warehouse_v1.pcd
```

Scene names may contain only letters, numbers, `_`, and `-`. An empty scene is
not saved. If the target already exists, repeat the call with `overwrite: true`
only when replacing it is intentional. The response contains the final path or
an error message.

An alternate output directory can be selected at launch time:

```bash
roslaunch map_generator draw_map.launch output_dir:=/absolute/path/to/maps
```

## Use the new scene in simulation

Saving does not switch the running simulator automatically. Open
`so3_quadrotor_simulator/launch/simulator.launch` and replace the PCD passed to
`map_pub`, for example:

```xml
<node pkg="map_generator" name="map_pub" type="map_pub" output="screen"
      args="$(find map_generator)/resource/warehouse_v1.pcd"/>
```

Restart the simulator after changing the file. `map_pub` loads the saved PCD
and publishes it on `/mock_map`.

## Map editor interfaces

| Interface | Type / purpose |
| --- | --- |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped`; two messages define a wall |
| `/clicked_point` | `geometry_msgs/PointStamped`; one message places a pillar |
| `/mock_map` | live, latched `sensor_msgs/PointCloud2` preview |
| `/scene_editor/markers` | `visualization_msgs/MarkerArray` editing hints |
| `/scene_editor/undo` | undo the last edit |
| `/scene_editor/clear` | clear all objects as one undoable edit |
| `/scene_editor/cancel_wall` | cancel the pending wall endpoint |
| `/scene_editor/save` | `map_generator/SaveScene`; save an ASCII PCD |

This package is based on
[yuwei-wu/map_generator](https://github.com/yuwei-wu/map_generator) and remains
compatible with point-cloud consumers used by Fast-Planner and EGO-Planner.
