# IFC benchmark pipeline

This directory defines the CPU-only portion of the DeltaSeek benchmark:

```text
IFC file -> nominal element manifest -> deviation scenario
                                      -> nominal SDF
                                      -> deviated SDF
                                      -> exact ground-truth YAML
```

The manifest is the stable boundary between IFC tooling and ROS/Gazebo. Every
element retains its IFC `GlobalId`, IFC class, name, pose, and geometry. The
same generator creates both the simulated scene and its labels, preventing
the world and ground truth from drifting apart.

## Convert an IFC model

IFC conversion is intentionally separate from the ROS runtime. IfcOpenShell
is installed without administrator access in the `deltaseek-ifc` Conda
environment:

The explicit `PYTHONPATH` matters: a ROS shell exports its own, and leaving it
in place would shadow the Conda environment's packages.

```bash
cd ~/deltaseek
PYTHONPATH=ros2_ws/src/deltaseek_gazebo \
conda run -n deltaseek-ifc python -m deltaseek_gazebo.ifc_to_manifest \
  --ifc /absolute/path/model.ifc \
  --output /absolute/path/nominal.yaml \
  --origin 0 0 0 \
  --threads 4
```

Use `--origin` to subtract a project/site coordinate offset when an IFC model
is georeferenced far from the desired Gazebo origin.

Each supported IFC product is represented by a box oriented in its own
placement frame. Orientation is not a refinement: building models are routinely
exported on a site grid rotated a fraction of a degree off the world axes, and
a world-aligned box inflates anything long and thin sitting on such a grid. On
`ERS_B_STRUCT.ifc` (Revit 2025, IFC2X3, a 44 x 20 m storey rotated 0.90 deg)
the world-aligned box turns 8-inch walls into 0.82 m ones -- a 4x error on the
dimension that decides whether the robot fits -- and blocks 11% of the
candidate base stations. `--geometry aabb` restores the old behaviour for
comparison.

Angles are written as roll-pitch-yaw in the `Rz @ Ry @ Rx` convention that
`visibility.rotation_matrix` and SDF both use, so one orientation is shared by
the planner, the evaluator, and Gazebo.

The approximation is still a box: it does not preserve openings or detailed
surface shape, so a wall exported as one 39 m entity with eleven doors in it
becomes a solid 39 m barrier. A tessellating export, or splitting voided walls
into jamb and lintel segments the way `synthetic_storey` does, is required
before image-based deviation detection on realistic geometry means anything.

## Define deviations

Each deviation has a stable ID and one operation:

- `displaced`: add a world-frame XYZ translation;
- `rotated`: add roll, pitch, and yaw angles in radians;
- `missing`: remove a nominal IFC element;
- `resized`: multiply geometry dimensions by XYZ scale factors;
- `unexpected`: add a complete element that is absent from the nominal model.

Only one deviation may target a given nominal element in a scenario. This
keeps labels unambiguous. More complex compound deviations should be emitted
as a precomposed operation in a future schema version.

## Generate a benchmark

After building and sourcing the ROS workspace:

```bash
ros2 run deltaseek_gazebo generate_benchmark \
  --manifest /absolute/path/nominal.yaml \
  --scenario /absolute/path/deviations.yaml \
  --output-dir /absolute/path/generated
```

The checked-in smoke-test data can be regenerated from the source tree with:

```bash
cd ~/deltaseek/ros2_ws/src/deltaseek_gazebo
PYTHONPATH=. python3 -m deltaseek_gazebo.generate_benchmark \
  --manifest config/benchmarks/demo_nominal.yaml \
  --scenario config/benchmarks/demo_deviations.yaml \
  --output-dir worlds/generated
```

Do not edit generated SDF or ground-truth files by hand. Modify the manifest
or scenario and regenerate them.

## Run with the official robot

```bash
source /opt/ros/jazzy/setup.bash
source ~/deltaseek/ros2_ws/install/setup.bash
ros2 launch deltaseek_gazebo benchmark.launch.py
```

The demo launch starts the official Clearpath-generated A300/UR5e in the
deviated world and publishes durable reference/actual markers on
`/deltaseek/ground_truth/discrepancies`. It shows the Gazebo GUI by default;
pass `headless:=true` on a machine without a display, and `rviz:=true` to see
the ground-truth markers.

## Sample scenarios at a controlled density

Hand-written scenarios pin one difficulty. A density sweep is what shows
whether a deviation-seeking planner keeps its advantage as deviations become
rare, so scenarios can also be sampled:

```bash
ros2 run deltaseek_gazebo sample_deviations \
  --manifest config/benchmarks/demo_nominal.yaml \
  --output /absolute/path/sampled.yaml \
  --scenario-id sweep_d50_s7 \
  --density 0.5 \
  --unexpected-rate 0.15 \
  --seed 7
```

`--density` is the fraction of nominal elements carrying a deviation.
Unexpected elements are counted separately by `--unexpected-rate`, because they
add to the scene rather than modifying an element, and so cannot consume an
element's single-deviation slot. A scenario id plus a seed reproduces a
benchmark exactly; the output is an ordinary scenario file that
`generate_benchmark` consumes unchanged.

## Score a run

`evaluate_run` replays a trajectory against exact ground truth and reports
detections against a *traversal distance budget*, which is the comparison the
project's claim rests on:

```bash
ros2 run deltaseek_gazebo evaluate_run \
  --manifest config/benchmarks/demo_nominal.yaml \
  --scenario config/benchmarks/demo_deviations.yaml \
  --trajectory config/trajectories/demo_sweep.yaml \
  --output /absolute/path/report.yaml
```

A trajectory is a list of viewpoints, each with a world-frame `base` position
and `camera` pose:

```yaml
schema_version: 1
trajectory: {id: demo_sweep, frame_id: world}
viewpoints:
  - base: {xyz: [-6.0, -3.0, 0.0], yaw: 0.0}
    camera: {xyz: [-6.0, -3.0, 1.45], rpy: [0.0, -0.35, 0.0]}
```

Distance accumulates over base positions only. Arm motion is excluded on
purpose: the claim under test is that including the manipulator in viewpoint
selection buys detections *without* extra driving, so charging the arm for
distance would obscure the result.

### How an element counts as observed

Detection is geometric rather than image-based. An element's surface is
sampled, and a sample counts when it is inside the view frustum, faces the
camera within an incidence limit, and is not occluded by another element.
`--min-visible-fraction` is the share of surface required, and stands in for a
detector's sensitivity.

Each deviation type has its own evidence:

| Type | Revealed by |
| --- | --- |
| `missing` | seeing into the nominal volume and finding it empty |
| `displaced` | seeing the element misplaced, or its modelled place empty |
| `rotated`, `resized`, `unexpected` | seeing the as-built element |

This model is fast enough to sit inside a planner loop, which rendering is
not. It assumes perfect recognition and perfect localization, so it measures
*viewpoint quality*, not perception robustness. Replacing it with a
depth-image detector behind the same interface is what the physical-robot
evaluation will need.

## Synthetic evaluation storey

The demo fixture is seven elements in one open room; every planner sees
everything and saturates, so it cannot separate them. `generate_storey`
produces a scene that can:

```bash
ros2 run deltaseek_gazebo generate_storey \
  --output /absolute/path/storey.yaml \
  --rooms-per-side 4 --seed 1
```

The layout is a central corridor with rooms down both sides, which is the
cheapest arrangement that forces the robot to enter a space to inspect it.
Defaults give 110 elements: walls, columns, corridor services, and per-room
equipment, branch ducts, wall panels and low fittings.

Contents sit at heights chosen to reward different arm postures. Services run
near the ceiling, wall panels at chest height, and fittings are tucked against
the far side of equipment where only a viewpoint that reaches past the cabinet
resolves them.

### Why the geometry is built rather than imported

Walls are emitted as segments around their openings — two jambs and a lintel —
so a doorway is a real gap the sensor can see through. `ifc_to_manifest`
cannot yet produce that: it approximates each IFC product by its world-aligned
bounding box, which turns a wall with a door into a solid barrier and inflates
occlusion exactly where the planner's reasoning depends on it. Building the
scene from primitives sidesteps that until a tessellating IFC importer exists.
`test_doorways_are_real_openings` guards the property.

### Traversal cost

Distance is the denominator of the project's claim, so it has to be a distance
the robot could drive. `navigation.py` computes it on an inflated occupancy
grid using the platform's own Nav2 settings: NavfnPlanner is Dijkstra over a
costmap, so the same quantity is computed directly rather than through an
action server, which would cost a round trip for each of the tens of thousands
of pairwise queries a planner makes.

    costmap resolution   0.06 m
    footprint            0.495 x 0.349 m half-extents
    circumscribed radius 0.606 m, used as the inflation

Obstacles inflated by the circumscribed radius are treated as lethal, which is
the conservative reading of an inflation layer: Nav2 would let a path graze an
inflated cell at higher cost, so these lengths bound what Nav2 returns from
above. On the synthetic storey, drivable distance averages about 1.8x the
straight line and reaches 11x, so the choice is not a refinement.

`--path-cost euclidean` restores straight lines for comparison.

### Planner baselines

`compare_planners` runs the deviation-seeking planner against coverage,
frontier and goal-directed baselines on a shared candidate set and a shared
budget. It is not part of the paper's experiment, which fixes the planner and
varies the sensing configuration; it is retained because `sensor_ablation`
uses the same selection code.
