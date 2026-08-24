# DeltaSeek

Code and data for **"DeltaSeek: Toward Active Perception in Evolving
Construction Environments."**

Construction environments change continuously, which makes perception hard to
treat as a static mapping problem. This repository accompanies an extended
abstract asking a question that underlies planning where to look: **how does
sensing embodiment constrain the observations available to a mobile robot?**

A Clearpath Husky A300 with a UR5e is evaluated in an IFC-derived simulated
room under two RGB-D configurations — one camera fixed to the chassis, an
identical one at the wrist — against eight controlled scene changes. Candidate
observations are scored by geometric visibility; effort is drivable distance on
the platform's own Nav2 costmap.

## Results

Eight changes in Hall B, two in each observability class, evaluated over **240
admissible chassis poses** and **1200 wrist poses**.

| Class | Chassis | Wrist | Both |
| --- | --- | --- | --- |
| Trivial | 2/2 | 2/2 | 2/2 |
| Height | **0/2** | 2/2 | 2/2 |
| Incidence | 1/2 | 2/2 | 2/2 |
| Occluded | 2/2 | 2/2 | 2/2 |
| **All** | **5/8** | 8/8 | 8/8 |

The headline is not 5/8. A single planned run conflates what a sensor *can*
see with what one route happened to reach, so the two are separated by
evaluating every admissible pose exhaustively:

- **Two changes are observable from no chassis pose at all**, at zero visible
  fraction. That is a capability limit set by embodiment.
- **One further change** the chassis-only run missed is observable from 14 of
  the 240 poses; the planner simply did not reach one inside the budget. That
  is routing, not evidence for the manipulator.

Where both configurations succeed, the difference is distance rather than
detections: median drivable distance to first observation is **6.0 m** for the
wrist against **14.2 m** for the chassis.

Only the *height* class produces a capability gap, and the geometry says why.
The highest world point a camera can place inside its frustum is
`h + r(cos p · tan(v/2) − sin p)`, with no yaw term — yaw rotates about the
vertical axis and cannot change the vertical extent. At the 5 m range limit
that is **2.24 m** for the chassis camera and **4.63 m** for the wrist, against
a ceiling at 2.44 m, leaving a wrist-only band **0.20 m** tall.

Two effects follow, and an active-perception system must treat them
differently. **Sensing capability** is binary and set by embodiment: no
planning recovers a change outside every achievable frustum. **Acquisition
cost** is continuous and set by the environment, and is where viewpoint
planning has leverage.

### Reproducing them

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
cd ros2_ws/src/deltaseek_gazebo

ros2 run deltaseek_gazebo sensor_ablation \
  --manifest config/benchmarks/hall_b_ablation_nominal.yaml \
  --scenario config/benchmarks/hall_b_ablation_deviations.yaml \
  --room 2.9 -6.2 9.9 6.3 --start 6.4 4.5 --output-dir ../../../results
```

Outputs land in [`results/`](results): `sensor_ablation.md` is the table above,
`sensor_ablation.csv` the per-change detail, and `sensor_ablation.json` the
machine-readable form including per-pose observation counts.

Two supporting measurements are also committed.
[`results/pose_construction.md`](results/pose_construction.md) documents how the
candidate poses are built, derives the frustum ceiling in closed form, checks it
against a numeric sweep to 1 mm, and reports a sampling-sensitivity sweep: the
height pair stays at exactly zero visible fraction through 1840 poses at 22.5°
yaw spacing, so the capability result is not a discretisation artefact.
[`results/sensing_envelope.json`](results/sensing_envelope.json) carries the
reaches.

### Scope

The result is one room, one ceiling height and eight changes; the 0.20 m band
is a property of that geometry. Detection is geometric — no detector, no
renderer, no noise — so the numbers describe viewpoint quality rather than
perception robustness. Elements are boxes, which discards the 48 opening
entities in the source IFC, so the room is sealed and the robot is spawned
inside it.

## Repository layout

| path | contents |
| --- | --- |
| `ros2_ws/` | the ROS 2 workspace: pipeline, planner, evaluation, 75 tests |
| `clearpath/` | `robot.yaml` and the generated robot description |
| `results/` | the ablation outputs and the pose-construction audit |
| `generated/` | worlds and manifests built from the IFC model |
| `scripts/` | installer and the Word renderer |
| `publication/` | manuscript, templates and rendered outputs (untracked) |

`publication/` holds the manuscript itself and is not tracked. The generators
that produce its figures and both document formats are, so a clone rebuilds
them from source:

```bash
python3 ros2_ws/src/deltaseek_gazebo/scripts/make_abstract_figures.py
python3 ros2_ws/src/deltaseek_gazebo/scripts/compose_platform_figure.py
python3 scripts/make_ieee_docx.py --paper=letter
```

## Build

ROS 2 Jazzy and `ros-jazzy-clearpath-simulator` are installed system-wide. Do
not put ROS in Conda: the Ubuntu packages are built against the system Python.

Conda must not be active when building or launching. Anaconda places its own
`python3` and its own `libstdc++` ahead of the system ones, and ROS's compiled
extensions then fail to load:

```text
import rclpy -> ImportError: libstdc++.so.6: version `GLIBCXX_3.4.30' not found
```

Keep base deactivated so that an ordinary shell is a working ROS shell, and
activate `deltaseek-ifc` only for the IFC conversion step:

```bash
conda config --set auto_activate_base false
```

Avoid paths containing spaces. `launch.substitutions.Command` splits its input
with `shlex.split`, so a space in the workspace path breaks the xacro call in
`simulation.launch.py`.

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

The launch files resolve their own location back to this checkout to find
[`clearpath/robot.yaml`](clearpath/robot.yaml), so `setup_path` only needs to
be passed when pointing at a different configuration, such as an installed
robot's `/etc/clearpath`. `DELTASEEK_SETUP_PATH` overrides it for a shell.

## Run the simulation

`clearpath_sim.launch.py` starts the official Clearpath construction world and
generates the robot description and controllers. The Gazebo GUI is on by
default:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
source install/setup.bash
ros2 launch deltaseek_gazebo clearpath_sim.launch.py
```

Add `rviz:=true` for the Clearpath RViz configuration. On a machine reached
over SSH, add `headless:=true` to run the Gazebo server with no display.

Leave that terminal running. In a second terminal, source the same two setup
files before using ROS commands.

Drive slowly for two seconds, then send a stop:

```bash
timeout 2s ros2 topic pub -r 10 /a300_00000/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.15}}}"
ros2 topic pub --once /a300_00000/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
```

Inspect the principal interfaces:

```bash
ros2 topic echo /a300_00000/platform/odom
ros2 topic echo /a300_00000/platform/joint_states
ros2 topic echo /a300_00000/sensors/imu_0/data
ros2 action list -t
ros2 service call /a300_00000/controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers '{}'
```

The verified UR5e action is
`/a300_00000/arm_0_joint_trajectory_controller/follow_joint_trajectory`.
Use MoveIt or a collision-checked trajectory before commanding broad arm
motions; a small controller smoke test is shown in
[`clearpath/README.md`](clearpath/README.md).

## Upstream launch and remote displays

The unmodified Clearpath entry point is also available, and is the reference
when checking whether a problem comes from this project's wrapper:

```bash
ros2 launch clearpath_gz simulation.launch.py \
  setup_path:=$PWD/clearpath \
  world:=construction
```

Gazebo's GUI must render on a machine with a display. Over SSH, either use
trusted X11 forwarding (`ssh -Y`, with `QT_X11_NO_MITSHM=1`), which is slow, or
run `headless:=true` on the remote machine and visualize ROS topics locally
with RViz/Foxglove over a suitable ROS 2 network or bridge.

## IFC-derived deviation benchmark

The CPU-only IFC/deviation benchmark pipeline converts IFC products into a
nominal element manifest keyed by IFC GlobalId, then generates a nominal SDF,
deviated SDF, and exact ground-truth YAML from an explicit scenario. Elements
are boxed in their own placement frame, because real exports sit on a rotated
site grid and a world-aligned box would inflate every long wall:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
source install/setup.bash
ros2 launch deltaseek_gazebo benchmark.launch.py
```

See
[`config/benchmarks/README.md`](ros2_ws/src/deltaseek_gazebo/config/benchmarks/README.md)
for IFC conversion, deviation generation, and schema details.

## Sensors

`robot.yaml` describes the three sensors the physical robot will carry. All
mount transforms are placeholders until the as-built robot is measured.

| sensor | topic prefix | mount | position (base_link) |
| --- | --- | --- | --- |
| eye-in-hand D435 | `sensors/camera_0/` | `arm_0_tool0` | moves with the arm |
| chassis D435 | `sensors/camera_1/` | enclosure deck | (0.460, 0.000, 0.418) |
| Ouster OS1 | `sensors/lidar3d_0/` | enclosure deck | (0.380, 0.000, 0.429) |

All under `/a300_00000/`. The chassis camera is deliberately the same model as
the eye-in-hand one, so the ablation isolates where a camera is mounted rather
than how good it is.

**Camera order in `robot.yaml` is load-bearing.** Clearpath indexes cameras
positionally, so the eye-in-hand sensor must stay first to remain `camera_0`.
Reordering that list silently repoints every `camera_0` reference in the launch
files, the RViz config and the forward-kinematics chain.

Both chassis sensors stand on the enclosure deck, its top face at z = 0.407
spanning x = ±0.486 and y = ±0.226. Placement is constrained by the arm, which
is mounted forward of centre at (0.243, 0.000): worst clearance across the
planner's postures is 0.078 m to the lidar and 0.168 m to the camera, after
subtracting link radii. Two consequences worth knowing:

- The camera stands 0.093 m directly in front of the lidar on the centreline
  and blocks roughly 30 deg of its azimuth dead ahead, along the direction of
  travel. Raising the lidar clears it.
- An earlier revision bracketed both sensors at |y| = 0.30. That is inside the
  0.349 m navigation footprint and cleared the arm comfortably, so every check
  passed -- but the enclosure side face is at y = 0.226, so they hung in the air
  beside the robot. `test_sensors.py` now asserts against the enclosure's
  measured mesh extents rather than the navigation footprint.

## Evaluating a run

Scenarios can be sampled at a controlled deviation density, and a trajectory
can be scored against exact ground truth:

```bash
cd ros2_ws/src/deltaseek_gazebo
ros2 run deltaseek_gazebo sample_deviations \
  --manifest config/benchmarks/demo_nominal.yaml \
  --output /tmp/sampled.yaml --density 0.5 --seed 7
ros2 run deltaseek_gazebo evaluate_run \
  --manifest config/benchmarks/demo_nominal.yaml \
  --scenario /tmp/sampled.yaml \
  --trajectory config/trajectories/demo_sweep.yaml
```

The report gives detection rate against traversal-distance budget, which is
the quantity the planner is meant to improve. Detection uses a geometric
visibility model rather than rendering, so it is fast enough to run inside a
planner loop; the trade-offs are documented in the benchmarks README.

## Viewpoint planning

`sensor_ablation` fixes the planner and varies the sensing configuration. The
selection code it uses is shared with `compare_planners`, which runs the
deviation-seeking planner against coverage, frontier and goal-directed
baselines on a shared candidate set and budget:

```bash
cd ros2_ws/src/deltaseek_gazebo
ros2 run deltaseek_gazebo compare_planners \
  --manifest config/benchmarks/hall_b_ablation_nominal.yaml \
  --scenario config/benchmarks/hall_b_ablation_deviations.yaml \
  --room 2.9 -6.2 9.9 6.3 --start 6.4 4.5 --budget 25 --max-range 5
```

That comparison is not part of the paper, which holds the planner fixed. It is
kept because the ablation depends on the same code, and because `--sensors
{chassis,wrist,both}` applies to both entry points: the sensor set drives
planning and scoring together, so a configuration is never scored on
information it did not have.

Camera poses come from forward kinematics over the generated URDF, so the arm
mount and camera transform stay driven by `robot.yaml`. That chain is verified
against the running simulation's TF tree to machine precision.

Travel cost is drivable distance by default: Dijkstra over an occupancy grid
inflated by the platform's own Nav2 footprint. `--path-cost euclidean` restores
straight lines.

## Working on a single room

A whole storey is the wrong unit for studying how a robot decides what it is
looking at, and it is also unusable directly: the elements are building-scale,
so one wall entity runs the full 39 m of a facade and one slab is the entire
floor. `extract_room` clips a manifest to a rectangle, cutting in each
element's own frame so the site rotation survives:

```bash
ros2 run deltaseek_gazebo extract_room \
  --manifest generated/ers/ers_nominal.yaml \
  --output generated/room/room_nominal.yaml \
  --room-id hall_b --bounds 2.70 -6.55 10.10 6.50 --z-range -0.2 2.55
```

Hall B falls out of `ERS_B_STRUCT.ifc` as 12 elements over 7.0 x 12.5 m: four
walls, two windows, a door, floor and ceiling.

Because boxed walls have no openings, a clipped room is a sealed box and its
door is impassable. The robot therefore has to be spawned inside it, which is
what the `x`, `y` and `yaw` arguments on the launch files are for:

```bash
ros2 launch deltaseek_gazebo benchmark.launch.py \
  world_file:=$PWD/generated/ablation/hall_b_ablation_deviated.sdf \
  world_name:=deltaseek_hall_b_ablation_hall_b_ablation \
  ground_truth_file:=$PWD/generated/ablation/hall_b_ablation_ground_truth.yaml \
  x:=6.4 y:=4.5 yaw:=-1.5708 rviz:=true
```

The platform's odometry reads zero wherever the robot spawns, so the odom frame
is anchored at the spawn pose and the launch publishes `world -> odom` to
match. Anything driving to planner output, which is in world coordinates, must
convert; `scripts/demo_execute_plan.py` takes `--spawn x y yaw` and logs both
frames so they can be compared.

## Which deviations need the arm?

The ablation and its result lead this README; see [Results](#results) for the
table and the reproduction command, and
[`results/pose_construction.md`](results/pose_construction.md) for how the
candidate poses are constructed.

Deviations carry a `class` recording *why* they are hard to observe, which is
independent of what kind of deviation they are:

| class | meaning |
| --- | --- |
| `trivial` | large unmodelled object in open floor space |
| `height` | above the chassis camera's sightline |
| `incidence` | resolvable only from an oblique angle |
| `occluded` | in another object's shadow from most base poses |

The eight changes are hand-placed in a committed scenario rather than sampled,
so each is explainable from geometry and the study is exactly reproducible. The
`trivial` pair is a control: a configuration that misses it is visibly broken.

## Legacy experimental overlay

`simulation.launch.py`, the custom world, inspection-camera model, and
discrepancy markers are an earlier project prototype. They remain useful for
algorithm experiments, but they are not the hardware-transfer baseline and do
not claim to reproduce the exact physical sensor/mount configuration. New
project sensors should be added only after their models and transforms are
confirmed from the as-built robot configuration.
