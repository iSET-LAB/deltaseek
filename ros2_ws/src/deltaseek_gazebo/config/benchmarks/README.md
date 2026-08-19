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

```bash
cd /ISET/sxa4756/deltaseek
PYTHONPATH=ros2_ws/src/deltaseek_gazebo \
conda run -n deltaseek-ifc python -m deltaseek_gazebo.ifc_to_manifest \
  --ifc /absolute/path/model.ifc \
  --output /absolute/path/nominal.yaml \
  --origin 0 0 0 \
  --threads 4
```

Use `--origin` to subtract a project/site coordinate offset when an IFC model
is georeferenced far from the desired Gazebo origin.

The first adapter represents each supported IFC product with a world-aligned
bounding box. This is deterministic, conservative, and inexpensive for
physics/planning development, but it does not preserve openings or detailed
surface shape. A mesh/tessellation export will be required before evaluating
image-based deviation detection on realistic geometry.

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
cd /ISET/sxa4756/deltaseek/ros2_ws/src/deltaseek_gazebo
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
source /ISET/sxa4756/deltaseek/ros2_ws/install/setup.bash
ros2 launch deltaseek_gazebo benchmark_headless.launch.py
```

The demo launch starts the official Clearpath-generated A300/UR5e in the
deviated world and publishes durable reference/actual markers on
`/deltaseek/ground_truth/discrepancies`.
