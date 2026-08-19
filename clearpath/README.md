# Clearpath configuration

`robot.yaml` is the single source of truth consumed by Clearpath's official
real-robot and Gazebo generators.

The checked-in file is a public-spec baseline assembled from Clearpath's
released A300 AMP and Universal Robots examples.  The following values are
placeholders until the physical robot's as-built configuration is available:

- serial number, hostname, and ROS namespace;
- host and UR controller IP addresses;
- the UR5e mounting transform;
- installed sensor and end-effector inventory.

Do not tune those values from photographs.  Before hardware deployment, copy
the robot's authoritative `/etc/clearpath/robot.yaml` here and review the diff.
Project-only simulated sensors and discrepancy annotations belong in the
DeltaSeek overlay, not in the hardware configuration.

Generate and launch the official Gazebo configuration with:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch clearpath_gz simulation.launch.py \
  setup_path:=/ISET/sxa4756/deltaseek/clearpath \
  world:=construction
```

That upstream command starts the Gazebo GUI and therefore needs a display. For
an SSH-safe server-only run, build the workspace and use the local wrapper:

```bash
source /opt/ros/jazzy/setup.bash
source /ISET/sxa4756/deltaseek/ros2_ws/install/setup.bash
ros2 launch deltaseek_gazebo clearpath_headless.launch.py \
  setup_path:=/ISET/sxa4756/deltaseek/clearpath
```

The installed `clearpath_generator_gz` 2.9.2 generates an empty bridge file for
the Phidgets Spatial IMU. The headless wrapper adds only that missing
simulation bridge; it does not modify the generated description or controller
configuration.

A conservative UR5e controller smoke test is:

```bash
ros2 action send_goal \
  /a300_00000/arm_0_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [arm_0_shoulder_pan_joint, arm_0_shoulder_lift_joint, arm_0_elbow_joint, arm_0_wrist_1_joint, arm_0_wrist_2_joint, arm_0_wrist_3_joint], points: [{positions: [0.0, 0.158, 0.0, 0.0, 0.0, 0.20], time_from_start: {sec: 3}}]}}"
```

This command is for simulation validation only. Re-derive safe poses and
collision constraints from the final as-built mounting transform before using
the physical arm.
