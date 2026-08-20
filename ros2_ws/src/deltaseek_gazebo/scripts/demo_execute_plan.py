"""Execute a planned inspection in simulation: drive, pose the arm, observe.

This runs the deviation-seeking planner on a scenario and then carries the
plan out on the robot, so the base and the arm move for the same reasons the
evaluation assumes. It is a demonstration rather than the evaluation path;
scoring is geometric and needs no simulator at all.

Base motion is a proportional controller on odometry, not Nav2. That is honest
for short hops between viewpoints in open space and nothing more: a plan that
has to cross a doorway needs the real navigation stack.
"""

import argparse
import math
from pathlib import Path

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml

from deltaseek_gazebo.benchmark import validate_manifest
from deltaseek_gazebo.detection import ObservationParams
from deltaseek_gazebo.kinematics import Chain
from deltaseek_gazebo.navigation import path_cost_for
from deltaseek_gazebo.planner import plan_deviation_seeking, visibility_matrix
from deltaseek_gazebo.viewpoints import (
    build_viewpoints,
    free_base_poses,
    scene_bounds,
)


JOINTS = [
    'arm_0_shoulder_pan_joint',
    'arm_0_shoulder_lift_joint',
    'arm_0_elbow_joint',
    'arm_0_wrist_1_joint',
    'arm_0_wrist_2_joint',
    'arm_0_wrist_3_joint',
]


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PlanRunner(Node):
    """Drives the base to each viewpoint and holds the planned arm posture."""

    def __init__(self, namespace, linear_speed, angular_speed, tolerance):
        super().__init__('demo_execute_plan')
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.tolerance = tolerance
        self.pose = None

        self.cmd = self.create_publisher(
            TwistStamped, f'/{namespace}/cmd_vel', 10)
        self.create_subscription(
            Odometry, f'/{namespace}/platform/odom', self._on_odom, 10)
        self.arm = ActionClient(
            self, FollowJointTrajectory,
            f'/{namespace}/arm_0_joint_trajectory_controller'
            '/follow_joint_trajectory')
        self.get_logger().info('waiting for arm controller')
        self.arm.wait_for_server()

    def _on_odom(self, message):
        position = message.pose.pose.position
        q = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (position.x, position.y, yaw)

    def _publish(self, linear, angular):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.twist.linear.x = float(linear)
        message.twist.angular.z = float(angular)
        self.cmd.publish(message)

    def stop(self):
        for _ in range(5):
            self._publish(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for_odom(self, timeout=25.0):
        deadline = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while self.pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds > deadline:
                raise RuntimeError('no odometry received')

    def drive_to(self, target, budget_seconds=45.0):
        """Turn toward the target, drive to it, and stop."""
        deadline = self.get_clock().now().nanoseconds + int(budget_seconds * 1e9)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is None:
                continue
            x, y, yaw = self.pose
            dx, dy = target[0] - x, target[1] - y
            distance = math.hypot(dx, dy)
            if distance <= self.tolerance:
                break
            if self.get_clock().now().nanoseconds > deadline:
                self.get_logger().warn(
                    f'gave up {distance:.2f} m short of the waypoint')
                break
            heading_error = _wrap(math.atan2(dy, dx) - yaw)
            if abs(heading_error) > 0.35:
                # Turn in place first: a differential base cannot strafe.
                self._publish(0.0, max(-self.angular_speed, min(
                    self.angular_speed, 1.5 * heading_error)))
            else:
                self._publish(
                    min(self.linear_speed, 0.8 * distance),
                    max(-1.0, min(1.0, 1.2 * heading_error)))
        self.stop()

    def strike(self, positions, seconds):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(
            joint_names=JOINTS,
            points=[JointTrajectoryPoint(
                positions=[float(value) for value in positions],
                time_from_start=Duration(sec=int(seconds)))],
        )
        handle = self.arm.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, handle)
        result = handle.result().get_result_async()
        rclpy.spin_until_future_complete(self, result)


def _parser():
    parser = argparse.ArgumentParser(
        description='Plan an inspection and execute it on the simulated robot.')
    parser.add_argument('--manifest', required=True, type=Path)
    parser.add_argument('--scenario', required=True, type=Path)
    parser.add_argument('--urdf', type=Path,
                        default=Path('/tmp/deltaseek_robot.urdf'))
    parser.add_argument('--namespace', default='a300_00000')
    parser.add_argument('--budget', type=float, default=25.0)
    parser.add_argument('--spacing', type=float, default=2.0)
    parser.add_argument('--max-viewpoints', type=int, default=8)
    parser.add_argument('--linear-speed', type=float, default=0.45)
    parser.add_argument('--angular-speed', type=float, default=0.7)
    parser.add_argument('--tolerance', type=float, default=0.25)
    parser.add_argument('--posture-seconds', type=float, default=3.0)
    parser.add_argument('--loop', action='store_true',
                        help='Repeat the plan until interrupted.')
    return parser


def plan(args):
    """Return the ordered viewpoints the planner selects for this scenario."""
    manifest = yaml.safe_load(args.manifest.read_text(encoding='utf-8'))
    elements = validate_manifest(manifest)['elements']
    chain = Chain.from_urdf(args.urdf, 'base_link', 'camera_0_link')
    bases = free_base_poses(
        elements, scene_bounds(elements), spacing=args.spacing,
        yaws=(0.0, 1.5707963, 3.1415927, -1.5707963))
    viewpoints = build_viewpoints(chain, bases)
    _, matrix = visibility_matrix(viewpoints, elements, ObservationParams())
    order, info = plan_deviation_seeking(
        viewpoints, matrix, args.budget, start=(0.0, 0.0),
        cost_fn=path_cost_for(elements))
    return [viewpoints[index] for index in order[:args.max_viewpoints]], info


def main():
    args = _parser().parse_args()
    selected, info = plan(args)
    print(f'planned {len(selected)} viewpoints over {info["distance"]:.1f} m')
    if not selected:
        print('planner selected nothing; try a larger --budget')
        return 1

    rclpy.init()
    node = PlanRunner(args.namespace, args.linear_speed, args.angular_speed,
                      args.tolerance)
    try:
        node.wait_for_odom()
        while rclpy.ok():
            for step, viewpoint in enumerate(selected, start=1):
                node.get_logger().info(
                    f'[{step}/{len(selected)}] drive to '
                    f'({viewpoint.base_xy[0]:.1f}, {viewpoint.base_xy[1]:.1f})')
                node.drive_to(viewpoint.base_xy)
                node.get_logger().info(f'[{step}/{len(selected)}] posing arm')
                node.strike(list(viewpoint.joint_values.values()),
                            args.posture_seconds)
            node.get_logger().info('plan complete')
            if not args.loop:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
