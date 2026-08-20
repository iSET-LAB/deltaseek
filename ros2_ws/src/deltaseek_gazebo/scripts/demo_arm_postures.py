"""Drive the UR5e through the planner's inspection postures, on repeat.

Useful for watching what a viewpoint actually looks like: the postures are the
same ones viewpoints.py composes with base poses, so the wrist camera shows
exactly what the planner assumes it would see from each one.
"""

import argparse

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from deltaseek_gazebo.viewpoints import ARM_POSTURES


JOINTS = [
    'arm_0_shoulder_pan_joint',
    'arm_0_shoulder_lift_joint',
    'arm_0_elbow_joint',
    'arm_0_wrist_1_joint',
    'arm_0_wrist_2_joint',
    'arm_0_wrist_3_joint',
]


class PostureDemo(Node):
    """Sends one trajectory per posture and waits for each to finish."""

    def __init__(self, namespace, hold):
        super().__init__('demo_arm_postures')
        self.hold = hold
        action = (f'/{namespace}/arm_0_joint_trajectory_controller'
                  '/follow_joint_trajectory')
        self.client = ActionClient(self, FollowJointTrajectory, action)
        self.get_logger().info(f'waiting for {action}')
        self.client.wait_for_server()

    def go(self, name, positions, seconds):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(
            joint_names=JOINTS,
            points=[JointTrajectoryPoint(
                positions=[float(v) for v in positions],
                time_from_start=Duration(sec=int(seconds)),
            )],
        )
        self.get_logger().info(f'-> {name}')
        handle = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, handle)
        result = handle.result().get_result_async()
        rclpy.spin_until_future_complete(self, result)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespace', default='a300_00000')
    parser.add_argument('--seconds', type=float, default=4.0,
                        help='Time allowed to reach each posture.')
    parser.add_argument('--cycles', type=int, default=0,
                        help='Number of passes; 0 repeats until stopped.')
    args = parser.parse_args()

    rclpy.init()
    node = PostureDemo(args.namespace, args.seconds)
    try:
        cycle = 0
        while args.cycles == 0 or cycle < args.cycles:
            for name, positions in ARM_POSTURES.items():
                node.go(name, positions, args.seconds)
            cycle += 1
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
