"""Publish BIM-reference and as-built discrepancy markers from YAML."""

import math

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
import yaml


SHAPES = {
    'box': Marker.CUBE,
    'cylinder': Marker.CYLINDER,
    'sphere': Marker.SPHERE,
    'mesh': Marker.MESH_RESOURCE,
}


def quaternion_from_rpy(roll, pitch, yaw):
    """Convert roll, pitch, yaw angles to a geometry_msgs quaternion."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def pose_from_mapping(mapping):
    xyz = mapping.get('xyz', [0.0, 0.0, 0.0])
    rpy = mapping.get('rpy', [0.0, 0.0, 0.0])
    return Pose(
        position=Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])),
        orientation=quaternion_from_rpy(*[float(value) for value in rpy]),
    )


class GroundTruthPublisher(Node):
    """Publish durable visualization markers for injected discrepancies."""

    def __init__(self):
        super().__init__('ground_truth_publisher')
        default_path = (
            get_package_share_directory('deltaseek_gazebo')
            + '/config/discrepancies.yaml'
        )
        self.declare_parameter('config_file', default_path)
        self.declare_parameter('topic', '/deltaseek/ground_truth/discrepancies')

        config_file = self.get_parameter('config_file').value
        with open(config_file, encoding='utf-8') as stream:
            self.config = yaml.safe_load(stream)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        topic = self.get_parameter('topic').value
        self.publisher = self.create_publisher(MarkerArray, topic, qos)
        self.timer = self.create_timer(1.0, self.publish_markers)
        self.publish_markers()
        self.get_logger().info(
            f'Loaded {len(self.config.get("discrepancies", []))} discrepancies '
            f'from {config_file}'
        )

    def make_marker(self, item, pose_key, marker_id):
        geometry_key = pose_key.replace('_pose', '_geometry')
        geometry = item.get(geometry_key) or item.get('geometry', {})
        geometry_type = geometry.get('type', geometry.get('shape', 'box'))
        marker = Marker()
        marker.header.frame_id = self.config.get('frame_id', 'odom')
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = pose_key
        marker.id = marker_id
        marker.type = SHAPES.get(geometry_type, Marker.CUBE)
        marker.action = Marker.ADD
        marker.pose = pose_from_mapping(item[pose_key])
        if geometry_type == 'box':
            scale = geometry.get('size', geometry.get('scale', [0.25] * 3))
        elif geometry_type == 'cylinder':
            diameter = 2.0 * float(geometry.get('radius', 0.125))
            scale = [diameter, diameter, float(geometry.get('length', 0.25))]
        elif geometry_type == 'sphere':
            diameter = 2.0 * float(geometry.get('radius', 0.125))
            scale = [diameter, diameter, diameter]
        else:
            scale = geometry.get('scale', [1.0, 1.0, 1.0])
            marker.mesh_resource = geometry.get('uri', '')
        marker.scale = Vector3(
            x=float(scale[0]), y=float(scale[1]), z=float(scale[2])
        )

        if pose_key == 'reference_pose':
            marker.color.r = 0.15
            marker.color.g = 0.45
            marker.color.b = 1.0
            marker.color.a = 0.28
        else:
            severity = max(0.0, min(1.0, float(item.get('severity', 0.5))))
            marker.color.r = 1.0
            marker.color.g = 1.0 - severity
            marker.color.b = 0.05
            marker.color.a = 0.85
        return marker

    def make_label(self, item, marker_id):
        pose_mapping = item.get('actual_pose') or item.get('reference_pose')
        marker = Marker()
        marker.header.frame_id = self.config.get('frame_id', 'odom')
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'labels'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose = pose_from_mapping(pose_mapping)
        marker.pose.position.z += 0.45
        marker.scale.z = 0.20
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = (
            f'{item["id"]}: {item["type"]} '
            f'(severity={float(item.get("severity", 0.0)):.2f})'
        )
        return marker

    def publish_markers(self):
        marker_array = MarkerArray()
        for index, item in enumerate(self.config.get('discrepancies', [])):
            base_id = index * 3
            if item.get('reference_pose') is not None:
                marker_array.markers.append(
                    self.make_marker(item, 'reference_pose', base_id)
                )
            if item.get('actual_pose') is not None:
                marker_array.markers.append(
                    self.make_marker(item, 'actual_pose', base_id + 1)
                )
            marker_array.markers.append(self.make_label(item, base_id + 2))
        self.publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

