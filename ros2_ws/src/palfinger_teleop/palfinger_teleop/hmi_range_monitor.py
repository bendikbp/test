#!/usr/bin/env python3
from dataclasses import dataclass
import math

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from palfinger_msgs.msg import HmiRangeState, SnapState

@dataclass
class PoseRPY:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


@dataclass(frozen=True)
class SurfaceBox:
    center_x: float
    center_y: float
    top_z: float
    half_x: float
    half_y: float

def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * ((w * x) + (y * z))
    cosr_cosp = 1.0 - (2.0 * ((x * x) + (y * y)))
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * ((w * y) - (z * x))
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def rotate_rpy(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> tuple[float, float, float]:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    x1 = x
    y1 = (cr * y) - (sr * z)
    z1 = (sr * y) + (cr * z)
    x2 = (cp * x1) + (sp * z1)
    y2 = y1
    z2 = (-sp * x1) + (cp * z1)
    x3 = (cy * x2) - (sy * y2)
    y3 = (sy * x2) + (cy * y2)
    z3 = z2
    return x3, y3, z3

def inverse_rotate_rpy(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> tuple[float, float, float]:
    return rotate_rpy(x, y, z, -roll, -pitch, -yaw)

class HmiRangeMonitor(Node):
    def __init__(self) -> None:
        super().__init__("hmi_range_monitor")

        command_qos = QoSProfile(depth=1)
        command_qos.reliability = ReliabilityPolicy.RELIABLE

        self.declare_parameter("ranges_topic", "/crane/hmi/ranges")
        self.declare_parameter("hook_frame", "hook_link")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("crane_world_x", 0.0)
        self.declare_parameter("crane_world_y", 0.0)
        self.declare_parameter("crane_world_z", 0.0)
        self.declare_parameter("crane_world_yaw", 0.0)
        self.declare_parameter("container_pose_topic", "/snap/container_pose")
        self.declare_parameter("ship_odom_topic", "/model/havyard_ship/odometry")
        self.declare_parameter("ship_x", 32.0)
        self.declare_parameter("ship_y", 0.0)
        self.declare_parameter("ship_z", -6.0)
        self.declare_parameter("ship_roll", 0.0)
        self.declare_parameter("ship_pitch", 0.0)
        self.declare_parameter("ship_yaw", 3.14)
        self.declare_parameter("platform_x", 0.0)
        self.declare_parameter("platform_y", 0.0)
        self.declare_parameter("platform_z", 0.0)
        self.declare_parameter("platform_roll", 0.0)
        self.declare_parameter("platform_pitch", 0.0)
        self.declare_parameter("platform_yaw", 0.0)
        self.declare_parameter("snap_state_topic", "/snap/state")
        self.declare_parameter("attached_container_mass_kg", 1000.0)

        self.ranges_topic = str(self.get_parameter("ranges_topic").value)
        self.hook_frame = str(self.get_parameter("hook_frame").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.publish_hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.crane_world_x = float(self.get_parameter("crane_world_x").value)
        self.crane_world_y = float(self.get_parameter("crane_world_y").value)
        self.crane_world_z = float(self.get_parameter("crane_world_z").value)
        self.crane_world_yaw = float(self.get_parameter("crane_world_yaw").value)
        self.container_pose_topic = str(self.get_parameter("container_pose_topic").value)
        self.ship_odom_topic = str(self.get_parameter("ship_odom_topic").value)

        self._container_surface = SurfaceBox(
            center_x=-0.024303,
            center_y=1.411676,
            top_z=-0.00233 + (2.99 / 2.0),
            half_x=6.1 / 2.0,
            half_y=2.5 / 2.0,
        )
        # Tightened against the transformed Havyard deck collision mesh in
        # model_havyard.sdf. The internal collision pose rotates the mesh into
        # the ship link frame, so these values are already in ship-local deck
        # coordinates as seen by odometry.
        self._ship_surface = SurfaceBox(
            center_x=0.0,
            center_y=-7.1735,
            top_z=9.1372,
            half_x=8.6,
            half_y=34.92,
        )
        # Tightened against the Hugin collision mesh after applying the link
        # flip and collision z-offset from model.sdf. The platform has multiple
        # height levels, so this rectangle focuses on the crane-side working
        # deck instead of the full structure envelope. The bounds come from the
        # deck-height band of the transformed collision mesh.
        self._platform_surface = SurfaceBox(
            center_x=13.45,
            center_y=8.85,
            top_z=48.87,
            half_x=5.35,
            half_y=14.25,
        )

        self._container_pose = PoseRPY(
            x=35.2,
            y=11.6,
            z=4.8,
            roll=0.0,
            pitch=0.0,
            yaw=1.5707,
        )
        self._ship_pose = PoseRPY(
            x=float(self.get_parameter("ship_x").value),
            y=float(self.get_parameter("ship_y").value),
            z=float(self.get_parameter("ship_z").value),
            roll=float(self.get_parameter("ship_roll").value),
            pitch=float(self.get_parameter("ship_pitch").value),
            yaw=float(self.get_parameter("ship_yaw").value),
        )
        self._platform_pose = PoseRPY(
            x=float(self.get_parameter("platform_x").value),
            y=float(self.get_parameter("platform_y").value),
            z=float(self.get_parameter("platform_z").value),
            roll=float(self.get_parameter("platform_roll").value),
            pitch=float(self.get_parameter("platform_pitch").value),
            yaw=float(self.get_parameter("platform_yaw").value),
        )

        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(HmiRangeState, self.ranges_topic, command_qos)
        self._snap_state_topic = str(self.get_parameter("snap_state_topic").value)
        self._attached_container_mass_kg = float(self.get_parameter("attached_container_mass_kg").value)
        self._load_mass_kg = 0.0

        pose_qos = QoSProfile(depth=1)
        pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        pose_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(Pose, self.container_pose_topic, self._container_pose_cb, pose_qos)
        self.create_subscription(Odometry, self.ship_odom_topic, self._ship_odom_cb, qos_profile_sensor_data)
        self.create_subscription(SnapState, self._snap_state_topic, self._snap_state_cb, command_qos)
        self.create_timer(1.0 / self.publish_hz, self._on_timer)

    def _container_pose_cb(self, msg: Pose) -> None:
        roll, pitch, yaw = quaternion_to_rpy(
            float(msg.orientation.x),
            float(msg.orientation.y),
            float(msg.orientation.z),
            float(msg.orientation.w),
        )
        self._container_pose = PoseRPY(
            x=float(msg.position.x),
            y=float(msg.position.y),
            z=float(msg.position.z),
            roll=roll,
            pitch=pitch,
            yaw=yaw,
        )

    def _ship_odom_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        roll, pitch, yaw = quaternion_to_rpy(q.x, q.y, q.z, q.w)
        self._ship_pose = PoseRPY(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            z=float(msg.pose.pose.position.z),
            roll=roll,
            pitch=pitch,
            yaw=yaw,
        )

    def _snap_state_cb(self, msg: SnapState) -> None:
        if msg.attached and str(msg.attached_target_id).startswith("container_"):
            self._load_mass_kg = self._attached_container_mass_kg
        else:
            self._load_mass_kg = 0.0

    def _lookup_hook_position(self) -> tuple[float, float, float] | None:
        try:
            transform = self._tf_buffer.lookup_transform(self.world_frame, self.hook_frame, Time())
        except TransformException:
            return None
        translation = transform.transform.translation
        local_x = float(translation.x)
        local_y = float(translation.y)
        local_z = float(translation.z)
        cy = math.cos(self.crane_world_yaw)
        sy = math.sin(self.crane_world_yaw)
        world_x = self.crane_world_x + (cy * local_x) - (sy * local_y)
        world_y = self.crane_world_y + (sy * local_x) + (cy * local_y)
        world_z = self.crane_world_z + local_z
        return world_x, world_y, world_z

    def _surface_clearance(
        self,
        hook_xyz: tuple[float, float, float],
        pose: PoseRPY,
        surface: SurfaceBox,
    ) -> tuple[float, bool]:
        rel_x = hook_xyz[0] - pose.x
        rel_y = hook_xyz[1] - pose.y
        rel_z = hook_xyz[2] - pose.z
        local_x, local_y, local_z = inverse_rotate_rpy(rel_x, rel_y, rel_z, pose.roll, pose.pitch, pose.yaw)
        clearance = local_z - surface.top_z
        over_surface = abs(local_x - surface.center_x) <= surface.half_x and abs(local_y - surface.center_y) <= surface.half_y
        return clearance, over_surface

    def _on_timer(self) -> None:
        hook_xyz = self._lookup_hook_position()
        msg = HmiRangeState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame

        if hook_xyz is None:
            msg.hook_world_x = float("nan")
            msg.hook_world_y = float("nan")
            msg.hook_world_z = float("nan")
            msg.hook_to_container_top = float("nan")
            msg.hook_over_container = False
            msg.hook_to_ship_deck = float("nan")
            msg.hook_over_ship_deck = False
            msg.hook_to_platform_deck = float("nan")
            msg.hook_over_platform_deck = False
            msg.load_mass_kg = self._load_mass_kg
            msg.status = "waiting_for_hook_tf"
            self._publisher.publish(msg)
            return

        msg.hook_world_x = hook_xyz[0]
        msg.hook_world_y = hook_xyz[1]
        msg.hook_world_z = hook_xyz[2]
        msg.hook_to_container_top, msg.hook_over_container = self._surface_clearance(
            hook_xyz, self._container_pose, self._container_surface
        )
        msg.hook_to_ship_deck, msg.hook_over_ship_deck = self._surface_clearance(
            hook_xyz, self._ship_pose, self._ship_surface
        )
        msg.hook_to_platform_deck, msg.hook_over_platform_deck = self._surface_clearance(
            hook_xyz, self._platform_pose, self._platform_surface
        )
        msg.load_mass_kg = self._load_mass_kg
        msg.status = "ok"
        self._publisher.publish(msg)

def main() -> None:
    rclpy.init()
    node = HmiRangeMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
