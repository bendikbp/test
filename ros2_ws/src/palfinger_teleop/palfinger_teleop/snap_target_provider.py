from dataclasses import dataclass
import math

from geometry_msgs.msg import Point, Pose, Vector3
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from palfinger_msgs.msg import SnapTarget, SnapTargetArray

@dataclass(frozen=True)
class PoseRPY:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

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

class SnapTargetProvider(Node):
    def __init__(self) -> None:
        super().__init__("snap_target_provider")

        command_qos = QoSProfile(depth=1)
        command_qos.reliability = ReliabilityPolicy.RELIABLE

        self.declare_parameter("targets_topic", "/snap/targets")
        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("ship_name", "havyard_ship")
        self.declare_parameter("ship_tag", "ship")
        self.declare_parameter("ship_x", 32.0)
        self.declare_parameter("ship_y", 0.0)
        self.declare_parameter("ship_z", -6.0)
        self.declare_parameter("ship_roll", 0.0)
        self.declare_parameter("ship_pitch", 0.0)
        self.declare_parameter("ship_yaw", 3.14)
        self.declare_parameter("ship_max_snap_distance", 2.0)
        self.declare_parameter("container_name", "container_blue")
        self.declare_parameter("container_tag", "container")
        self.declare_parameter("container_x", 35.2)
        self.declare_parameter("container_y", 11.6)
        self.declare_parameter("container_z", 4.8)
        self.declare_parameter("container_roll", 0.0)
        self.declare_parameter("container_pitch", 0.0)
        self.declare_parameter("container_yaw", 1.5707)
        self.declare_parameter("container_max_snap_distance", 2.25)
        self.declare_parameter("container_pose_topic", "/snap/container_pose")

        self.targets_topic = str(self.get_parameter("targets_topic").value)
        self.publish_hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.ship_name = str(self.get_parameter("ship_name").value)
        self.ship_tag = str(self.get_parameter("ship_tag").value)
        self.ship_pose = PoseRPY(
            x=float(self.get_parameter("ship_x").value),
            y=float(self.get_parameter("ship_y").value),
            z=float(self.get_parameter("ship_z").value),
            roll=float(self.get_parameter("ship_roll").value),
            pitch=float(self.get_parameter("ship_pitch").value),
            yaw=float(self.get_parameter("ship_yaw").value),
        )
        self.ship_max_snap_distance = float(self.get_parameter("ship_max_snap_distance").value)
        self.container_name = str(self.get_parameter("container_name").value)
        self.container_tag = str(self.get_parameter("container_tag").value)
        self.container_pose = PoseRPY(
            x=float(self.get_parameter("container_x").value),
            y=float(self.get_parameter("container_y").value),
            z=float(self.get_parameter("container_z").value),
            roll=float(self.get_parameter("container_roll").value),
            pitch=float(self.get_parameter("container_pitch").value),
            yaw=float(self.get_parameter("container_yaw").value),
        )
        self.container_max_snap_distance = float(self.get_parameter("container_max_snap_distance").value)
        self.container_pose_topic = str(self.get_parameter("container_pose_topic").value)

        self._pub = self.create_publisher(SnapTargetArray, self.targets_topic, command_qos)
        pose_qos = QoSProfile(depth=1)
        pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        pose_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(Pose, self.container_pose_topic, self._container_pose_cb, pose_qos)
        self.create_timer(1.0 / self.publish_hz, self._publish_targets)

    def _container_pose_cb(self, msg: Pose) -> None:
        roll, pitch, yaw = quaternion_to_rpy(
            float(msg.orientation.x),
            float(msg.orientation.y),
            float(msg.orientation.z),
            float(msg.orientation.w),
        )
        self.container_pose = PoseRPY(
            x=float(msg.position.x),
            y=float(msg.position.y),
            z=float(msg.position.z),
            roll=roll,
            pitch=pitch,
            yaw=yaw,
        )

    def _make_world_point(self, pose: PoseRPY, local_xyz: tuple[float, float, float]) -> Point:
        dx, dy, dz = rotate_rpy(local_xyz[0], local_xyz[1], local_xyz[2], pose.roll, pose.pitch, pose.yaw)
        point = Point()
        point.x = pose.x + dx
        point.y = pose.y + dy
        point.z = pose.z + dz
        return point

    def _make_target(
        self,
        target_id: str,
        target_frame: str,
        tag: str,
        world_point: Point,
        max_snap_distance: float,
        *,
        use_box: bool = False,
        box_half_extents: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> SnapTarget:
        target = SnapTarget()
        target.target_id = target_id
        target.target_frame = target_frame
        target.tag = tag
        target.position = world_point
        target.max_snap_distance = max_snap_distance
        target.enabled = True
        target.use_box = use_box
        target.box_half_extents = Vector3(
            x=float(box_half_extents[0]),
            y=float(box_half_extents[1]),
            z=float(box_half_extents[2]),
        )
        return target

    def _build_ship_targets(self) -> list[SnapTarget]:
        samples = {
            "deck_center": (0.0, 0.0, 10.0),
            "deck_fore_port": (35.0, 8.0, 10.0),
            "deck_fore_starboard": (35.0, -8.0, 10.0),
            "deck_aft_port": (-35.0, 8.0, 10.0),
            "deck_aft_starboard": (-35.0, -8.0, 10.0),
        }
        return [
            self._make_target(
                target_id=f"{self.ship_name}:{name}",
                target_frame=f"{self.ship_name}/{name}",
                tag=self.ship_tag,
                world_point=self._make_world_point(self.ship_pose, local_xyz),
                max_snap_distance=self.ship_max_snap_distance,
            )
            for name, local_xyz in samples.items()
        ]

    def _build_container_targets(self) -> list[SnapTarget]:
        base_offset_x = -0.024303
        base_offset_y = 1.411676
        half_length = 6.1 / 2.0
        half_width = 2.5 / 2.0
        collision_top_z = -0.00233 + (2.99 / 2.0)
        top_z = collision_top_z + 0.10
        hitbox_center_z = collision_top_z + 0.45
        samples = {
            "top_front_left": (base_offset_x + half_length, base_offset_y + half_width, top_z),
            "top_front_right": (base_offset_x + half_length, base_offset_y - half_width, top_z),
            "top_back_left": (base_offset_x - half_length, base_offset_y + half_width, top_z),
            "top_back_right": (base_offset_x - half_length, base_offset_y - half_width, top_z),
            "top_center": (base_offset_x, base_offset_y, top_z),
            "top_front_center": (base_offset_x + half_length, base_offset_y, top_z),
            "top_back_center": (base_offset_x - half_length, base_offset_y, top_z),
            "top_left_center": (base_offset_x, base_offset_y + half_width, top_z),
            "top_right_center": (base_offset_x, base_offset_y - half_width, top_z),
        }
        targets = [
            self._make_target(
                target_id=f"{self.container_name}:{name}",
                target_frame=f"{self.container_name}/{name}",
                tag=self.container_tag,
                world_point=self._make_world_point(self.container_pose, local_xyz),
                max_snap_distance=self.container_max_snap_distance,
            )
            for name, local_xyz in samples.items()
        ]
        # Large logical hitbox over the top of the container for easier hook testing.
        # Roughly 50% of the container footprint, with generous vertical tolerance.
        targets.append(
            self._make_target(
                target_id=f"{self.container_name}:top_hitbox",
                target_frame=f"{self.container_name}/top_hitbox",
                tag=self.container_tag,
                world_point=self._make_world_point(self.container_pose, (base_offset_x, base_offset_y, hitbox_center_z)),
                max_snap_distance=self.container_max_snap_distance,
                use_box=True,
                box_half_extents=(1.525, 0.625, 0.45),
            )
        )
        return targets

    def _publish_targets(self) -> None:
        msg = SnapTargetArray()
        msg.targets.extend(self._build_ship_targets())
        msg.targets.extend(self._build_container_targets())
        self._pub.publish(msg)

def main() -> None:
    rclpy.init()
    node = SnapTargetProvider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass

if __name__ == "__main__":
    main()
