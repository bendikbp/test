import math
import os
import re
import tempfile
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose, SpawnEntity
from tf2_ros import Buffer, TransformException, TransformListener

from palfinger_msgs.msg import SnapCommand

@dataclass(frozen=True)
class LocalAnchor:
    x: float
    y: float
    z: float

def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = (cr * cp * cy) + (sr * sp * sy)
    qx = (sr * cp * cy) - (cr * sp * sy)
    qy = (cr * sp * cy) + (sr * cp * sy)
    qz = (cr * cp * sy) - (sr * sp * cy)
    return qx, qy, qz, qw

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

class SnapExecutor(Node):
    def __init__(self) -> None:
        super().__init__("snap_executor")

        self.declare_parameter("command_topic", "/snap/command")
        self.declare_parameter("hook_frame", "hook_link")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("set_pose_service", "/world/crane_world/set_pose")
        self.declare_parameter("update_hz", 20.0)
        self.declare_parameter("container_roll", 0.0)
        self.declare_parameter("container_pitch", 0.0)
        self.declare_parameter("container_yaw", 1.5707)
        self.declare_parameter("crane_world_x", 0.0)
        self.declare_parameter("crane_world_y", 0.0)
        self.declare_parameter("crane_world_z", 0.0)
        self.declare_parameter("crane_world_yaw", 0.0)
        self.declare_parameter("hook_clearance_z", 0.75)
        self.declare_parameter("follow_alpha", 0.35)
        self.declare_parameter("container_model_sdf", "")
        self.declare_parameter("create_service", "/world/crane_world/create")
        self.declare_parameter("remove_service", "/world/crane_world/remove")
        self.declare_parameter("container_pose_topic", "/snap/container_pose")
        self.declare_parameter("replace_model_on_attach", False)
        self.declare_parameter("detach_spawn_lift_z", 0.10)

        self.command_topic = str(self.get_parameter("command_topic").value)
        self.hook_frame = str(self.get_parameter("hook_frame").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.set_pose_service = str(self.get_parameter("set_pose_service").value)
        self.update_hz = max(float(self.get_parameter("update_hz").value), 1.0)
        self.container_roll = float(self.get_parameter("container_roll").value)
        self.container_pitch = float(self.get_parameter("container_pitch").value)
        self.container_yaw = float(self.get_parameter("container_yaw").value)
        self.crane_world_x = float(self.get_parameter("crane_world_x").value)
        self.crane_world_y = float(self.get_parameter("crane_world_y").value)
        self.crane_world_z = float(self.get_parameter("crane_world_z").value)
        self.crane_world_yaw = float(self.get_parameter("crane_world_yaw").value)
        self.hook_clearance_z = float(self.get_parameter("hook_clearance_z").value)
        self.follow_alpha = min(max(float(self.get_parameter("follow_alpha").value), 0.0), 1.0)
        self.container_model_sdf = str(self.get_parameter("container_model_sdf").value)
        self.create_service_name = str(self.get_parameter("create_service").value)
        self.remove_service_name = str(self.get_parameter("remove_service").value)
        self.container_pose_topic = str(self.get_parameter("container_pose_topic").value)
        self.replace_model_on_attach = bool(self.get_parameter("replace_model_on_attach").value)
        self.detach_spawn_lift_z = max(float(self.get_parameter("detach_spawn_lift_z").value), 0.0)

        base_offset_x = -0.024303
        base_offset_y = 1.411676
        half_length = 6.1 / 2.0
        half_width = 2.5 / 2.0
        collision_top_z = -0.00233 + (2.99 / 2.0)
        top_z = collision_top_z + 0.10
        self.anchor_offsets = {
            "top_front_left": LocalAnchor(base_offset_x + half_length, base_offset_y + half_width, top_z),
            "top_front_right": LocalAnchor(base_offset_x + half_length, base_offset_y - half_width, top_z),
            "top_back_left": LocalAnchor(base_offset_x - half_length, base_offset_y + half_width, top_z),
            "top_back_right": LocalAnchor(base_offset_x - half_length, base_offset_y - half_width, top_z),
            "top_center": LocalAnchor(base_offset_x, base_offset_y, top_z),
            "top_hitbox": LocalAnchor(base_offset_x, base_offset_y, top_z),
            "top_front_center": LocalAnchor(base_offset_x + half_length, base_offset_y, top_z),
            "top_back_center": LocalAnchor(base_offset_x - half_length, base_offset_y, top_z),
            "top_left_center": LocalAnchor(base_offset_x, base_offset_y + half_width, top_z),
            "top_right_center": LocalAnchor(base_offset_x, base_offset_y - half_width, top_z),
        }

        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._set_pose_client = self.create_client(SetEntityPose, self.set_pose_service)
        self._create_client = self.create_client(SpawnEntity, self.create_service_name)
        self._remove_client = self.create_client(DeleteEntity, self.remove_service_name)
        command_qos = QoSProfile(depth=1)
        command_qos.reliability = ReliabilityPolicy.RELIABLE
        pose_qos = QoSProfile(depth=1)
        pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        pose_qos.reliability = ReliabilityPolicy.RELIABLE
        self._container_pose_pub = self.create_publisher(Pose, self.container_pose_topic, pose_qos)
        self.create_subscription(SnapCommand, self.command_topic, self._command_cb, command_qos)
        self.create_timer(1.0 / self.update_hz, self._on_timer)

        self._attached_model_name: str | None = None
        self._attached_anchor: LocalAnchor | None = None
        self._pending_future = None
        self._last_pose: Pose | None = None
        self._pending_replace = False
        self._static_container_model_sdf = (
            self._prepare_attached_sdf(self.container_model_sdf) if self.replace_model_on_attach else ""
        )
        self._tf_warned = False

        self.get_logger().info(
            f"Snap executor active on {self.command_topic}. "
            f"Service={self.set_pose_service}, hook_frame={self.hook_frame}"
        )

    def _publish_container_pose(self, pose: Pose) -> None:
        self._container_pose_pub.publish(pose)

    def _prepare_attached_sdf(self, source_path: str) -> str:
        if not source_path or not os.path.exists(source_path):
            return ""
        with open(source_path, "r", encoding="utf-8") as handle:
            sdf_text = handle.read()
        sdf_text = sdf_text.replace("<static>false</static>", "<static>true</static>", 1)
        sdf_text = re.sub(r"<collision\\b.*?</collision>", "", sdf_text, count=1, flags=re.DOTALL)
        fd, temp_path = tempfile.mkstemp(prefix="attached_container_", suffix=".sdf")
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(sdf_text)
        return temp_path

    def _lifted_pose(self, pose: Pose, dz: float) -> Pose:
        lifted = Pose()
        lifted.position.x = float(pose.position.x)
        lifted.position.y = float(pose.position.y)
        lifted.position.z = float(pose.position.z) + dz
        lifted.orientation.x = float(pose.orientation.x)
        lifted.orientation.y = float(pose.orientation.y)
        lifted.orientation.z = float(pose.orientation.z)
        lifted.orientation.w = float(pose.orientation.w)
        return lifted

    def _request_remove_and_spawn(self, model_name: str, sdf_path: str, pose: Pose) -> bool:
        if not self._remove_client.service_is_ready() or not self._create_client.service_is_ready():
            self.get_logger().warning("Create/remove services are not ready for snap replacement.")
            return False

        remove_req = DeleteEntity.Request()
        remove_req.entity = Entity(name=model_name, type=Entity.MODEL)
        remove_future = self._remove_client.call_async(remove_req)
        remove_future.add_done_callback(
            lambda future: self._on_remove_done(future, model_name, sdf_path, pose)
        )
        self._pending_replace = True
        return True

    def _on_remove_done(self, future, model_name: str, sdf_path: str, pose: Pose) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed removing {model_name}: {exc}")
            self._pending_replace = False
            return
        if not response.success:
            self.get_logger().error(f"Gazebo remove failed for {model_name}.")
            self._pending_replace = False
            return

        spawn_req = SpawnEntity.Request()
        spawn_req.entity_factory.name = model_name
        spawn_req.entity_factory.allow_renaming = False
        spawn_req.entity_factory.sdf_filename = sdf_path
        spawn_req.entity_factory.pose = pose
        spawn_req.entity_factory.relative_to = "world"
        spawn_future = self._create_client.call_async(spawn_req)
        spawn_future.add_done_callback(lambda f: self._on_spawn_done(f, model_name))

    def _on_spawn_done(self, future, model_name: str) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed spawning {model_name}: {exc}")
            self._pending_replace = False
            return
        if not response.success:
            self.get_logger().error(f"Gazebo spawn failed for {model_name}.")
            self._pending_replace = False
            return
        self._pending_replace = False

    def _command_cb(self, msg: SnapCommand) -> None:
        if not msg.accepted:
            return

        if msg.action == "detach":
            if self._attached_model_name is not None:
                if self._last_pose is not None:
                    if self.replace_model_on_attach and self.container_model_sdf:
                        detach_pose = self._lifted_pose(self._last_pose, self.detach_spawn_lift_z)
                        self._request_remove_and_spawn(self._attached_model_name, self.container_model_sdf, detach_pose)
                        self._publish_container_pose(detach_pose)
                    else:
                        self._publish_container_pose(self._last_pose)
                self.get_logger().info(f"Detached {self._attached_model_name} from hook follow mode.")
            self._attached_model_name = None
            self._attached_anchor = None
            self._last_pose = None
            return

        if msg.action != "attach" or not msg.target_id:
            return

        model_name, _, anchor_name = msg.target_id.partition(":")
        if not model_name.startswith("container_"):
            self.get_logger().info(
                f"Snap attach target {msg.target_id} is not a container. "
                "Deck attach is not implemented yet in snap_executor."
            )
            return
        anchor = self.anchor_offsets.get(anchor_name)
        if anchor is None:
            self.get_logger().warning(f"Unsupported container anchor '{anchor_name}' in target {msg.target_id}.")
            return

        hook_xyz = self._lookup_hook_position()
        if hook_xyz is None:
            self.get_logger().warning("Cannot attach container: hook pose unavailable.")
            return
        attach_pose = self._build_request(model_name, hook_xyz, anchor).pose
        if self._static_container_model_sdf:
            if not self._request_remove_and_spawn(model_name, self._static_container_model_sdf, attach_pose):
                return

        self._attached_model_name = model_name
        self._attached_anchor = anchor
        self._last_pose = attach_pose
        self._publish_container_pose(attach_pose)
        self.get_logger().info(f"Following hook with model {model_name} using anchor {anchor_name}.")

    def _lookup_hook_position(self) -> tuple[float, float, float] | None:
        try:
            transform = self._tf_buffer.lookup_transform(self.world_frame, self.hook_frame, Time())
        except TransformException:
            if not self._tf_warned:
                self.get_logger().warn(
                    f"Waiting for TF {self.world_frame} -> {self.hook_frame}. "
                    "Attached payload updates will pause until TF is available again."
                )
                self._tf_warned = True
            return None
        self._tf_warned = False
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

    def _build_request(self, model_name: str, hook_xyz: tuple[float, float, float], anchor: LocalAnchor) -> SetEntityPose.Request:
        ox, oy, oz = rotate_rpy(
            anchor.x,
            anchor.y,
            anchor.z + self.hook_clearance_z,
            self.container_roll,
            self.container_pitch,
            self.container_yaw,
        )
        pose = Pose()
        pose.position.x = hook_xyz[0] - ox
        pose.position.y = hook_xyz[1] - oy
        pose.position.z = hook_xyz[2] - oz
        qx, qy, qz, qw = quaternion_from_rpy(self.container_roll, self.container_pitch, self.container_yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        if self._last_pose is not None:
            alpha = self.follow_alpha
            pose.position.x = ((1.0 - alpha) * self._last_pose.position.x) + (alpha * pose.position.x)
            pose.position.y = ((1.0 - alpha) * self._last_pose.position.y) + (alpha * pose.position.y)
            pose.position.z = ((1.0 - alpha) * self._last_pose.position.z) + (alpha * pose.position.z)

        request = SetEntityPose.Request()
        request.entity = Entity(name=model_name, type=Entity.MODEL)
        request.pose = pose
        return request

    def _on_timer(self) -> None:
        if self._attached_model_name is None or self._attached_anchor is None:
            return
        if self._pending_replace:
            return
        if not self._set_pose_client.service_is_ready():
            return
        if self._pending_future is not None and not self._pending_future.done():
            return

        hook_xyz = self._lookup_hook_position()
        if hook_xyz is None:
            return

        request = self._build_request(self._attached_model_name, hook_xyz, self._attached_anchor)
        self._last_pose = request.pose
        self._publish_container_pose(request.pose)
        self._pending_future = self._set_pose_client.call_async(request)

def main() -> None:
    rclpy.init()
    node = SnapExecutor()
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
