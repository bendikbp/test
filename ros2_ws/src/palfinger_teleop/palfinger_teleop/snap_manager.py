from dataclasses import dataclass
from math import cos, sin, sqrt

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Joy
from tf2_ros import Buffer, TransformException, TransformListener

from palfinger_msgs.msg import SnapCommand, SnapState, SnapTargetArray

@dataclass
class Candidate:
    target_id: str
    target_frame: str
    tag: str
    x: float
    y: float
    z: float
    max_snap_distance: float
    use_box: bool
    box_half_x: float
    box_half_y: float
    box_half_z: float

class SnapManager(Node):
    def __init__(self) -> None:
        super().__init__("snap_manager")

        command_qos = QoSProfile(depth=1)
        command_qos.reliability = ReliabilityPolicy.RELIABLE

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("targets_topic", "/snap/targets")
        self.declare_parameter("command_topic", "/snap/command")
        self.declare_parameter("state_topic", "/snap/state")
        self.declare_parameter("hook_frame", "hook_link")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("snap_button", 2)
        self.declare_parameter("evaluation_hz", 10.0)
        self.declare_parameter("default_max_snap_distance", 1.0)
        self.declare_parameter("allowed_tags", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("attachable_tags", ["container"])
        self.declare_parameter("crane_world_x", 0.0)
        self.declare_parameter("crane_world_y", 0.0)
        self.declare_parameter("crane_world_z", 0.0)
        self.declare_parameter("crane_world_yaw", 0.0)

        self.joy_topic = str(self.get_parameter("joy_topic").value)
        self.targets_topic = str(self.get_parameter("targets_topic").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.hook_frame = str(self.get_parameter("hook_frame").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.snap_button = int(self.get_parameter("snap_button").value)
        self.default_max_snap_distance = float(self.get_parameter("default_max_snap_distance").value)
        self.crane_world_x = float(self.get_parameter("crane_world_x").value)
        self.crane_world_y = float(self.get_parameter("crane_world_y").value)
        self.crane_world_z = float(self.get_parameter("crane_world_z").value)
        self.crane_world_yaw = float(self.get_parameter("crane_world_yaw").value)
        allowed_tags = self.get_parameter("allowed_tags").value
        self.allowed_tags = {str(tag).strip() for tag in allowed_tags if str(tag).strip()}
        attachable_tags = self.get_parameter("attachable_tags").value
        self.attachable_tags = {str(tag).strip() for tag in attachable_tags if str(tag).strip()}

        self._buttons: list[int] = []
        self._snap_was_pressed = False
        self._candidates: dict[str, Candidate] = {}
        self._best_candidate: Candidate | None = None
        self._best_candidate_distance = float("inf")
        self._attached_candidate: Candidate | None = None
        self._status = "waiting_for_targets"
        self._tf_warned = False

        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._command_pub = self.create_publisher(SnapCommand, self.command_topic, command_qos)
        self._state_pub = self.create_publisher(SnapState, self.state_topic, command_qos)
        self.create_subscription(Joy, self.joy_topic, self._joy_cb, qos_profile_sensor_data)
        self.create_subscription(SnapTargetArray, self.targets_topic, self._targets_cb, command_qos)

        hz = max(float(self.get_parameter("evaluation_hz").value), 1.0)
        self.create_timer(1.0 / hz, self._on_timer)

    def _joy_cb(self, msg: Joy) -> None:
        self._buttons = list(msg.buttons)

    def _targets_cb(self, msg: SnapTargetArray) -> None:
        updated: dict[str, Candidate] = {}
        for target in msg.targets:
            if not target.enabled:
                continue
            tag = str(target.tag).strip()
            if self.allowed_tags and tag not in self.allowed_tags:
                continue
            if self.attachable_tags and tag not in self.attachable_tags:
                continue
            max_snap_distance = float(target.max_snap_distance)
            if max_snap_distance <= 0.0:
                max_snap_distance = self.default_max_snap_distance
            updated[target.target_id] = Candidate(
                target_id=str(target.target_id),
                target_frame=str(target.target_frame),
                tag=tag,
                x=float(target.position.x),
                y=float(target.position.y),
                z=float(target.position.z),
                max_snap_distance=max_snap_distance,
                use_box=bool(target.use_box),
                box_half_x=float(target.box_half_extents.x),
                box_half_y=float(target.box_half_extents.y),
                box_half_z=float(target.box_half_extents.z),
            )
        self._candidates = updated

    def _get_button(self, index: int) -> bool:
        if index < 0 or index >= len(self._buttons):
            return False
        return bool(self._buttons[index])

    def _lookup_hook_position(self) -> tuple[float, float, float] | None:
        try:
            transform = self._tf_buffer.lookup_transform(self.world_frame, self.hook_frame, Time())
        except TransformException:
            self._status = "waiting_for_hook_tf"
            self._best_candidate = None
            self._best_candidate_distance = float("inf")
            self._tf_warned = True
            return None
        self._tf_warned = False
        translation = transform.transform.translation
        local_x = float(translation.x)
        local_y = float(translation.y)
        local_z = float(translation.z)
        cy = cos(self.crane_world_yaw)
        sy = sin(self.crane_world_yaw)
        world_x = self.crane_world_x + (cy * local_x) - (sy * local_y)
        world_y = self.crane_world_y + (sy * local_x) + (cy * local_y)
        world_z = self.crane_world_z + local_z
        return (world_x, world_y, world_z)

    def _distance(self, hook_xyz: tuple[float, float, float], candidate: Candidate) -> float:
        dx = hook_xyz[0] - candidate.x
        dy = hook_xyz[1] - candidate.y
        dz = hook_xyz[2] - candidate.z
        return sqrt((dx * dx) + (dy * dy) + (dz * dz))

    def _box_distance(self, hook_xyz: tuple[float, float, float], candidate: Candidate) -> float:
        dx = max(abs(hook_xyz[0] - candidate.x) - candidate.box_half_x, 0.0)
        dy = max(abs(hook_xyz[1] - candidate.y) - candidate.box_half_y, 0.0)
        dz = max(abs(hook_xyz[2] - candidate.z) - candidate.box_half_z, 0.0)
        return sqrt((dx * dx) + (dy * dy) + (dz * dz))

    def _select_best_candidate(self, hook_xyz: tuple[float, float, float]) -> tuple[Candidate | None, float]:
        best_candidate: Candidate | None = None
        best_distance = float("inf")
        for candidate in self._candidates.values():
            if candidate.use_box:
                distance = self._box_distance(hook_xyz, candidate)
                if distance > 0.0:
                    continue
                # Use center distance only for tie-breaking among valid hitboxes.
                distance = self._distance(hook_xyz, candidate)
            else:
                distance = self._distance(hook_xyz, candidate)
                if distance > candidate.max_snap_distance:
                    continue
            if distance < best_distance:
                best_candidate = candidate
                best_distance = distance
        return best_candidate, best_distance

    def _publish_command(self, action: str, candidate: Candidate | None, distance: float, accepted: bool, reason: str) -> None:
        msg = SnapCommand()
        msg.action = action
        msg.target_id = "" if candidate is None else candidate.target_id
        msg.target_frame = "" if candidate is None else candidate.target_frame
        msg.distance = distance if distance != float("inf") else -1.0
        msg.accepted = accepted
        msg.reason = reason
        self._command_pub.publish(msg)

    def _publish_state(self) -> None:
        msg = SnapState()
        msg.attached = self._attached_candidate is not None
        if self._attached_candidate is not None:
            msg.attached_target_id = self._attached_candidate.target_id
            msg.attached_target_frame = self._attached_candidate.target_frame
        msg.candidate_in_range = self._best_candidate is not None
        if self._best_candidate is not None:
            msg.candidate_target_id = self._best_candidate.target_id
            msg.candidate_target_frame = self._best_candidate.target_frame
            msg.candidate_distance = self._best_candidate_distance
        else:
            msg.candidate_distance = -1.0
        msg.status = self._status
        self._state_pub.publish(msg)

    def _handle_toggle(self) -> None:
        if self._attached_candidate is not None:
            candidate = self._attached_candidate
            self._attached_candidate = None
            self._status = f"detached:{candidate.target_id}"
            self._publish_command("detach", candidate, self._best_candidate_distance, True, "toggle_detach")
            return
        if self._best_candidate is None:
            self._status = "attach_rejected:no_candidate_in_range"
            self._publish_command("attach", None, float("inf"), False, "no_candidate_in_range")
            return
        self._attached_candidate = self._best_candidate
        self._status = f"attached:{self._best_candidate.target_id}"
        self._publish_command("attach", self._best_candidate, self._best_candidate_distance, True, "toggle_attach")

    def _on_timer(self) -> None:
        hook_xyz = self._lookup_hook_position()
        if hook_xyz is None:
            self._best_candidate = None
            self._best_candidate_distance = float("inf")
            self._publish_state()
            return
        self._best_candidate, self._best_candidate_distance = self._select_best_candidate(hook_xyz)
        if self._attached_candidate is None:
            if not self._candidates:
                self._status = "waiting_for_targets"
            elif self._best_candidate is None:
                self._status = "no_target_in_range"
            else:
                self._status = f"candidate_ready:{self._best_candidate.target_id}"
        else:
            self._status = f"attached:{self._attached_candidate.target_id}"

        snap_pressed = self._get_button(self.snap_button)
        if snap_pressed and not self._snap_was_pressed:
            self._handle_toggle()
        self._snap_was_pressed = snap_pressed
        self._publish_state()

def main() -> None:
    rclpy.init()
    node = SnapManager()
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
