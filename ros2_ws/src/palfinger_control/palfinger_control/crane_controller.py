import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from palfinger_msgs.msg import CraneCommand, CraneTarget
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class CraneController(Node):
    """
    Subscribes:
        /crane/cmd (palfinger_msgs/CraneCommand)
        /joint_states (sensor_msgs/JointState)

    Publishes:
        Configurable command array topic (std_msgs/Float64MultiArray)

    Converts normalized rates [-1..1] to joint velocity setpoints, integrates
    those rates over time, and publishes absolute position commands.
    """

    def __init__(self):
        # Avoid name collision with ros2_control controller identifiers.
        super().__init__("crane_cmd_bridge")

        # Parameters
        self.declare_parameter("cmd_in_topic", "/crane/cmd")
        self.declare_parameter("priority_cmd_in_topic", "")
        self.declare_parameter("disturbance_cmd_topic", "/crane/disturbance_cmd")
        self.declare_parameter("priority_disturbance_cmd_topic", "")
        self.declare_parameter("target_in_topic", "/crane/target")
        self.declare_parameter("priority_target_in_topic", "")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter(
            "cmd_out_topic",
            "/crane_position_controller/commands"
        )

        # Order must match crane_position_controller joint order
        self.declare_parameter(
            "joint_names",
            ["boom_yaw_joint", "trolley_joint"]
        )

        # Maps CraneCommand fields into positions in joint_names.
        # Use -1 to disable a command field.
        self.declare_parameter("idx_slew", 0)
        self.declare_parameter("idx_boom", -1)
        self.declare_parameter("idx_winch", -1)
        self.declare_parameter("idx_sway", -1)

        self.declare_parameter("max_slew_vel", 0.10)
        self.declare_parameter("max_slew_accel", 0.6)
        self.declare_parameter("max_boom_vel", 0.5)
        self.declare_parameter("max_winch_vel", 0.4)
        self.declare_parameter("max_winch_accel", 0.8)
        self.declare_parameter("reach_aware_slew", True)
        self.declare_parameter("min_slew_scale_at_max_reach", 0.25)
        self.declare_parameter("reach_weight_trolley", 0.8)
        self.declare_parameter("reach_weight_winch", 0.2)
        self.declare_parameter("sway_joint_name", "wire_sway_joint")
        self.declare_parameter("sway_joint_names", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("sway_brake_gain", 6.0)
        self.declare_parameter("sway_soft_limit", 0.02)
        self.declare_parameter("sway_hard_limit", 0.04)
        self.declare_parameter("slew_zero_threshold", 0.0)
        self.declare_parameter("boom_zero_threshold", 0.0)
        self.declare_parameter("winch_zero_threshold", 0.0)
        self.declare_parameter("lower_limits", [-3.14159, -1.0])
        self.declare_parameter("upper_limits", [3.14159, 1.0])
        self.declare_parameter("max_target_error", [-1.0, -1.0])
        # Software mimic rules for engines that don't support joint mimic constraints.
        # Format per entry: "dst_joint:src_joint:multiplier:offset"
        self.declare_parameter("mimic_rules", Parameter.Type.STRING_ARRAY)

        self.declare_parameter("timeout_sec", 0.25)
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("wait_for_joint_states", True)
        self.declare_parameter("startup_target_slew", 0.0)
        self.declare_parameter("startup_target_boom", 0.0)
        self.declare_parameter("startup_target_winch", -1.0)

        # Load parameters
        self.joint_names = list(self.get_parameter("joint_names").value)

        self.idx_slew = int(self.get_parameter("idx_slew").value)
        self.idx_boom = int(self.get_parameter("idx_boom").value)
        self.idx_winch = int(self.get_parameter("idx_winch").value)
        self.idx_sway = int(self.get_parameter("idx_sway").value)

        self.max_slew = float(self.get_parameter("max_slew_vel").value)
        self.max_slew_accel = float(self.get_parameter("max_slew_accel").value)
        self.max_boom = float(self.get_parameter("max_boom_vel").value)
        self.max_winch = float(self.get_parameter("max_winch_vel").value)
        self.max_winch_accel = float(self.get_parameter("max_winch_accel").value)
        self.reach_aware_slew = bool(self.get_parameter("reach_aware_slew").value)
        self.min_slew_scale_at_max_reach = float(self.get_parameter("min_slew_scale_at_max_reach").value)
        self.reach_weight_trolley = float(self.get_parameter("reach_weight_trolley").value)
        self.reach_weight_winch = float(self.get_parameter("reach_weight_winch").value)
        self.sway_joint_name = str(self.get_parameter("sway_joint_name").value)
        raw_sway_joint_names = []
        try:
            sway_joint_names_param = self.get_parameter("sway_joint_names")
            if sway_joint_names_param.type_ != Parameter.Type.NOT_SET:
                raw_sway_joint_names = list(sway_joint_names_param.value)
        except ParameterUninitializedException:
            raw_sway_joint_names = []
        self.sway_joint_names = [str(name) for name in raw_sway_joint_names if str(name)]
        if not self.sway_joint_names and self.sway_joint_name:
            self.sway_joint_names = [self.sway_joint_name]
        self.sway_brake_gain = float(self.get_parameter("sway_brake_gain").value)
        self.sway_soft_limit = float(self.get_parameter("sway_soft_limit").value)
        self.sway_hard_limit = float(self.get_parameter("sway_hard_limit").value)
        self.slew_zero_threshold = float(self.get_parameter("slew_zero_threshold").value)
        self.boom_zero_threshold = float(self.get_parameter("boom_zero_threshold").value)
        self.winch_zero_threshold = float(self.get_parameter("winch_zero_threshold").value)

        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.wait_for_joint_states = bool(self.get_parameter("wait_for_joint_states").value)
        self.startup_target_slew = float(self.get_parameter("startup_target_slew").value)
        self.startup_target_boom = float(self.get_parameter("startup_target_boom").value)
        self.startup_target_winch = float(self.get_parameter("startup_target_winch").value)

        self.cmd_in_topic = str(self.get_parameter("cmd_in_topic").value)
        self.priority_cmd_in_topic = str(self.get_parameter("priority_cmd_in_topic").value)
        self.disturbance_cmd_topic = str(self.get_parameter("disturbance_cmd_topic").value)
        self.priority_disturbance_cmd_topic = str(self.get_parameter("priority_disturbance_cmd_topic").value)
        self.target_in_topic = str(self.get_parameter("target_in_topic").value)
        self.priority_target_in_topic = str(self.get_parameter("priority_target_in_topic").value)
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.cmd_out_topic = str(self.get_parameter("cmd_out_topic").value)
        self.lower_limits = [float(v) for v in self.get_parameter("lower_limits").value]
        self.upper_limits = [float(v) for v in self.get_parameter("upper_limits").value]
        self.max_target_error = [float(v) for v in self.get_parameter("max_target_error").value]
        self.mimic_rules = []
        self.mimic_dst_indices = set()
        raw_mimic_rules = []
        try:
            mimic_rules_param = self.get_parameter("mimic_rules")
            if mimic_rules_param.type_ != Parameter.Type.NOT_SET:
                raw_mimic_rules = mimic_rules_param.value
        except ParameterUninitializedException:
            raw_mimic_rules = []
        for raw_rule in raw_mimic_rules:
            try:
                dst_name, src_name, mult_raw, off_raw = str(raw_rule).split(":")
                dst_idx = self.joint_names.index(dst_name)
                src_idx = self.joint_names.index(src_name)
                mult = float(mult_raw)
                off = float(off_raw)
                self.mimic_rules.append((dst_idx, src_idx, mult, off))
                self.mimic_dst_indices.add(dst_idx)
            except Exception:
                self.get_logger().warn(
                    f"Ignoring invalid mimic rule '{raw_rule}'. "
                    "Expected format: dst_joint:src_joint:multiplier:offset"
                )
        self.validate_configuration()

        # ROS Interfaces
        command_qos = QoSProfile(depth=1)
        command_qos.reliability = ReliabilityPolicy.RELIABLE

        self.pub = self.create_publisher(
            Float64MultiArray,
            self.cmd_out_topic,
            command_qos
        )

        self.sub = self.create_subscription(
            CraneCommand,
            self.cmd_in_topic,
            self.on_cmd,
            command_qos
        )
        self.priority_sub = None
        if self.priority_cmd_in_topic:
            self.priority_sub = self.create_subscription(
                CraneCommand,
                self.priority_cmd_in_topic,
                self.on_priority_cmd,
                command_qos
            )
        self.disturbance_sub = self.create_subscription(
            CraneCommand,
            self.disturbance_cmd_topic,
            self.on_disturbance_cmd,
            command_qos
        )
        self.priority_disturbance_sub = None
        if self.priority_disturbance_cmd_topic:
            self.priority_disturbance_sub = self.create_subscription(
                CraneCommand,
                self.priority_disturbance_cmd_topic,
                self.on_priority_disturbance_cmd,
                command_qos
            )
        self.target_sub = self.create_subscription(
            CraneTarget,
            self.target_in_topic,
            self.on_target,
            command_qos
        )
        self.priority_target_sub = None
        if self.priority_target_in_topic:
            self.priority_target_sub = self.create_subscription(
                CraneTarget,
                self.priority_target_in_topic,
                self.on_priority_target,
                command_qos
            )
        self.js_sub = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.on_joint_states,
            qos_profile_sensor_data
        )

        self.last_cmd = None
        self.last_priority_cmd = None
        self.last_disturbance_cmd = None
        self.last_priority_disturbance_cmd = None
        self.last_target = None
        self.last_priority_target = None
        self.last_cmd_time = self.get_clock().now()
        self.last_priority_cmd_time = self.get_clock().now()
        self.last_disturbance_cmd_time = self.get_clock().now()
        self.last_priority_disturbance_cmd_time = self.get_clock().now()
        self.last_target_time = self.get_clock().now()
        self.last_priority_target_time = self.get_clock().now()
        self.target_positions = None
        self.current_positions = {}
        self.current_velocities = {}
        self.have_joint_states = False
        self.last_publish_time = self.get_clock().now()
        self.prev_slew_vel = 0.0
        self.prev_winch_vel = 0.0
        self._init_warned = False
        self._multi_pub_warned = False

        hz = float(self.get_parameter("publish_hz").value)
        period = 1.0 / max(hz, 1.0)
        self.timer = self.create_timer(period, self.publish_position_commands)
        self.create_timer(2.0, self.check_topic_diagnostics)

        self.get_logger().info(
            f"Bridge ready: {self.cmd_in_topic} -> {self.cmd_out_topic} "
            f"({len(self.joint_names)} joints)"
        )

    def validate_configuration(self):
        joint_count = len(self.joint_names)
        if joint_count == 0:
            raise ValueError("joint_names must not be empty")

        duplicates = sorted({name for name in self.joint_names if self.joint_names.count(name) > 1})
        if duplicates:
            raise ValueError(f"joint_names contains duplicates: {duplicates}")

        configured_indices = [
            ("idx_slew", self.idx_slew),
            ("idx_boom", self.idx_boom),
            ("idx_winch", self.idx_winch),
            ("idx_sway", self.idx_sway),
        ]
        active_indices = []
        for label, idx in configured_indices:
            if idx < 0:
                continue
            if idx >= joint_count:
                raise ValueError(f"{label}={idx} is out of range for {joint_count} joints")
            active_indices.append((label, idx))

        seen_indices = {}
        for label, idx in active_indices:
            other = seen_indices.get(idx)
            if other is not None:
                raise ValueError(f"{label} and {other} both target joint_names[{idx}]")
            seen_indices[idx] = label

        if len(self.lower_limits) != joint_count:
            raise ValueError(
                f"lower_limits length {len(self.lower_limits)} does not match "
                f"joint_names length {joint_count}"
            )
        if len(self.upper_limits) != joint_count:
            raise ValueError(
                f"upper_limits length {len(self.upper_limits)} does not match "
                f"joint_names length {joint_count}"
            )
        if len(self.max_target_error) != joint_count:
            raise ValueError(
                f"max_target_error length {len(self.max_target_error)} does not match "
                f"joint_names length {joint_count}"
            )

        for i, joint_name in enumerate(self.joint_names):
            if self.lower_limits[i] > self.upper_limits[i]:
                raise ValueError(
                    f"Invalid limits for {joint_name}: "
                    f"lower {self.lower_limits[i]} > upper {self.upper_limits[i]}"
                )

    def check_topic_diagnostics(self):
        publishers = self.get_publishers_info_by_topic(self.cmd_out_topic)
        current_name = self.get_fully_qualified_name()
        other_publishers = []
        for info in publishers:
            name = f"{info.node_namespace}/{info.node_name}".replace("//", "/")
            if name == current_name:
                continue
            other_publishers.append(name)

        if other_publishers and not self._multi_pub_warned:
            self.get_logger().warn(
                f"Detected {len(other_publishers) + 1} publishers on {self.cmd_out_topic}: "
                f"{[current_name] + other_publishers}. "
                "This can cause command-size mismatches if an old bridge process is still alive."
            )
            self._multi_pub_warned = True
        elif not other_publishers:
            self._multi_pub_warned = False

    # Callback
    def on_cmd(self, msg: CraneCommand):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def on_priority_cmd(self, msg: CraneCommand):
        self.last_priority_cmd = msg
        self.last_priority_cmd_time = self.get_clock().now()

    def on_disturbance_cmd(self, msg: CraneCommand):
        self.last_disturbance_cmd = msg
        self.last_disturbance_cmd_time = self.get_clock().now()

    def on_priority_disturbance_cmd(self, msg: CraneCommand):
        self.last_priority_disturbance_cmd = msg
        self.last_priority_disturbance_cmd_time = self.get_clock().now()

    def on_target(self, msg: CraneTarget):
        self.last_target = msg
        self.last_target_time = self.get_clock().now()

    def on_priority_target(self, msg: CraneTarget):
        self.last_priority_target = msg
        self.last_priority_target_time = self.get_clock().now()

    def on_joint_states(self, msg: JointState):
        if msg.name:
            self.have_joint_states = True
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_positions[name] = float(msg.position[i])
            if i < len(msg.velocity):
                self.current_velocities[name] = float(msg.velocity[i])

    def get_joint_position(self, joint_name: str):
        if joint_name in self.current_positions:
            return self.current_positions[joint_name]
        for name, pos in self.current_positions.items():
            if name.endswith(f"/{joint_name}") or name.endswith(f"::{joint_name}"):
                return pos
        return None

    def get_joint_velocity(self, joint_name: str):
        if joint_name in self.current_velocities:
            return self.current_velocities[joint_name]
        for name, vel in self.current_velocities.items():
            if name.endswith(f"/{joint_name}") or name.endswith(f"::{joint_name}"):
                return vel
        return None

    def get_sway_state(self):
        sway_positions = []
        sway_velocities = []
        for joint_name in self.sway_joint_names:
            pos = self.get_joint_position(joint_name)
            vel = self.get_joint_velocity(joint_name)
            if pos is not None:
                sway_positions.append(abs(pos))
            if vel is not None:
                sway_velocities.append(abs(vel))
        sway_pos = max(sway_positions) if sway_positions else None
        sway_vel = max(sway_velocities) if sway_velocities else None
        return sway_pos, sway_vel

    def normalized_joint_position(self, idx: int) -> float:
        if idx < 0 or idx >= len(self.joint_names):
            return 0.0
        span = self.upper_limits[idx] - self.lower_limits[idx]
        if span <= 1e-9:
            return 0.0
        pos = self.target_positions[idx]
        return clamp((pos - self.lower_limits[idx]) / span, 0.0, 1.0)

    def apply_limit_guard(self, idx: int, cmd: float, margin_ratio: float = 0.08) -> float:
        if idx < 0 or idx >= len(self.joint_names):
            return cmd
        joint_name = self.joint_names[idx]
        pos = self.get_joint_position(joint_name)
        if pos is None:
            return cmd

        lower = self.lower_limits[idx]
        upper = self.upper_limits[idx]
        span = upper - lower
        if span <= 1e-9:
            return 0.0

        margin = max(span * margin_ratio, 1e-3)

        if cmd < 0.0:
            dist = pos - lower
            if dist <= 0.0:
                self.target_positions[idx] = pos
                return 0.0
            if dist < margin:
                return cmd * clamp(dist / margin, 0.0, 1.0)

        if cmd > 0.0:
            dist = upper - pos
            if dist <= 0.0:
                self.target_positions[idx] = pos
                return 0.0
            if dist < margin:
                return cmd * clamp(dist / margin, 0.0, 1.0)

        return cmd

    def apply_target_error_guard(self, idx: int) -> None:
        if idx < 0 or idx >= len(self.joint_names):
            return
        max_error = self.max_target_error[idx]
        if max_error < 0.0:
            return
        joint_name = self.joint_names[idx]
        pos = self.get_joint_position(joint_name)
        if pos is None:
            return
        lo = max(self.lower_limits[idx], pos - max_error)
        hi = min(self.upper_limits[idx], pos + max_error)
        self.target_positions[idx] = clamp(self.target_positions[idx], lo, hi)

    # Publish loop
    def publish_position_commands(self):
        if self.wait_for_joint_states and not self.have_joint_states:
            return

        if self.target_positions is None:
            initial_positions = []
            for joint_name in self.joint_names:
                pos = self.get_joint_position(joint_name)
                if pos is None:
                    pos = 0.0
                initial_positions.append(pos)

            required_indices = [
                idx for idx in (self.idx_slew, self.idx_boom, self.idx_winch, self.idx_sway)
                if 0 <= idx < len(self.joint_names)
            ]
            missing_required = [
                self.joint_names[idx]
                for idx in required_indices
                if self.get_joint_position(self.joint_names[idx]) is None
            ]
            if missing_required and self.wait_for_joint_states:
                if not self._init_warned:
                    seen = ", ".join(sorted(self.current_positions.keys()))
                    self.get_logger().warn(
                        "Waiting for required joints in /joint_states before initializing targets. "
                        f"Missing required: {missing_required}. Seen: [{seen}]"
                    )
                    self._init_warned = True
                return

            self.target_positions = initial_positions
            if 0 <= self.idx_slew < len(self.target_positions):
                self.target_positions[self.idx_slew] = clamp(
                    self.startup_target_slew,
                    self.lower_limits[self.idx_slew],
                    self.upper_limits[self.idx_slew],
                )
            if 0 <= self.idx_boom < len(self.target_positions):
                self.target_positions[self.idx_boom] = clamp(
                    self.startup_target_boom,
                    self.lower_limits[self.idx_boom],
                    self.upper_limits[self.idx_boom],
                )
            if 0 <= self.idx_winch < len(self.target_positions):
                startup_winch = self.startup_target_winch
                if startup_winch < 0.0:
                    startup_winch = self.lower_limits[self.idx_winch]
                self.target_positions[self.idx_winch] = clamp(
                    startup_winch,
                    self.lower_limits[self.idx_winch],
                    self.upper_limits[self.idx_winch],
                )
            self.apply_mimic_rules()
            self._init_warned = False
        else:
            # Keep uncommanded joints synced to measured state when available.
            for i, joint_name in enumerate(self.joint_names):
                if i in (self.idx_slew, self.idx_boom, self.idx_winch, self.idx_sway) or i in self.mimic_dst_indices:
                    continue
                pos = self.get_joint_position(joint_name)
                if pos is not None:
                    self.target_positions[i] = pos

        now = self.get_clock().now()
        age = (now - self.last_cmd_time).nanoseconds / 1e9
        priority_age = (now - self.last_priority_cmd_time).nanoseconds / 1e9
        target_age = (now - self.last_target_time).nanoseconds / 1e9
        priority_target_age = (now - self.last_priority_target_time).nanoseconds / 1e9
        disturbance_age = (now - self.last_disturbance_cmd_time).nanoseconds / 1e9
        priority_disturbance_age = (now - self.last_priority_disturbance_cmd_time).nanoseconds / 1e9
        dt = (now - self.last_publish_time).nanoseconds / 1e9
        dt = max(0.0, min(dt, 0.1))
        self.last_publish_time = now

        priority_target_fresh = (
            self.last_priority_target is not None
            and priority_target_age <= self.timeout_sec
        )
        if priority_target_fresh:
            target = self.last_priority_target
            target_active = bool(target.enable)
        else:
            target = self.last_target
            target_active = (
                target is not None
                and target_age <= self.timeout_sec
                and bool(target.enable)
            )

        if target_active:
            if 0 <= self.idx_slew < len(self.target_positions):
                self.target_positions[self.idx_slew] = clamp(
                    float(target.slew_position),
                    self.lower_limits[self.idx_slew],
                    self.upper_limits[self.idx_slew],
                )
            if 0 <= self.idx_boom < len(self.target_positions):
                self.target_positions[self.idx_boom] = clamp(
                    float(target.trolley_position),
                    self.lower_limits[self.idx_boom],
                    self.upper_limits[self.idx_boom],
                )
            if 0 <= self.idx_winch < len(self.target_positions):
                self.target_positions[self.idx_winch] = clamp(
                    float(target.winch_length),
                    self.lower_limits[self.idx_winch],
                    self.upper_limits[self.idx_winch],
                )
            self.prev_slew_vel = 0.0
            self.prev_winch_vel = 0.0
            self.apply_mimic_rules()
            self.publish_array(self.target_positions)
            return

        priority_cmd_fresh = (
            self.last_priority_cmd is not None
            and priority_age <= self.timeout_sec
        )
        cmd = self.last_priority_cmd if priority_cmd_fresh else self.last_cmd
        cmd_age = priority_age if priority_cmd_fresh else age

        if cmd is None or cmd_age > self.timeout_sec:
            self.prev_slew_vel = 0.0
            self.prev_winch_vel = 0.0
            self.apply_mimic_rules()
            self.publish_array(self.target_positions)
            return

        if not cmd.enable:
            self.prev_slew_vel = 0.0
            self.prev_winch_vel = 0.0
            self.apply_mimic_rules()
            self.publish_array(self.target_positions)
            return

        velocities = [0.0] * len(self.joint_names)
        priority_disturbance_fresh = (
            self.last_priority_disturbance_cmd is not None
            and priority_disturbance_age <= self.timeout_sec
        )
        disturbance = None
        if priority_disturbance_fresh:
            if bool(self.last_priority_disturbance_cmd.enable):
                disturbance = self.last_priority_disturbance_cmd
        elif (
            self.last_disturbance_cmd is not None
            and disturbance_age <= self.timeout_sec
            and bool(self.last_disturbance_cmd.enable)
        ):
            disturbance = self.last_disturbance_cmd

        slew_cmd = float(cmd.slew_rate)
        boom_cmd = float(cmd.boom_rate)
        winch_cmd = float(cmd.winch_rate)
        sway_cmd = float(getattr(cmd, "sway_rate", 0.0))
        if disturbance is not None:
            slew_cmd += float(disturbance.slew_rate)
            boom_cmd += float(disturbance.boom_rate)
            winch_cmd += float(disturbance.winch_rate)
            sway_cmd += float(getattr(disturbance, "sway_rate", 0.0))
        slew_cmd = clamp(slew_cmd, -1.0, 1.0)
        boom_cmd = clamp(boom_cmd, -1.0, 1.0)
        winch_cmd = clamp(winch_cmd, -1.0, 1.0)
        sway_cmd = clamp(sway_cmd, -1.0, 1.0)

        if abs(slew_cmd) < self.slew_zero_threshold:
            slew_cmd = 0.0
        if abs(boom_cmd) < self.boom_zero_threshold:
            boom_cmd = 0.0
        if abs(winch_cmd) < self.winch_zero_threshold:
            winch_cmd = 0.0

        if 0 <= self.idx_slew < len(self.joint_names):
            self.apply_target_error_guard(self.idx_slew)
        if 0 <= self.idx_boom < len(self.joint_names):
            self.apply_target_error_guard(self.idx_boom)
            boom_cmd = self.apply_limit_guard(self.idx_boom, boom_cmd)
        if 0 <= self.idx_winch < len(self.joint_names):
            self.apply_target_error_guard(self.idx_winch)
            winch_cmd = self.apply_limit_guard(self.idx_winch, winch_cmd)
        if 0 <= self.idx_sway < len(self.joint_names):
            self.apply_target_error_guard(self.idx_sway)
            sway_cmd = self.apply_limit_guard(self.idx_sway, sway_cmd, margin_ratio=0.20)

        if boom_cmd == 0.0 and 0 <= self.idx_boom < len(self.joint_names):
            measured_boom = self.get_joint_position(self.joint_names[self.idx_boom])
            if measured_boom is not None:
                self.target_positions[self.idx_boom] = measured_boom
        if sway_cmd == 0.0 and 0 <= self.idx_sway < len(self.joint_names):
            measured_sway = self.get_joint_position(self.joint_names[self.idx_sway])
            if measured_sway is not None:
                self.target_positions[self.idx_sway] = measured_sway

        slew_scale = 1.0
        if self.reach_aware_slew and self.target_positions is not None:
            trolley_n = self.normalized_joint_position(self.idx_boom)
            winch_n = self.normalized_joint_position(self.idx_winch)
            reach_n = clamp(
                self.reach_weight_trolley * trolley_n + self.reach_weight_winch * winch_n,
                0.0,
                1.0,
            )
            min_scale = clamp(self.min_slew_scale_at_max_reach, 0.05, 1.0)
            slew_scale = 1.0 - (1.0 - min_scale) * reach_n

        slew_vel_target = slew_cmd * self.max_slew * slew_scale
        max_delta = self.max_slew_accel * max(0.2, slew_scale) * dt
        slew_vel = clamp(
            slew_vel_target,
            self.prev_slew_vel - max_delta,
            self.prev_slew_vel + max_delta,
        )
        boom_vel = boom_cmd * self.max_boom
        winch_vel_target = winch_cmd * self.max_winch
        max_winch_delta = self.max_winch_accel * dt
        winch_vel = clamp(
            winch_vel_target,
            self.prev_winch_vel - max_winch_delta,
            self.prev_winch_vel + max_winch_delta,
        )

        if 0 <= self.idx_slew < len(velocities):
            velocities[self.idx_slew] = slew_vel

        if 0 <= self.idx_boom < len(velocities):
            velocities[self.idx_boom] = boom_vel

        if 0 <= self.idx_winch < len(velocities):
            velocities[self.idx_winch] = winch_vel
        if 0 <= self.idx_sway < len(velocities):
            velocities[self.idx_sway] = sway_cmd * self.max_boom

        for i, vel in enumerate(velocities):
            next_pos = self.target_positions[i] + vel * dt
            self.target_positions[i] = clamp(next_pos, self.lower_limits[i], self.upper_limits[i])

        self.prev_slew_vel = slew_vel
        self.prev_winch_vel = winch_vel
        self.apply_mimic_rules()

        self.publish_array(self.target_positions)

    def apply_mimic_rules(self):
        for dst_idx, src_idx, mult, off in self.mimic_rules:
            mimic_pos = self.target_positions[src_idx] * mult + off
            self.target_positions[dst_idx] = clamp(
                mimic_pos,
                self.lower_limits[dst_idx],
                self.upper_limits[dst_idx],
            )

    def publish_array(self, v):
        if len(v) != len(self.joint_names):
            self.get_logger().error(
                f"Refusing to publish {len(v)} commands to {self.cmd_out_topic}; "
                f"expected {len(self.joint_names)} for joints {self.joint_names}"
            )
            return
        msg = Float64MultiArray()
        msg.data = v
        self.pub.publish(msg)

def main():
    rclpy.init()
    try:
        node = CraneController()
    except Exception as exc:
        rclpy.logging.get_logger("crane_cmd_bridge").error(
            f"Failed to initialize crane controller bridge: {exc}"
        )
        try:
            rclpy.shutdown()
        except Exception:
            pass
        raise
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
