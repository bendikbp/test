import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from palfinger_msgs.msg import CraneCommand, CraneTarget, SnapState

TRAINING_PROFILES = {
    "off",
    "gusty_wind",
    "sway_training",
    "recovery",
    "pendulum_kick",
    "sudden_yaw_reversal",
    "hoist_snag_release",
    "custom",
}

SCENARIO_SHORTCUTS = {
    "1": "gusty_wind",
    "2": "sway_training",
    "3": "recovery",
    "4": "pendulum_kick",
    "5": "hoist_snag_release",
    "0": "off",
}

SCENARIO_DESCRIPTIONS = {
    "off": "No disturbance.",
    "gusty_wind": "Wind gusts that excite suspended payload sway only while attached.",
    "sway_training": "Continuous boom and winch sway to practice damping.",
    "recovery": "Alternating axis reversals for coordinated recovery.",
    "pendulum_kick": "Short periodic kicks that re-excite hook swing after calm intervals.",
    "sudden_yaw_reversal": "Abrupt opposing slew pulses while boom is biased off-center.",
    "hoist_snag_release": "Winch stalls briefly, then releases with a sharp hoist correction.",
    "custom": "Directly specified disturbance rates.",
}

def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))

class TeleopOperatorTraining(Node):
    def __init__(self):
        super().__init__("teleop_operator_training")

        command_qos = QoSProfile(depth=1)
        command_qos.reliability = ReliabilityPolicy.RELIABLE

        self.declare_parameter("mode", "operator_training")
        self.declare_parameter("cmd_topic", "/crane/cmd")
        self.declare_parameter("target_topic", "/crane/target")
        self.declare_parameter("disturbance_topic", "/crane/disturbance_cmd")
        self.declare_parameter("external_command_topic", "/crane/training_command")
        self.declare_parameter("snap_state_topic", "/snap/state")
        self.declare_parameter("training_profile", "off")
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("gust_amplitude", 0.30)
        self.declare_parameter("gust_frequency_hz", 0.10)
        self.declare_parameter("sway_boom_amplitude", 0.28)
        self.declare_parameter("sway_winch_amplitude", 0.12)
        self.declare_parameter("recovery_slew_amplitude", 0.12)
        self.declare_parameter("recovery_boom_amplitude", 0.08)
        self.declare_parameter("recovery_winch_amplitude", 0.06)
        self.declare_parameter("recovery_slew_period", 6.0)
        self.declare_parameter("recovery_boom_period", 8.0)
        self.declare_parameter("recovery_winch_period", 10.0)

        self.mode = str(self.get_parameter("mode").value).strip().lower()
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.target_topic = str(self.get_parameter("target_topic").value)
        self.disturbance_topic = str(self.get_parameter("disturbance_topic").value)
        self.external_command_topic = str(self.get_parameter("external_command_topic").value)
        self.snap_state_topic = str(self.get_parameter("snap_state_topic").value)

        self.cmd_pub = self.create_publisher(CraneCommand, self.cmd_topic, command_qos)
        self.target_pub = self.create_publisher(CraneTarget, self.target_topic, command_qos)
        self.disturbance_pub = self.create_publisher(CraneCommand, self.disturbance_topic, command_qos)
        self.external_command_sub = self.create_subscription(
            String,
            self.external_command_topic,
            self._on_external_command,
            command_qos,
        )
        self.snap_state_sub = self.create_subscription(
            SnapState,
            self.snap_state_topic,
            self._on_snap_state,
            command_qos,
        )

        self.rate_state = CraneCommand()
        self.rate_state.enable = True
        self.target_state = CraneTarget()
        self.target_state.enable = True
        requested_training_profile = str(self.get_parameter("training_profile").value).strip().lower()
        if requested_training_profile not in TRAINING_PROFILES:
            self.get_logger().warn(
                f"Unknown training_profile '{requested_training_profile}', defaulting to 'off'"
            )
            requested_training_profile = "off"
        self.disturbance_profile = requested_training_profile
        self.disturbance_enable = True
        self.custom_disturbance = CraneCommand()
        self.custom_disturbance.enable = True
        self.scenario_started_ns = self.get_clock().now().nanoseconds
        self.payload_attached = False
        self.attached_target_id = ""

        hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.timer = self.create_timer(1.0 / hz, self.on_timer)

        self._running = True
        self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self._input_thread.start()

        self._show_welcome()

    def on_timer(self):
        if self.mode == "target":
            self.target_pub.publish(self.target_state)
            return
        if self.mode == "rate":
            self.cmd_pub.publish(self.rate_state)
            return
        self.disturbance_pub.publish(self._build_disturbance_msg())

    def _input_loop(self):
        while self._running and rclpy.ok():
            try:
                raw = input("training> ").strip()
            except EOFError:
                self._running = False
                break
            except Exception as exc:
                self.get_logger().error(f"Terminal input failed: {exc}")
                continue

            if raw:
                self._handle_command(raw)

    def _on_external_command(self, msg: String):
        raw = msg.data.strip()
        if not raw:
            self.get_logger().warn(
                "Ignoring empty message on external command topic "
                f"{self.external_command_topic}"
            )
            return

        self.get_logger().info(f"External command received: {raw}")
        self._handle_command(raw)

    def _print_block(self, lines: list[str]):
        print("")
        for line in lines:
            print(line)
        print("", flush=True)

    def _show_welcome(self):
        self._print_block([
            "Palfinger Operator Training",
            "---------------------------",
            f"Mode: {self.mode}",
            f"Command topic: {self.external_command_topic}",
            "",
            "Quick Start",
            "0 Off",
            "1 Gusty Wind",
            "2 Sway Training",
            "3 Recovery",
            "4 Pendulum Kick",
            "5 Hoist Snag Release",
            "",
            "Type a number to trigger a scenario.",
            "Type 'status' for current state, 'menu' for help, 'quit' to exit.",
        ])

    def _log_menu(self):
        self._print_block([
            "Menu",
            "----",
            "Modes: operator_training | rate | target",
            "Shortcuts: 0=off, 1=gusty_wind, 2=sway_training, 3=recovery, 4=pendulum_kick, 5=hoist_snag_release",
            "",
            "Commands",
            "menu",
            "status",
            "mode <operator_training|rate|target>",
            "training <off|gusty_wind|sway_training|recovery|pendulum_kick|sudden_yaw_reversal|hoist_snag_release|custom>",
            "custom <slew> <boom> <winch> [enable]",
            "gust <amplitude> <frequency_hz>",
            "sway <boom_amp> <winch_amp>",
            "recovery <slew_amp> <boom_amp> <winch_amp> [slew_period] [boom_period] [winch_period]",
            "rate <slew> <boom> <winch> [enable]",
            "target <slew_pos> <trolley_pos> <winch_len> [enable]",
            "enable | disable | stop | quit",
        ])
        self._log_scenarios()

    def _log_scenarios(self):
        lines = ["Scenarios", "---------"]
        for shortcut, name in sorted(
            SCENARIO_SHORTCUTS.items(),
            key=lambda item: int(item[0]),
        ):
            lines.append(f"{shortcut} {name}: {SCENARIO_DESCRIPTIONS[name]}")
        lines.append("extra sudden_yaw_reversal: Abrupt opposing slew pulses while boom is biased off-center.")
        lines.append("extra custom: Directly specified disturbance rates.")
        self._print_block(lines)

    def _show_status(self):
        disturbance = self._build_disturbance_msg()
        attached_text = self.attached_target_id if self.attached_target_id else "none"
        self._print_block([
            "Status",
            "------",
            f"Mode: {self.mode}",
            f"Scenario: {self.disturbance_profile}",
            f"Payload attached: {'yes' if self.payload_attached else 'no'}",
            f"Attached target: {attached_text}",
            f"Disturbance enabled: {'yes' if disturbance.enable else 'no'}",
            f"Disturbance rates: slew={disturbance.slew_rate:.3f} boom={disturbance.boom_rate:.3f} winch={disturbance.winch_rate:.3f} sway={disturbance.sway_rate:.3f}",
        ])

    def _show_action(self, title: str, detail: str = ""):
        lines = [title]
        if detail:
            lines.append(detail)
        self._print_block(lines)

    def _parse_enable(self, tokens: list[str], default: bool) -> bool:
        if not tokens:
            return default
        value = tokens[0].strip().lower()
        return value not in {"0", "false", "off", "disable", "disabled", "no"}

    def _on_snap_state(self, msg: SnapState):
        self.payload_attached = bool(msg.attached)
        self.attached_target_id = str(msg.attached_target_id)

    def _set_training_profile(self, profile: str):
        self.mode = "operator_training"
        self.disturbance_profile = profile
        self.scenario_started_ns = self.get_clock().now().nanoseconds
        detail = SCENARIO_DESCRIPTIONS.get(profile, "")
        if profile == "gusty_wind" and not self.payload_attached:
            detail = "Waiting for attached payload before wind becomes active."
        self._show_action(f"Scenario: {self.disturbance_profile}", detail)

    def _build_disturbance_msg(self) -> CraneCommand:
        msg = CraneCommand()
        profile = self.disturbance_profile
        msg.enable = self.disturbance_enable and profile != "off"

        if not msg.enable:
            return msg

        t = (self.get_clock().now().nanoseconds - self.scenario_started_ns) / 1e9

        if profile == "gusty_wind":
            if not self.payload_attached:
                msg.enable = False
                return msg
            amp = float(self.get_parameter("gust_amplitude").value)
            freq = float(self.get_parameter("gust_frequency_hz").value)
            gust = amp * math.sin(2.0 * math.pi * freq * t)
            # Excite the suspended payload through both sway and small winch/slew
            # disturbances so the effect remains visible with the current bridge
            # thresholds and joint limits.
            msg.sway_rate = gust
            msg.slew_rate = 0.18 * gust
            msg.boom_rate = 0.0
            msg.winch_rate = -0.45 * gust
        elif profile == "sway_training":
            msg.slew_rate = 0.10 * math.sin(0.25 * t + 0.4)
            msg.boom_rate = float(self.get_parameter("sway_boom_amplitude").value) * math.sin(0.35 * t)
            msg.winch_rate = float(self.get_parameter("sway_winch_amplitude").value) * math.sin(0.55 * t + 1.2)
            msg.sway_rate = 0.22 * math.sin(0.75 * t + 0.3)
        elif profile == "recovery":
            slew_amp = abs(float(self.get_parameter("recovery_slew_amplitude").value))
            boom_amp = abs(float(self.get_parameter("recovery_boom_amplitude").value))
            winch_amp = abs(float(self.get_parameter("recovery_winch_amplitude").value))
            slew_period = max(float(self.get_parameter("recovery_slew_period").value), 0.1)
            boom_period = max(float(self.get_parameter("recovery_boom_period").value), 0.1)
            winch_period = max(float(self.get_parameter("recovery_winch_period").value), 0.1)
            msg.slew_rate = slew_amp if int(t / slew_period) % 2 == 0 else -slew_amp
            msg.boom_rate = -boom_amp if int(t / boom_period) % 2 == 0 else boom_amp
            msg.winch_rate = winch_amp if int(t / winch_period) % 2 == 0 else -winch_amp
        elif profile == "pendulum_kick":
            cycle_t = math.fmod(t, 7.5)
            if cycle_t < 1.2:
                msg.slew_rate = 0.32 * math.sin(math.pi * cycle_t / 1.2)
                msg.boom_rate = -0.18 * math.sin(math.pi * cycle_t / 1.2)
                msg.winch_rate = 0.10 * math.sin(2.0 * math.pi * cycle_t / 1.2)
            else:
                msg.slew_rate = 0.04 * math.sin(0.7 * t)
                msg.boom_rate = 0.0
                msg.winch_rate = 0.0
        elif profile == "sudden_yaw_reversal":
            cycle_t = math.fmod(t, 9.0)
            msg.boom_rate = 0.08 if cycle_t < 4.5 else -0.08
            msg.winch_rate = 0.03 * math.sin(0.9 * t)
            if cycle_t < 1.0:
                msg.slew_rate = 0.42
            elif cycle_t < 1.7:
                msg.slew_rate = -0.58
            elif cycle_t < 2.6:
                msg.slew_rate = 0.26
            else:
                msg.slew_rate = -0.05 * math.sin(0.5 * t)
        elif profile == "hoist_snag_release":
            cycle_t = math.fmod(t, 10.0)
            msg.slew_rate = 0.06 * math.sin(0.6 * t)
            if cycle_t < 2.5:
                msg.boom_rate = 0.05
                msg.winch_rate = -0.12
            elif cycle_t < 4.0:
                msg.boom_rate = -0.02
                msg.winch_rate = 0.0
            elif cycle_t < 5.2:
                msg.boom_rate = -0.16
                msg.winch_rate = 0.42
            else:
                msg.boom_rate = 0.03 * math.sin(0.8 * t)
                msg.winch_rate = -0.05 * math.sin(0.4 * t)
        elif profile == "custom":
            msg.slew_rate = clamp(float(self.custom_disturbance.slew_rate), -1.0, 1.0)
            msg.boom_rate = clamp(float(self.custom_disturbance.boom_rate), -1.0, 1.0)
            msg.winch_rate = clamp(float(self.custom_disturbance.winch_rate), -1.0, 1.0)
            msg.enable = bool(self.custom_disturbance.enable) and self.disturbance_enable
        else:
            msg.enable = False

        return msg

    def _handle_command(self, raw: str):
        tokens = raw.split()
        cmd = tokens[0].lower()

        if cmd in {"menu", "help"}:
            self._log_menu()
            return

        if cmd == "scenarios":
            self._log_scenarios()
            return

        if cmd in SCENARIO_SHORTCUTS and len(tokens) == 1:
            self._set_training_profile(SCENARIO_SHORTCUTS[cmd])
            return

        if cmd == "mode":
            if len(tokens) != 2 or tokens[1] not in {"operator_training", "rate", "target"}:
                self.get_logger().warn("Usage: mode <operator_training|rate|target>")
                return
            self.mode = tokens[1]
            self._show_action(f"Mode changed to {self.mode}")
            return

        if cmd == "training":
            if len(tokens) != 2 or tokens[1] not in TRAINING_PROFILES:
                self.get_logger().warn(
                    "Usage: training <off|gusty_wind|sway_training|recovery|"
                    "pendulum_kick|sudden_yaw_reversal|hoist_snag_release|custom>"
                )
                return
            self._set_training_profile(tokens[1])
            return

        if cmd == "custom":
            if len(tokens) < 4:
                self.get_logger().warn("Usage: custom <slew> <boom> <winch> [enable]")
                return
            try:
                self.custom_disturbance.slew_rate = clamp(float(tokens[1]), -1.0, 1.0)
                self.custom_disturbance.boom_rate = clamp(float(tokens[2]), -1.0, 1.0)
                self.custom_disturbance.winch_rate = clamp(float(tokens[3]), -1.0, 1.0)
                self.custom_disturbance.sway_rate = 0.0
                self.custom_disturbance.enable = self._parse_enable(tokens[4:], self.custom_disturbance.enable)
                self._set_training_profile("custom")
            except ValueError:
                self.get_logger().warn("Custom disturbance values must be numeric.")
            return

        if cmd == "gust":
            if len(tokens) != 3:
                self.get_logger().warn("Usage: gust <amplitude> <frequency_hz>")
                return
            try:
                amp = clamp(float(tokens[1]), -1.0, 1.0)
                freq = max(float(tokens[2]), 0.0)
                self.set_parameters([
                    rclpy.parameter.Parameter("gust_amplitude", value=amp),
                    rclpy.parameter.Parameter("gust_frequency_hz", value=freq),
                ])
                self._set_training_profile("gusty_wind")
                self._show_action("Gust parameters updated", f"amplitude={amp:.3f} frequency={freq:.3f} Hz")
            except ValueError:
                self.get_logger().warn("Gust values must be numeric.")
            return

        if cmd == "sway":
            if len(tokens) != 3:
                self.get_logger().warn("Usage: sway <boom_amp> <winch_amp>")
                return
            try:
                boom_amp = clamp(float(tokens[1]), -1.0, 1.0)
                winch_amp = clamp(float(tokens[2]), -1.0, 1.0)
                self.set_parameters([
                    rclpy.parameter.Parameter("sway_boom_amplitude", value=boom_amp),
                    rclpy.parameter.Parameter("sway_winch_amplitude", value=winch_amp),
                ])
                self._set_training_profile("sway_training")
                self._show_action("Sway parameters updated", f"boom={boom_amp:.3f} winch={winch_amp:.3f}")
            except ValueError:
                self.get_logger().warn("Sway values must be numeric.")
            return

        if cmd == "recovery":
            if len(tokens) not in {4, 7}:
                self.get_logger().warn(
                    "Usage: recovery <slew_amp> <boom_amp> <winch_amp> "
                    "[slew_period] [boom_period] [winch_period]"
                )
                return
            try:
                slew_amp = clamp(abs(float(tokens[1])), 0.0, 1.0)
                boom_amp = clamp(abs(float(tokens[2])), 0.0, 1.0)
                winch_amp = clamp(abs(float(tokens[3])), 0.0, 1.0)
                params = [
                    rclpy.parameter.Parameter("recovery_slew_amplitude", value=slew_amp),
                    rclpy.parameter.Parameter("recovery_boom_amplitude", value=boom_amp),
                    rclpy.parameter.Parameter("recovery_winch_amplitude", value=winch_amp),
                ]
                if len(tokens) == 7:
                    params.extend([
                        rclpy.parameter.Parameter("recovery_slew_period", value=max(float(tokens[4]), 0.1)),
                        rclpy.parameter.Parameter("recovery_boom_period", value=max(float(tokens[5]), 0.1)),
                        rclpy.parameter.Parameter("recovery_winch_period", value=max(float(tokens[6]), 0.1)),
                    ])
                self.set_parameters(params)
                self._set_training_profile("recovery")
                self._show_action("Recovery parameters updated")
            except ValueError:
                self.get_logger().warn("Recovery values must be numeric.")
            return

        if cmd == "rate":
            if len(tokens) < 4:
                self.get_logger().warn("Usage: rate <slew> <boom> <winch> [enable]")
                return
            try:
                self.rate_state.slew_rate = clamp(float(tokens[1]), -1.0, 1.0)
                self.rate_state.boom_rate = clamp(float(tokens[2]), -1.0, 1.0)
                self.rate_state.winch_rate = clamp(float(tokens[3]), -1.0, 1.0)
                self.rate_state.enable = self._parse_enable(tokens[4:], self.rate_state.enable)
                self.mode = "rate"
                self._show_action(
                    "Rate command armed",
                    f"slew={self.rate_state.slew_rate:.3f} boom={self.rate_state.boom_rate:.3f} winch={self.rate_state.winch_rate:.3f}",
                )
            except ValueError:
                self.get_logger().warn("Rate values must be numeric.")
            return

        if cmd == "target":
            if len(tokens) < 4:
                self.get_logger().warn("Usage: target <slew_pos> <trolley_pos> <winch_len> [enable]")
                return
            try:
                self.target_state.slew_position = float(tokens[1])
                self.target_state.trolley_position = float(tokens[2])
                self.target_state.winch_length = float(tokens[3])
                self.target_state.enable = self._parse_enable(tokens[4:], self.target_state.enable)
                self.mode = "target"
                self._show_action(
                    "Target command armed",
                    f"slew={self.target_state.slew_position:.3f} trolley={self.target_state.trolley_position:.3f} winch={self.target_state.winch_length:.3f}",
                )
            except ValueError:
                self.get_logger().warn("Target values must be numeric.")
            return

        if cmd == "enable":
            self.rate_state.enable = True
            self.target_state.enable = True
            self.disturbance_enable = True
            self.custom_disturbance.enable = True
            self._show_action("Commands enabled")
            return

        if cmd == "disable":
            self.rate_state.enable = False
            self.target_state.enable = False
            self.disturbance_enable = False
            self.custom_disturbance.enable = False
            self._show_action("Commands disabled")
            return

        if cmd == "stop":
            self.rate_state.slew_rate = 0.0
            self.rate_state.boom_rate = 0.0
            self.rate_state.winch_rate = 0.0
            self.disturbance_profile = "off"
            self._show_action("Stopped", "Rates zeroed and disturbance set to off.")
            return

        if cmd == "status":
            self._show_status()
            return

        if cmd in {"quit", "exit"}:
            self._show_action("Stopping operator terminal")
            self._running = False
            rclpy.shutdown()
            return

        self.get_logger().warn(f"Unknown command: {cmd}. Use 'menu' to list options.")

def main():
    rclpy.init()
    node = TeleopOperatorTraining()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
