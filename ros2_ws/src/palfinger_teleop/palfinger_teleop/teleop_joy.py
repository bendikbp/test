import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from sensor_msgs.msg import Joy
from palfinger_msgs.msg import CraneCommand

# For running the simulation with joystic input

def deadzone(x: float, dz: float) -> float:
    if abs(x) < dz:
        return 0.0
    # rescale so it still reaches 1.0 at full stick
    return (x - dz) / (1.0 - dz) if x > 0.0 else (x + dz) / (1.0 - dz)

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

# Speed adjustment
def expo(x: float, power: float) -> float:
    p = max(1.0, float(power))
    return (abs(x) ** p) * (1.0 if x >= 0.0 else -1.0)

class CraneTeleopJoy(Node):
    def __init__(self):
        super().__init__("crane_teleop_joy")

        command_qos = QoSProfile(depth=1)
        command_qos.reliability = ReliabilityPolicy.RELIABLE

        # Topics
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("cmd_topic", "/crane/cmd")

        # Axis mapping (common default for Xbox on Linux, but configurable)
        # Adjust later after you see /joy axes order.
        self.declare_parameter("axis_slew", 0)   # left stick left/right
        self.declare_parameter("axis_boom", 1)   # left stick up/down
        self.declare_parameter("axis_winch", 4)  # right stick up/down (often 4)

        # Scaling
        self.declare_parameter("scale_slew", 1.0)
        self.declare_parameter("scale_boom", 1.0)
        self.declare_parameter("scale_winch", 1.0)
        self.declare_parameter("expo_slew", 1.0)
        self.declare_parameter("expo_boom", 1.0)
        self.declare_parameter("expo_winch", 1.0)

        # Deadzone and rate
        self.declare_parameter("deadzone", 0.08)
        # Per-axis overrides. Negative => use shared deadzone.
        self.declare_parameter("deadzone_slew", -1.0)
        self.declare_parameter("deadzone_boom", -1.0)
        self.declare_parameter("deadzone_winch", -1.0)
        self.declare_parameter("publish_hz", 50.0)

        # Enable/deadman
        self.declare_parameter("require_deadman", True)
        self.declare_parameter("deadman_button", 3)  # Y on common Xbox mappings
        self.declare_parameter("deadman_buttons", [3])
        self.declare_parameter("deadman_toggle", True)
        self.declare_parameter("enable_when_deadman", True)

        self.pub = self.create_publisher(
            CraneCommand, str(self.get_parameter("cmd_topic").value), command_qos
        )

        joy_topic = str(self.get_parameter("joy_topic").value)
        self.sub = self.create_subscription(Joy, joy_topic, self.on_joy, qos_profile_sensor_data)

        self.last_joy: Joy | None = None
        self.last_msg = CraneCommand()
        self._deadman_enabled = False
        self._deadman_was_pressed = False

        hz = float(self.get_parameter("publish_hz").value)
        period = 1.0 / max(hz, 1.0)
        self.timer = self.create_timer(period, self.publish_cmd)

        self.get_logger().info(
            f"Listening to {joy_topic} and publishing {self.get_parameter('cmd_topic').value} at {hz:.1f} Hz"
        )

    def on_joy(self, msg: Joy):
        self.last_joy = msg

    def publish_cmd(self):
        msg = CraneCommand()

        # Default safe output
        msg.slew_rate = 0.0
        msg.boom_rate = 0.0
        msg.winch_rate = 0.0
        msg.enable = False

        if self.last_joy is None:
            self.pub.publish(msg)
            return

        dz = float(self.get_parameter("deadzone").value)
        dz_slew = float(self.get_parameter("deadzone_slew").value)
        dz_boom = float(self.get_parameter("deadzone_boom").value)
        dz_winch = float(self.get_parameter("deadzone_winch").value)
        if dz_slew < 0.0:
            dz_slew = dz
        if dz_boom < 0.0:
            dz_boom = dz
        if dz_winch < 0.0:
            dz_winch = dz

        def get_axis(i: int) -> float:
            if i < 0 or i >= len(self.last_joy.axes):
                return 0.0
            return float(self.last_joy.axes[i])

        def get_button(i: int) -> bool:
            if i < 0 or i >= len(self.last_joy.buttons):
                return False
            return bool(self.last_joy.buttons[i])

        axis_slew = int(self.get_parameter("axis_slew").value)
        axis_boom = int(self.get_parameter("axis_boom").value)
        axis_winch = int(self.get_parameter("axis_winch").value)

        slew = deadzone(get_axis(axis_slew), dz_slew)
        boom = deadzone(get_axis(axis_boom), dz_boom)
        winch = deadzone(get_axis(axis_winch), dz_winch)

        slew = expo(slew, float(self.get_parameter("expo_slew").value))
        boom = expo(boom, float(self.get_parameter("expo_boom").value))
        winch = expo(winch, float(self.get_parameter("expo_winch").value))

        slew *= float(self.get_parameter("scale_slew").value)
        boom *= float(self.get_parameter("scale_boom").value)
        winch *= float(self.get_parameter("scale_winch").value)

        # clamp to [-1, 1] just in case scaling overshoots
        msg.slew_rate = clamp(slew, -1.0, 1.0)
        msg.boom_rate = clamp(boom, -1.0, 1.0)
        msg.winch_rate = clamp(winch, -1.0, 1.0)

        require_deadman = bool(self.get_parameter("require_deadman").value)
        if require_deadman:
            deadman_buttons = self.get_parameter("deadman_buttons").value
            if deadman_buttons:
                deadman_pressed = any(get_button(int(i)) for i in deadman_buttons)
            else:
                deadman_button = int(self.get_parameter("deadman_button").value)
                deadman_pressed = get_button(deadman_button)

            if bool(self.get_parameter("deadman_toggle").value):
                if deadman_pressed and not self._deadman_was_pressed:
                    self._deadman_enabled = not self._deadman_enabled
                    state = "enabled" if self._deadman_enabled else "disabled"
                    self.get_logger().info(f"Crane teleop {state} by deadman toggle.")
                self._deadman_was_pressed = deadman_pressed
                active = self._deadman_enabled
            else:
                active = deadman_pressed

            msg.enable = active and bool(self.get_parameter("enable_when_deadman").value)
            if not active:
                # If deadman is inactive, force zero outputs.
                msg.slew_rate = 0.0
                msg.boom_rate = 0.0
                msg.winch_rate = 0.0
        else:
            # Treat teleop as active only when a real post-deadzone command exists.
            msg.enable = any(
                abs(value) > 0.0
                for value in (msg.slew_rate, msg.boom_rate, msg.winch_rate)
            )

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CraneTeleopJoy()
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
