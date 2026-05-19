import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import String

def clamp(value: float, limit: float) -> float:
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value

def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def quaternion_to_roll_pitch(x: float, y: float, z: float, w: float) -> tuple[float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    return roll, pitch

def roll_pitch_yaw_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w

class ShipDpHold(Node):
    def __init__(self) -> None:
        super().__init__('ship_dp_hold')

        command_qos = QoSProfile(depth=1)
        command_qos.reliability = ReliabilityPolicy.RELIABLE

        self.declare_parameter('ship_model_name', 'havyard_ship')
        self.declare_parameter('enable_waves', True)
        self.declare_parameter('enable_dp_hold', True)
        self.declare_parameter('sea_state_profile', 'moderate')
        self.declare_parameter('wave_amplitude', -1.0)
        self.declare_parameter('wave_period', -1.0)
        self.declare_parameter('wave_direction_deg', 90.0)
        self.declare_parameter('swap_roll_pitch_axes', False)
        self.declare_parameter('heave_scale', 0.0)
        self.declare_parameter('max_heave_vel', 0.25)
        self.declare_parameter('wave_control_topic', '/wave_control')
        self.declare_parameter('kp_attitude', 0.45)
        self.declare_parameter('kd_attitude', 0.90)
        self.declare_parameter('max_attitude_correction', 0.18)
        self.declare_parameter('kp_xy', 0.9)
        self.declare_parameter('kd_xy', 1.8)
        self.declare_parameter('kp_yaw', 1.6)
        self.declare_parameter('kd_yaw', 2.2)
        self.declare_parameter('max_linear_correction', 1.5)
        self.declare_parameter('max_angular_correction', 0.35)
        self.declare_parameter('publish_hz', 50.0)
        self.declare_parameter('set_pose_service', '/world/crane_world/set_pose')
        self.declare_parameter('home_x', float('nan'))
        self.declare_parameter('home_y', float('nan'))
        self.declare_parameter('home_z', float('nan'))
        self.declare_parameter('home_roll', float('nan'))
        self.declare_parameter('home_pitch', float('nan'))
        self.declare_parameter('home_yaw', float('nan'))

        self.ship_model_name = str(self.get_parameter('ship_model_name').value)
        self.enable_waves = bool(self.get_parameter('enable_waves').value)
        self.enable_dp_hold = bool(self.get_parameter('enable_dp_hold').value)
        self.sea_state_profile = str(self.get_parameter('sea_state_profile').value).strip().lower()

        self.wave_direction_deg = float(self.get_parameter('wave_direction_deg').value)
        self.swap_roll_pitch_axes = bool(self.get_parameter('swap_roll_pitch_axes').value)
        self.heave_scale = max(0.0, float(self.get_parameter('heave_scale').value))
        self.max_heave_vel = max(0.0, float(self.get_parameter('max_heave_vel').value))
        self.wave_control_topic = str(self.get_parameter('wave_control_topic').value)
        self.kp_attitude = float(self.get_parameter('kp_attitude').value)
        self.kd_attitude = float(self.get_parameter('kd_attitude').value)
        self.max_attitude_correction = float(self.get_parameter('max_attitude_correction').value)
        self.kp_xy = float(self.get_parameter('kp_xy').value)
        self.kd_xy = float(self.get_parameter('kd_xy').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.kd_yaw = float(self.get_parameter('kd_yaw').value)
        self.max_linear_correction = float(self.get_parameter('max_linear_correction').value)
        self.max_angular_correction = float(self.get_parameter('max_angular_correction').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.set_pose_service = str(self.get_parameter('set_pose_service').value)
        self.home_x = float(self.get_parameter('home_x').value)
        self.home_y = float(self.get_parameter('home_y').value)
        self.home_z = float(self.get_parameter('home_z').value)
        self.home_roll = float(self.get_parameter('home_roll').value)
        self.home_pitch = float(self.get_parameter('home_pitch').value)
        self.home_yaw = float(self.get_parameter('home_yaw').value)

        profile_amp, profile_period = self._sea_profile(self.sea_state_profile)
        wave_amp_param = float(self.get_parameter('wave_amplitude').value)
        wave_period_param = float(self.get_parameter('wave_period').value)
        self.wave_amplitude = wave_amp_param if wave_amp_param >= 0.0 else profile_amp
        self.wave_period = wave_period_param if wave_period_param > 0.0 else profile_period

        odom_topic = f'/model/{self.ship_model_name}/odometry'
        cmd_topic = f'/model/{self.ship_model_name}/cmd_vel'

        self.odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_cb, qos_profile_sensor_data)
        self.wave_control_sub = self.create_subscription(String, self.wave_control_topic, self._wave_control_cb, command_qos)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, command_qos)
        self.set_pose_client = self.create_client(SetEntityPose, self.set_pose_service)
        self.timer = self.create_timer(1.0 / max(self.publish_hz, 1.0), self._tick)

        self.last_odom: Optional[Odometry] = None
        self.x_ref: Optional[float] = None
        self.y_ref: Optional[float] = None
        self.z_ref: Optional[float] = None
        self.yaw_ref: Optional[float] = None
        self.roll_ref: Optional[float] = None
        self.pitch_ref: Optional[float] = None
        self.start_t = self.get_clock().now()
        self.roll_scale = 1.0
        self.pitch_scale = 1.0
        self.heave_scale_runtime = 1.0
        self.wave_amplitude_scale = 1.0
        self.recovery_boost_until: Optional[float] = None
        self.wave_pause_until: Optional[float] = None
        self.home_hold_until: Optional[float] = None
        self.last_home_pose_request_time: float = -1.0

        self.get_logger().info(
            f'Ship DP hold active on {cmd_topic}. '
            f'waves={self.enable_waves}, dp_hold={self.enable_dp_hold}, '
            f'profile={self.sea_state_profile}, amp={self.wave_amplitude:.3f}m, '
            f'period={self.wave_period:.2f}s, dir={self.wave_direction_deg:.1f}deg'
        )
        self.get_logger().info(
            f'Live wave control available on {self.wave_control_topic}. '
            'Commands: pitch_up/down, roll_up/down, heave_up/down, amp_up/down, level'
        )

    @staticmethod
    def _sea_profile(profile: str) -> tuple[float, float]:
        if profile == 'calm':
            return 0.12, 9.0
        if profile == 'rough':
            return 0.55, 6.0
        return 0.28, 7.5

    def _odom_cb(self, msg: Odometry) -> None:
        self.last_odom = msg
        if self.x_ref is None:
            self.x_ref = float(msg.pose.pose.position.x)
            self.y_ref = float(msg.pose.pose.position.y)
            self.z_ref = float(msg.pose.pose.position.z)
            q = msg.pose.pose.orientation
            self.yaw_ref = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            self.roll_ref, self.pitch_ref = quaternion_to_roll_pitch(q.x, q.y, q.z, q.w)
            self.get_logger().info(
                f'DP reference locked at x={self.x_ref:.2f}, y={self.y_ref:.2f}, '
                f'yaw={self.yaw_ref:.2f}rad, roll={self.roll_ref:.2f}rad, pitch={self.pitch_ref:.2f}rad'
            )

    def _log_wave_scales(self) -> None:
        self.get_logger().info(
            'Wave runtime scales: '
            f'pitch={self.pitch_scale:.2f}, roll={self.roll_scale:.2f}, '
            f'heave={self.heave_scale_runtime:.2f}, amp={self.wave_amplitude_scale:.2f}'
        )

    def _reset_wave_scales(self) -> None:
        self.roll_scale = 1.0
        self.pitch_scale = 1.0
        self.heave_scale_runtime = 1.0
        self.wave_amplitude_scale = 1.0

    def _request_home_pose(self) -> bool:
        use_configured_home = all(
            not math.isnan(value)
            for value in (
                self.home_x,
                self.home_y,
                self.home_z,
                self.home_roll,
                self.home_pitch,
                self.home_yaw,
            )
        )

        if not use_configured_home and (
            self.x_ref is None
            or self.y_ref is None
            or self.z_ref is None
            or self.roll_ref is None
            or self.pitch_ref is None
            or self.yaw_ref is None
        ):
            self.get_logger().warning('Cannot level yet: reference pose has not been locked from odometry.')
            return False

        if not self.set_pose_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warning(
                f'Level requested, but set_pose service {self.set_pose_service} is not available.'
            )
            return False

        if use_configured_home:
            home_x, home_y, home_z = self.home_x, self.home_y, self.home_z
            home_roll, home_pitch, home_yaw = self.home_roll, self.home_pitch, self.home_yaw
        else:
            home_x, home_y, home_z = self.x_ref, self.y_ref, self.z_ref
            home_roll, home_pitch, home_yaw = self.roll_ref, self.pitch_ref, self.yaw_ref

        qx, qy, qz, qw = roll_pitch_yaw_to_quaternion(home_roll, home_pitch, home_yaw)
        request = SetEntityPose.Request()
        request.entity.name = self.ship_model_name
        request.entity.type = Entity.MODEL
        request.pose.position.x = home_x
        request.pose.position.y = home_y
        request.pose.position.z = home_z
        request.pose.orientation.x = qx
        request.pose.orientation.y = qy
        request.pose.orientation.z = qz
        request.pose.orientation.w = qw
        self.set_pose_client.call_async(request)
        return True

    def _wave_control_cb(self, msg: String) -> None:
        command = msg.data.strip().lower()
        pitch_step = 0.25
        roll_step = 0.25
        scale_step = 0.25
        amplitude_step = 0.25
        max_scale = 5.0

        if command == 'pitch_up':
            self.pitch_scale = min(self.pitch_scale + pitch_step, max_scale)
        elif command == 'pitch_down':
            self.pitch_scale = max(self.pitch_scale - pitch_step, 0.0)
        elif command == 'roll_up':
            self.roll_scale = min(self.roll_scale + roll_step, max_scale)
        elif command == 'roll_down':
            self.roll_scale = max(self.roll_scale - roll_step, 0.0)
        elif command == 'heave_up':
            self.heave_scale_runtime = min(self.heave_scale_runtime + scale_step, max_scale)
        elif command == 'heave_down':
            self.heave_scale_runtime = max(self.heave_scale_runtime - scale_step, 0.0)
        elif command == 'amp_up':
            self.wave_amplitude_scale = min(self.wave_amplitude_scale + amplitude_step, max_scale)
        elif command == 'amp_down':
            self.wave_amplitude_scale = max(self.wave_amplitude_scale - amplitude_step, 0.0)
        elif command == 'level':
            self._reset_wave_scales()
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            # Send the ship back to home pose and pin it there briefly so
            # residual motion does not immediately pull it away again.
            self.wave_pause_until = now_sec + 4.0
            self.recovery_boost_until = now_sec + 6.0
            self.home_hold_until = now_sec + 2.5
            self.last_home_pose_request_time = -1.0
            self._request_home_pose()
        else:
            self.get_logger().warning(
                f'Unknown wave control command "{msg.data}". '
                'Use: pitch_up/down, roll_up/down, heave_up/down, amp_up/down, level'
            )
            return

        self._log_wave_scales()

    def _tick(self) -> None:
        cmd = Twist()
        now = self.get_clock().now()
        t = (now - self.start_t).nanoseconds * 1e-9

        # Synthetic wave motion (heave + roll + pitch via velocity commands)
        now_sec = now.nanoseconds * 1e-9
        waves_paused = self.wave_pause_until is not None and now_sec < self.wave_pause_until
        if self.wave_pause_until is not None and now_sec >= self.wave_pause_until:
            self.wave_pause_until = None

        if self.enable_waves and self.wave_period > 0.01 and not waves_paused:
            omega = 2.0 * math.pi / self.wave_period
            direction_rad = math.radians(self.wave_direction_deg)

            if self.swap_roll_pitch_axes:
                roll_weight = abs(math.cos(direction_rad))
                pitch_weight = abs(math.sin(direction_rad))
            else:
                roll_weight = abs(math.sin(direction_rad))
                pitch_weight = abs(math.cos(direction_rad))
            wave_amplitude = self.wave_amplitude * self.wave_amplitude_scale
            heave_amp = self.heave_scale * self.heave_scale_runtime * wave_amplitude
            roll_amp = 0.09 * wave_amplitude * roll_weight * self.roll_scale
            pitch_amp = 0.09 * wave_amplitude * pitch_weight * self.pitch_scale

            cmd.linear.z = clamp(heave_amp * omega * math.cos(omega * t), self.max_heave_vel)
            roll_cmd = roll_amp * omega * math.sin(omega * t)
            pitch_cmd = pitch_amp * omega * math.sin(omega * t + 0.5 * math.pi)

            if self.swap_roll_pitch_axes:
                # Havyard's effective visual/body axes are rotated relative to the
                # intuitive marine frame, so front waves should still look like pitch.
                cmd.angular.x = pitch_cmd
                cmd.angular.y = roll_cmd
            else:
                cmd.angular.x = roll_cmd
                cmd.angular.y = pitch_cmd

        home_hold_active = self.home_hold_until is not None and now_sec < self.home_hold_until
        if self.home_hold_until is not None and now_sec >= self.home_hold_until:
            self.home_hold_until = None

        if home_hold_active:
            # Keep the model pinned near home for a short settling window after
            # `level`, and avoid injecting residual velocity.
            if self.last_home_pose_request_time < 0.0 or (now_sec - self.last_home_pose_request_time) >= 0.2:
                if self._request_home_pose():
                    self.last_home_pose_request_time = now_sec
            self.cmd_pub.publish(Twist())
            return

        # Keep pitch and roll drifting back toward trim over time.
        if self.last_odom is not None and self.roll_ref is not None and self.pitch_ref is not None:
            q = self.last_odom.pose.pose.orientation
            tw = self.last_odom.twist.twist
            roll, pitch = quaternion_to_roll_pitch(q.x, q.y, q.z, q.w)

            recovery_gain = 1.0
            if self.recovery_boost_until is not None and now_sec < self.recovery_boost_until:
                recovery_gain = 6.0
            elif self.recovery_boost_until is not None and now_sec >= self.recovery_boost_until:
                self.recovery_boost_until = None

            eroll = wrap_to_pi(roll - self.roll_ref)
            epitch = wrap_to_pi(pitch - self.pitch_ref)

            restore_roll = clamp(
                -(recovery_gain * self.kp_attitude * eroll + recovery_gain * self.kd_attitude * float(tw.angular.x)),
                recovery_gain * self.max_attitude_correction,
            )
            restore_pitch = clamp(
                -(recovery_gain * self.kp_attitude * epitch + recovery_gain * self.kd_attitude * float(tw.angular.y)),
                recovery_gain * self.max_attitude_correction,
            )

            if self.swap_roll_pitch_axes:
                cmd.angular.x += restore_pitch
                cmd.angular.y += restore_roll
            else:
                cmd.angular.x += restore_roll
                cmd.angular.y += restore_pitch

        # DP hold in x / y / yaw only.
        if self.enable_dp_hold and self.last_odom is not None and self.x_ref is not None and self.y_ref is not None and self.yaw_ref is not None:
            p = self.last_odom.pose.pose.position
            q = self.last_odom.pose.pose.orientation
            tw = self.last_odom.twist.twist
            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

            ex = float(p.x) - self.x_ref
            ey = float(p.y) - self.y_ref
            eyaw = wrap_to_pi(yaw - self.yaw_ref)

            vx = float(tw.linear.x)
            vy = float(tw.linear.y)
            wz = float(tw.angular.z)

            dp_vx = clamp(-(self.kp_xy * ex + self.kd_xy * vx), self.max_linear_correction)
            dp_vy = clamp(-(self.kp_xy * ey + self.kd_xy * vy), self.max_linear_correction)
            dp_wz = clamp(-(self.kp_yaw * eyaw + self.kd_yaw * wz), self.max_angular_correction)

            cmd.linear.x += dp_vx
            cmd.linear.y += dp_vy
            cmd.angular.z += dp_wz

        # If both wave and DP are disabled, avoid publishing empty commands.
        if (
            not self.enable_waves
            and (
                not self.enable_dp_hold
                or self.last_odom is None
                or self.x_ref is None
                or self.y_ref is None
                or self.yaw_ref is None
            )
        ):
            return

        self.cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ShipDpHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
