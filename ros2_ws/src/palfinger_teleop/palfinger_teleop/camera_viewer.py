#!/usr/bin/env python3
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, Joy
from std_msgs.msg import Float64

class CameraViewer(Node):
    def __init__(self) -> None:
        super().__init__("camera_viewer")

        command_qos = QoSProfile(depth=1)
        command_qos.reliability = ReliabilityPolicy.RELIABLE

        self.declare_parameter("front_topic", "/crane/boom_tip_camera/image")
        self.declare_parameter("left_topic", "/crane/hook_mid_camera/image")
        self.declare_parameter("right_topic", "/crane/cabin_camera/image")
        self.declare_parameter("window_name", "Crane Cameras")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("joy_enabled", True)
        self.declare_parameter("joy_cycle_button", 0)
        self.declare_parameter("joy_zoom_modifier_button", 5)
        self.declare_parameter("joy_pan_axis", 6)
        self.declare_parameter("joy_tilt_zoom_axis", 7)
        self.declare_parameter("joy_pan_axis_sign", -1.0)
        self.declare_parameter("joy_axis_threshold", 0.5)
        self.declare_parameter("joy_repeat_hz", 8.0)
        self.declare_parameter("front_pan_cmd_topic", "/crane/boom_tip_camera/pan_cmd")
        self.declare_parameter("front_tilt_cmd_topic", "/crane/boom_tip_camera/tilt_cmd")
        self.declare_parameter("left_pan_cmd_topic", "/crane/hook_mid_camera/pan_cmd")
        self.declare_parameter("left_tilt_cmd_topic", "/crane/hook_mid_camera/tilt_cmd")
        self.declare_parameter("right_pan_cmd_topic", "/crane/cabin_camera/pan_cmd")
        self.declare_parameter("right_tilt_cmd_topic", "/crane/cabin_camera/tilt_cmd")

        self.front_topic = str(self.get_parameter("front_topic").value)
        self.left_topic = str(self.get_parameter("left_topic").value)
        self.right_topic = str(self.get_parameter("right_topic").value)
        self.window_name = str(self.get_parameter("window_name").value)
        self.joy_topic = str(self.get_parameter("joy_topic").value)
        self._joy_enabled = bool(self.get_parameter("joy_enabled").value)

        self._front: Optional[np.ndarray] = None
        self._left: Optional[np.ndarray] = None
        self._right: Optional[np.ndarray] = None
        self._mode_cycle = ["all", "front", "left", "right"]
        self._mode_index = 0
        self._view_state = {
            "front": {"pan_deg": 0.0, "tilt_deg": 0.0, "zoom": 1.0},
            "left": {"pan_deg": 0.0, "tilt_deg": 0.0, "zoom": 1.0},
            "right": {"pan_deg": 0.0, "tilt_deg": 0.0, "zoom": 1.0},
        }
        self._display_state = {
            "front": {"pan_deg": 0.0, "tilt_deg": 0.0, "zoom": 1.0},
            "left": {"pan_deg": 0.0, "tilt_deg": 0.0, "zoom": 1.0},
            "right": {"pan_deg": 0.0, "tilt_deg": 0.0, "zoom": 1.0},
        }
        self._max_pan_deg = 80.0
        self._max_tilt_deg = 80.0
        self._max_zoom = 4.0
        self._min_zoom = 1.0
        self._pan_step_deg = 3.0
        self._tilt_step_deg = 3.0
        self._zoom_step = 0.2
        self._joy_axes: list[float] = []
        self._joy_buttons: list[int] = []
        self._joy_cycle_was_pressed = False
        self._joy_last_repeat_time = 0.0
        self._joy_last_command = (0, 0, 0)
        self._pan_pubs = {
            "front": self.create_publisher(Float64, str(self.get_parameter("front_pan_cmd_topic").value), command_qos),
            "left": self.create_publisher(Float64, str(self.get_parameter("left_pan_cmd_topic").value), command_qos),
            "right": self.create_publisher(Float64, str(self.get_parameter("right_pan_cmd_topic").value), command_qos),
        }
        self._tilt_pubs = {
            "front": self.create_publisher(Float64, str(self.get_parameter("front_tilt_cmd_topic").value), command_qos),
            "left": self.create_publisher(Float64, str(self.get_parameter("left_tilt_cmd_topic").value), command_qos),
            "right": self.create_publisher(Float64, str(self.get_parameter("right_tilt_cmd_topic").value), command_qos),
        }

        self.create_subscription(Image, self.front_topic, self._front_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.left_topic, self._left_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.right_topic, self._right_cb, qos_profile_sensor_data)
        if self._joy_enabled:
            self.create_subscription(Joy, self.joy_topic, self._joy_cb, qos_profile_sensor_data)

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1280, 720)

    def _front_cb(self, msg: Image) -> None:
        self._front = self._image_to_bgr(msg)

    def _left_cb(self, msg: Image) -> None:
        self._left = self._image_to_bgr(msg)

    def _right_cb(self, msg: Image) -> None:
        self._right = self._image_to_bgr(msg)

    def _joy_cb(self, msg: Joy) -> None:
        self._joy_axes = list(msg.axes)
        self._joy_buttons = list(msg.buttons)

    def _image_to_bgr(self, msg: Image) -> Optional[np.ndarray]:
        if msg.encoding not in ("rgb8", "bgr8"):
            return None
        channels = 3
        expected_step = msg.width * channels
        if msg.step < expected_step:
            return None
        buffer = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            image = buffer.reshape((msg.height, msg.step))[:, :expected_step]
            image = image.reshape((msg.height, msg.width, channels))
        except ValueError:
            return None
        if msg.encoding == "rgb8":
            image = image[:, :, ::-1]
        return image.copy()

    def _resize(self, image: np.ndarray, width: int, height: int) -> np.ndarray:
        src_h, src_w = image.shape[:2]
        if width < src_w or height < src_h:
            interpolation = cv2.INTER_AREA
        elif width > src_w or height > src_h:
            interpolation = cv2.INTER_CUBIC
        else:
            interpolation = cv2.INTER_LINEAR
        return cv2.resize(image, (width, height), interpolation=interpolation)

    def _label(self, image: np.ndarray, text: str, show_help: bool = True) -> np.ndarray:
        out = image.copy()
        bar_height = 64 if show_help else 36
        cv2.rectangle(out, (0, 0), (out.shape[1], bar_height), (20, 20, 20), -1)
        cv2.putText(out, text, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (240, 240, 240), 2, cv2.LINE_AA)
        if show_help:
            help_text = "Xbox: B0 cycle | Axis 6 pan | Axis 7 tilt | Hold B5 + Axis 7 zoom"
            cv2.putText(out, help_text, (12, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        return out

    def _status_text(self, label: str) -> str:
        mode = self._mode_cycle[self._mode_index]
        state = self._display_state.get(mode, {"pan_deg": 0.0, "tilt_deg": 0.0, "zoom": 1.0})
        return (
            f"{label} | pan {state['pan_deg']:+.0f} deg | tilt {state['tilt_deg']:+.0f} deg | "
            f"zoom {state['zoom']:.1f}x"
        )

    def _placeholder(self, text: str, width: int, height: int) -> np.ndarray:
        image = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.putText(image, text, (30, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (180, 180, 180), 2, cv2.LINE_AA)
        return image

    def _render_all(self) -> np.ndarray:
        width = 1280
        height = 720
        side_h = 180
        side_w = width // 2
        main_h = height - side_h
        front = self._front if self._front is not None else self._placeholder("Waiting for front camera", width, main_h)
        left = self._left if self._left is not None else self._placeholder("Waiting for left camera", side_w, side_h)
        right = self._right if self._right is not None else self._placeholder("Waiting for right camera", side_w, side_h)
        front = self._label(self._resize(front, width, main_h), "Boom Tip", show_help=False)
        left = self._label(self._resize(left, side_w, side_h), "Hook", show_help=False)
        right = self._label(self._resize(right, side_w, side_h), "Cabin", show_help=False)
        canvas = np.zeros((height, width, 3), dtype=np.uint8)
        canvas[:main_h, :, :] = front
        canvas[main_h:, :side_w, :] = left
        canvas[main_h:, side_w:, :] = right
        return canvas

    def _apply_digital_crop(self, image: np.ndarray, zoom: float) -> np.ndarray:
        if zoom <= 1.01:
            return image
        src_h, src_w = image.shape[:2]
        crop_w = max(int(src_w / zoom), 64)
        crop_h = max(int(src_h / zoom), 64)
        x0 = max(0, (src_w - crop_w) // 2)
        y0 = max(0, (src_h - crop_h) // 2)
        cropped = image[y0:y0 + crop_h, x0:x0 + crop_w]
        return self._resize(cropped, src_w, src_h)

    def _render_single(self, image: Optional[np.ndarray], label: str) -> np.ndarray:
        width = 1280
        height = 720
        mode = self._mode_cycle[self._mode_index]
        state = self._display_state.get(mode, {"pan_deg": 0.0, "tilt_deg": 0.0, "zoom": 1.0})
        if image is None:
            image = self._placeholder(f"Waiting for {label.lower()} camera", width, height)
        image = self._apply_digital_crop(image, state["zoom"])
        image = self._resize(image, width, height)
        return self._label(image, self._status_text(label), show_help=True)

    def _publish_ptz(self, mode: str) -> None:
        if mode not in self._view_state:
            return
        state = self._view_state[mode]
        pan = Float64()
        pan.data = np.deg2rad(-state["pan_deg"]).item()
        tilt = Float64()
        tilt.data = np.deg2rad(-state["tilt_deg"]).item()
        self._pan_pubs[mode].publish(pan)
        self._tilt_pubs[mode].publish(tilt)

    def _set_mode_index(self, index: int) -> None:
        self._mode_index = index % len(self._mode_cycle)

    def _cycle_mode(self) -> None:
        self._set_mode_index(self._mode_index + 1)

    def _adjust_pan(self, mode: str, direction: int) -> None:
        if mode not in self._view_state or direction == 0:
            return
        self._view_state[mode]["pan_deg"] = float(np.clip(
            self._view_state[mode]["pan_deg"] + (direction * self._pan_step_deg),
            -self._max_pan_deg,
            self._max_pan_deg,
        ))
        self._display_state[mode]["pan_deg"] = self._view_state[mode]["pan_deg"]
        self._publish_ptz(mode)

    def _adjust_tilt(self, mode: str, direction: int) -> None:
        if mode not in self._view_state or direction == 0:
            return
        self._view_state[mode]["tilt_deg"] = float(np.clip(
            self._view_state[mode]["tilt_deg"] + (direction * self._tilt_step_deg),
            -self._max_tilt_deg,
            self._max_tilt_deg,
        ))
        self._display_state[mode]["tilt_deg"] = self._view_state[mode]["tilt_deg"]
        self._publish_ptz(mode)

    def _adjust_zoom(self, mode: str, direction: int) -> None:
        if mode not in self._view_state or direction == 0:
            return
        self._view_state[mode]["zoom"] = float(np.clip(
            self._view_state[mode]["zoom"] + (direction * self._zoom_step),
            self._min_zoom,
            self._max_zoom,
        ))
        self._display_state[mode]["zoom"] = self._view_state[mode]["zoom"]

    def _reset_view(self, mode: str) -> None:
        if mode not in self._view_state:
            return
        self._view_state[mode] = {"pan_deg": 0.0, "tilt_deg": 0.0, "zoom": 1.0}
        self._display_state[mode] = {"pan_deg": 0.0, "tilt_deg": 0.0, "zoom": 1.0}
        self._publish_ptz(mode)

    def _get_axis(self, index: int) -> float:
        if index < 0 or index >= len(self._joy_axes):
            return 0.0
        return float(self._joy_axes[index])

    def _get_button(self, index: int) -> bool:
        if index < 0 or index >= len(self._joy_buttons):
            return False
        return bool(self._joy_buttons[index])

    def _digital_axis(self, value: float) -> int:
        threshold = float(self.get_parameter("joy_axis_threshold").value)
        if value >= threshold:
            return 1
        if value <= -threshold:
            return -1
        return 0

    def process_joy_input(self) -> None:
        if not self._joy_enabled or (not self._joy_axes and not self._joy_buttons):
            return
        cycle_button = int(self.get_parameter("joy_cycle_button").value)
        zoom_modifier_button = int(self.get_parameter("joy_zoom_modifier_button").value)
        pan_axis = int(self.get_parameter("joy_pan_axis").value)
        tilt_zoom_axis = int(self.get_parameter("joy_tilt_zoom_axis").value)

        cycle_pressed = self._get_button(cycle_button)
        if cycle_pressed and not self._joy_cycle_was_pressed:
            self._cycle_mode()
        self._joy_cycle_was_pressed = cycle_pressed

        mode = self._mode_cycle[self._mode_index]
        if mode == "all":
            self._joy_last_command = (0, 0, 0)
            return

        pan_axis_sign = float(self.get_parameter("joy_pan_axis_sign").value)
        pan_dir = self._digital_axis(self._get_axis(pan_axis) * pan_axis_sign)
        vertical_dir = self._digital_axis(self._get_axis(tilt_zoom_axis))
        zoom_modifier = self._get_button(zoom_modifier_button)
        tilt_dir = 0 if zoom_modifier else vertical_dir
        zoom_dir = vertical_dir if zoom_modifier else 0
        command = (pan_dir, tilt_dir, zoom_dir)

        if command == (0, 0, 0):
            self._joy_last_command = command
            return

        now = self.get_clock().now().nanoseconds / 1e9
        repeat_hz = max(float(self.get_parameter("joy_repeat_hz").value), 1.0)
        repeat_period = 1.0 / repeat_hz
        if command != self._joy_last_command or (now - self._joy_last_repeat_time) >= repeat_period:
            self._adjust_pan(mode, pan_dir)
            self._adjust_tilt(mode, tilt_dir)
            self._adjust_zoom(mode, zoom_dir)
            self._joy_last_repeat_time = now
        self._joy_last_command = command

    def render_current_view(self) -> np.ndarray:
        mode = self._mode_cycle[self._mode_index]
        if mode == "front":
            return self._render_single(self._front, "Boom Tip")
        if mode == "left":
            return self._render_single(self._left, "Hook")
        if mode == "right":
            return self._render_single(self._right, "Cabin")
        return self._render_all()

def main() -> None:
    rclpy.init()
    node = CameraViewer()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.process_joy_input()
            frame = node.render_current_view()
            cv2.imshow(node.window_name, frame)
            cv2.waitKey(1)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
