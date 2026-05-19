#!/usr/bin/env python3
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from palfinger_msgs.msg import HmiRangeState

class HmiRangeViewer(Node):
    def __init__(self) -> None:
        super().__init__("hmi_range_viewer")

        command_qos = QoSProfile(depth=1)
        command_qos.reliability = ReliabilityPolicy.RELIABLE

        self.declare_parameter("ranges_topic", "/crane/hmi/ranges")
        self.declare_parameter("window_name", "Palfinger HMI Ranges")
        self.declare_parameter("refresh_hz", 15.0)

        self.ranges_topic = str(self.get_parameter("ranges_topic").value)
        self.window_name = str(self.get_parameter("window_name").value)
        refresh_hz = max(float(self.get_parameter("refresh_hz").value), 1.0)

        self._last_msg: HmiRangeState | None = None
        self.create_subscription(HmiRangeState, self.ranges_topic, self._ranges_cb, command_qos)
        self.create_timer(1.0 / refresh_hz, self._render)

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def _ranges_cb(self, msg: HmiRangeState) -> None:
        self._last_msg = msg

    def _metric_color(self, value: float, in_bounds: bool) -> tuple[int, int, int]:
        if math.isnan(value):
            return (90, 90, 90)
        if value < 0.0:
            return (140, 140, 140)
        if in_bounds and value <= 0.75:
            return (60, 180, 255)
        if in_bounds:
            return (60, 200, 120)
        return (140, 140, 140)

    def _format_metric(self, value: float) -> str:
        if math.isnan(value):
            return "--"
        return f"{value:5.2f} m"

    def _draw_metric(
        self,
        canvas: np.ndarray,
        *,
        top: int,
        title: str,
        value: float,
        in_bounds: bool,
        hint: str,
    ) -> None:
        color = self._metric_color(value, in_bounds)
        cv2.rectangle(canvas, (30, top), (770, top + 100), color, -1)
        cv2.putText(canvas, title, (50, top + 34), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (245, 245, 245), 2, cv2.LINE_AA)
        cv2.putText(
            canvas,
            self._format_metric(value),
            (50, top + 76),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.1,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(canvas, hint, (360, top + 76), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (245, 245, 245), 2, cv2.LINE_AA)

    def _render(self) -> None:
        canvas = np.zeros((470, 800, 3), dtype=np.uint8)
        canvas[:, :] = (22, 26, 32)

        cv2.putText(canvas, "Crane HMI Range Monitor", (30, 42), cv2.FONT_HERSHEY_SIMPLEX, 1.05, (235, 235, 235), 2, cv2.LINE_AA)

        if self._last_msg is None:
            cv2.putText(canvas, "Waiting for HMI range data...", (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (180, 180, 180), 2, cv2.LINE_AA)
            cv2.imshow(self.window_name, canvas)
            cv2.waitKey(1)
            return

        self._draw_metric(
            canvas,
            top=75,
            title="Hook to Container Top",
            value=float(self._last_msg.hook_to_container_top),
            in_bounds=bool(self._last_msg.hook_over_container),
            hint="directly over container" if self._last_msg.hook_over_container else "outside container footprint",
        )
        self._draw_metric(
            canvas,
            top=190,
            title="Hook to Ship Deck",
            value=float(self._last_msg.hook_to_ship_deck),
            in_bounds=bool(self._last_msg.hook_over_ship_deck),
            hint="directly over ship deck" if self._last_msg.hook_over_ship_deck else "outside ship deck footprint",
        )
        self._draw_metric(
            canvas,
            top=305,
            title="Hook to Platform Deck",
            value=float(self._last_msg.hook_to_platform_deck),
            in_bounds=bool(self._last_msg.hook_over_platform_deck),
            hint="directly over platform deck" if self._last_msg.hook_over_platform_deck else "outside platform footprint",
        )

        cv2.rectangle(canvas, (30, 415), (770, 450), (45, 56, 68), -1)
        cv2.putText(canvas, "Load Mass", (50, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (235, 235, 235), 2, cv2.LINE_AA)
        cv2.putText(
            canvas,
            f"{float(self._last_msg.load_mass_kg):6.1f} kg",
            (560, 440),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        status = f"Status: {self._last_msg.status}"
        hook_xyz = (
            f"Hook xyz: ({self._last_msg.hook_world_x:6.2f}, "
            f"{self._last_msg.hook_world_y:6.2f}, {self._last_msg.hook_world_z:6.2f})"
        )
        cv2.putText(canvas, status, (30, 465), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(canvas, hook_xyz, (340, 465), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)

        cv2.imshow(self.window_name, canvas)
        cv2.waitKey(1)

def main() -> None:
    rclpy.init()
    node = HmiRangeViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
