#!/usr/bin/env python3
# dobot_sequence_node.py
#
# ros2 jazzy + rclpy
# - enable robot
# - set speed factor
# - execute a MovJ joint sequence
# - inputs are specified in radians, converted to degrees for MovJ

import math
import time

import rclpy
from rclpy.node import Node

from dobot_msgs_v4.srv import EnableRobot, SpeedFactor, MovJ


def rad_to_deg(x: float) -> float:
    return x * 180.0 / math.pi


class DobotSequenceNode(Node):
    def __init__(self):
        super().__init__("dobot_sequence_node")

        # ---- tune these ----
        self.speed_ratio = 5           # SpeedFactor ratio (from your docs example)
        self.wait_after_move_s = 2.0     # time-based wait after each MovJ call
        # --------------------

        # Service names (match your CLI examples)
        self.enable_srv_name = "/dobot_bringup_ros2/srv/EnableRobot"
        self.speed_srv_name  = "/dobot_bringup_ros2/srv/SpeedFactor"
        self.movj_srv_name   = "/dobot_bringup_ros2/srv/MovJ"

        # Create clients
        self.enable_cli = self.create_client(EnableRobot, self.enable_srv_name)
        self.speed_cli  = self.create_client(SpeedFactor, self.speed_srv_name)
        self.movj_cli   = self.create_client(MovJ, self.movj_srv_name)

        # Wait for services
        self._wait_for_service(self.enable_cli, self.enable_srv_name)
        self._wait_for_service(self.speed_cli,  self.speed_srv_name)
        self._wait_for_service(self.movj_cli,   self.movj_srv_name)

        # Run once then exit
        self.run_sequence()

        self.get_logger().info("Sequence finished. Shutting down node.")
        rclpy.shutdown()

    def _wait_for_service(self, client, name: str):
        self.get_logger().info(f"Waiting for service {name} ...")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Service {name} not available yet...")
        self.get_logger().info(f"Service {name} is available.")

    def _call_service_sync(self, client, req, label: str):
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            raise RuntimeError(f"{label} failed: {future.exception()}")
        return future.result()

    def enable_robot(self):
        req = EnableRobot.Request()  # your CLI uses "{}" so no fields needed
        resp = self._call_service_sync(self.enable_cli, req, "EnableRobot")
        self.get_logger().info(f"EnableRobot response: {resp}")
        return resp

    def set_speed_factor(self, ratio: int):
        req = SpeedFactor.Request()
        req.ratio = int(ratio)
        resp = self._call_service_sync(self.speed_cli, req, "SpeedFactor")
        self.get_logger().info(f"SpeedFactor response: {resp}")
        return resp

    def movj_deg(self, deg6):
        # deg6 = (a,b,c,d,e,f) in DEGREES
        req = MovJ.Request()
        req.mode = True
        req.a, req.b, req.c, req.d, req.e, req.f = [float(x) for x in deg6]
        req.param_value = []  # per your examples

        self.get_logger().info(
            f"Calling MovJ (deg): a={req.a:.3f}, b={req.b:.3f}, c={req.c:.3f}, "
            f"d={req.d:.3f}, e={req.e:.3f}, f={req.f:.3f}"
        )

        resp = self._call_service_sync(self.movj_cli, req, "MovJ")
        # You showed robot_return='{5}', res=0
        try:
            self.get_logger().info(f"MovJ response: robot_return={resp.robot_return}, res={resp.res}")
        except Exception:
            self.get_logger().info(f"MovJ response: {resp}")
        return resp

    def run_sequence(self):
        self.get_logger().info("=== Starting Dobot sequence ===")

        # 1) enable + speed
        self.enable_robot()
        self.set_speed_factor(self.speed_ratio)

        # 2) your sequence (given in radians)
        seq_rad = [
            (0.0, 0.0, 0.0,   0.0,   0.0, 0.0),
            (0.0, 0.0, 0.108, 0.645, 0.0, 0.0),
            (0.0, 0.0, 0.387, 0.645, 0.0, 0.0),
            (0.0, 0.0, 0.0,   0.0,   0.0, 0.0),
        ]

        # convert to degrees
        seq_deg = []
        for pose in seq_rad:
            seq_deg.append(tuple(rad_to_deg(x) for x in pose))

        # 3) execute
        for i, pose_deg in enumerate(seq_deg, start=1):
            self.get_logger().info(f"--- Step {i}/{len(seq_deg)} ---")
            self.movj_deg(pose_deg)
            self.get_logger().info(f"Waiting {self.wait_after_move_s:.1f}s ...")
            time.sleep(self.wait_after_move_s)


def main():
    rclpy.init()
    DobotSequenceNode()


if __name__ == "__main__":
    main()
