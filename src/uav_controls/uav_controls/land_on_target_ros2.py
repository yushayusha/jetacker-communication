#!/usr/bin/env python3
import time
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.srv import CommandBool, CommandLong
from mavros_msgs.msg import State

BEST_EFFORT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=10,
)

MODE_NUMBERS = {
    "GUIDED": 4,
    "LAND": 9,
}


class LandOnTarget(Node):
    def __init__(self, tx, ty, alt):
        super().__init__("land_on_target_node")
        self.tx, self.ty, self.alt = float(tx), float(ty), float(alt)
        self.state = State()

        # State + odom
        self.create_subscription(State, "/mavros/state", self._state_cb, 10)
        self.create_subscription(
            Odometry,
            "/mavros/local_position/odom",
            lambda m: None,
            BEST_EFFORT_QOS,
        )

        # Setpoint publisher
        self.sp_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 20
        )

        # Services
        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.cmd_long_cli = self.create_client(CommandLong, "/mavros/cmd/command")

        # Wait for services
        while not self.arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /mavros/cmd/arming ...")
        while not self.cmd_long_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /mavros/cmd/command ...")

    # ----------------- callbacks / helpers -----------------

    def _state_cb(self, msg: State):
        self.state = msg

    def _call_cmd_long_mode(self, name: str) -> bool:
        mode_id = MODE_NUMBERS[name]
        req = CommandLong.Request()
        req.broadcast = False
        req.command = 176      # MAV_CMD_DO_SET_MODE
        req.confirmation = 0
        req.param1 = 1.0       # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        req.param2 = float(mode_id)
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0
        fut = self.cmd_long_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        resp = fut.result()
        ok = bool(resp and resp.success)
        self.get_logger().info(
            f"set_mode({name}) -> success={ok}, result={getattr(resp, 'result', None)}"
        )
        return ok

    def _call_arm(self, value: bool) -> bool:
        req = CommandBool.Request()
        req.value = bool(value)
        fut = self.arm_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        resp = fut.result()
        ok = bool(resp and resp.success)
        self.get_logger().info(
            f"arming({value}) -> success={ok}, result={getattr(resp, 'result', None)}"
        )
        return ok

    def _publish_for(self, pose: PoseStamped, seconds: float):
        t0 = self.get_clock().now().nanoseconds / 1e9
        rate_hz = 20.0
        period = 1.0 / rate_hz
        while (
            rclpy.ok()
            and (self.get_clock().now().nanoseconds / 1e9 - t0) < float(seconds)
        ):
            pose.header.stamp = self.get_clock().now().to_msg()
            self.sp_pub.publish(pose)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    # ----------------- main sequence -----------------

    def run(self):
        # Stream neutral setpoints at takeoff altitude
        neutral = PoseStamped()
        neutral.header.frame_id = "map"
        neutral.pose.position.z = self.alt  # ENU, z up
        self._publish_for(neutral, 2.0)

        # GUIDED + arm
        if not self._call_cmd_long_mode("GUIDED"):
            self.get_logger().error("Failed to set GUIDED, aborting")
            return
        while rclpy.ok() and not self.state.armed:
            if not self._call_arm(True):
                self.get_logger().warn("Arm failed, retrying ...")
            time.sleep(0.5)

        # Go to target (tx, ty, alt)
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.tx
        goal.pose.position.y = self.ty
        goal.pose.position.z = self.alt
        self._publish_for(goal, 6.0)

        # Land
        self._call_cmd_long_mode("LAND")
        self.get_logger().info("Landing command sent.")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--tx", type=float, required=True)
    p.add_argument("--ty", type=float, required=True)
    p.add_argument("--alt", type=float, required=True)
    a = p.parse_args()

    rclpy.init()
    node = LandOnTarget(a.tx, a.ty, a.alt)
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()