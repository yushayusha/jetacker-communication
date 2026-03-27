#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.srv import CommandBool, CommandLong

BEST_EFFORT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# ArduCopter Copter mode numbers for MAV_CMD_DO_SET_MODE (param2)
MODE_NUMBERS = {
    "GUIDED": 4,
    "LAND": 9,
}


class AreaScanNode(Node):
    def __init__(self, alt=10.0, x_max=60.0, y_max=40.0, lane=10.0, speed=3.0):
        super().__init__("area_scan_node")
        self.alt = float(alt)
        self.x_max = float(x_max)
        self.y_max = float(y_max)
        self.lane = float(lane)
        self.speed = float(speed)

        # Publishers / subscribers
        self.pose_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            "/mavros/local_position/odom",
            self._odom_cb,
            BEST_EFFORT_QOS,
        )

        # Services
        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.cmd_long_cli = self.create_client(CommandLong, "/mavros/cmd/command")

        self.current = None          # last Odometry
        self.timer = None
        self.targets = self._build_lawnmower()
        self.target_idx = 0

    # ------------------------ callbacks & helpers ------------------------

    def _odom_cb(self, msg: Odometry):
        self.current = msg

    def _build_lawnmower(self):
        """Build lawn-mower pattern in ENU frame (x east, y north, z up)."""
        pts = []
        y = -self.y_max / 2.0
        x_left, x_right = -self.x_max / 2.0, self.x_max / 2.0
        left_to_right = True

        while y <= self.y_max / 2.0 + 1e-3:
            if left_to_right:
                pts.append((x_left,  y, self.alt))
                pts.append((x_right, y, self.alt))
            else:
                pts.append((x_right, y, self.alt))
                pts.append((x_left,  y, self.alt))
            y += self.lane
            left_to_right = not left_to_right

        self.get_logger().info(f"Lawnmower with {len(pts)} waypoints built")
        return pts

    def _pose_msg(self, x, y, z):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)   # ENU: z up (positive altitude)
        msg.pose.orientation.w = 1.0
        return msg

    def reach(self, goal, tol=1.5):
        if self.current is None:
            return False
        cx = self.current.pose.pose.position.x
        cy = self.current.pose.pose.position.y
        cz = self.current.pose.pose.position.z
        dx = goal[0] - cx
        dy = goal[1] - cy
        dz = goal[2] - cz
        return math.sqrt(dx * dx + dy * dy + dz * dz) < tol

    # ------------------------ main sequence ------------------------

    def start(self):
        # Wait for odometry to start streaming
        self.get_logger().info("Waiting for /mavros/local_position/odom ...")
        while rclpy.ok() and self.current is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Odometry is streaming")

        # Wait for services
        while not self.arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /mavros/cmd/arming ...")
        while not self.cmd_long_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /mavros/cmd/command ...")

        # GUIDED + arm via COMMAND_LONG (no /mavros/set_mode)
        if not self._set_mode("GUIDED"):
            self.get_logger().error("Failed to set GUIDED, aborting")
            return
        if not self._arm(True):
            self.get_logger().error("Failed to arm, aborting")
            return

        # Start 20 Hz setpoint loop
        self.timer = self.create_timer(0.05, self._tick)
        self.get_logger().info("AreaScan started")

    def _tick(self):
        if self.target_idx >= len(self.targets):
            # Scan finished -> LAND
            if self._set_mode("LAND"):
                self.get_logger().info("Scan complete -> LAND")
            else:
                self.get_logger().warn(
                    "Scan complete but failed to set LAND, holding last setpoint"
                )
            self.timer.cancel()
            return

        goal = self.targets[self.target_idx]
        self.pose_pub.publish(self._pose_msg(*goal))

        if self.reach(goal):
            self.get_logger().info(
                f"Reached waypoint {self.target_idx + 1}/{len(self.targets)}"
            )
            self.target_idx += 1

    # ------------------------ MAVROS helpers ------------------------

    def _set_mode(self, name: str) -> bool:
        """Set Copter mode using generic COMMAND_LONG (MAV_CMD_DO_SET_MODE)."""
        mode_id = MODE_NUMBERS[name]
        req = CommandLong.Request()
        req.broadcast = False
        req.command = 176          # MAV_CMD_DO_SET_MODE
        req.confirmation = 0
        req.param1 = 1.0           # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
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

    def _arm(self, value: bool) -> bool:
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


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--alt", type=float, default=10.0)
    parser.add_argument("--x",   type=float, default=60.0)
    parser.add_argument("--y",   type=float, default=40.0)
    parser.add_argument("--lane", type=float, default=10.0)
    parser.add_argument(
        "--speed", type=float, default=3.0,
        help="reserved; SITL uses position hold")
    args = parser.parse_args()

    rclpy.init()
    node = AreaScanNode(args.alt, args.x, args.y, args.lane, args.speed)
    try:
        node.start()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()