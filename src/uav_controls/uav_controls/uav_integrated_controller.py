#!/usr/bin/env python3
"""
uav_integrated_controller_px4_full.py

PX4-compatible (via MAVROS) full-stack UAV controller for Operation Touchdown challenges.
- Uses PX4 OFFBOARD mode + position setpoints (/mavros/setpoint_position/local)
- Communicates directly with UGV via /uav_to_ugv/command (no ground station required)
- Integrates ArUco detections from /aruco/detection
- Optional "lawnmower" area scan for Challenges 2/3 and forwards scan info to UGV

Notes on rules compliance (v0.4):
- Minimum UAV altitude for Challenge 1 flight segment: 4 ft (1.22 m)
- Minimum flight time: 5 seconds
- UGV min speed: 0.2 mph (0.089 m/s)
- UGV may not start moving in Challenge 2/3 until destination is received
- No GPS is used for navigation in this code (local odometry + vision only)
"""

import argparse
import json
import math
import os
import time
import sys
import csv
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.utilities import remove_ros_args
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode

# ==================== Competition Constants ====================

MIN_ALTITUDE_M = 1.22          # 4 ft
MIN_FLIGHT_TIME_S = 5.0
UGV_MIN_SPEED_MS = 0.089       # 0.2 mph
CHALLENGE_1_TIME_LIMIT_S = 7 * 60
CHALLENGE_2_3_TIME_LIMIT_S = 10 * 60  # Rules v0.4: 10 minutes from first UGV movement
CHALLENGE_1_TRAVEL_TIME_S = 30.0
CHALLENGE_2_3_TRAVEL_TIME_S = 10.0  # Rules v0.4: travel together for 10 seconds after landing
OFFBOARD_SETPOINT_HZ = 20.0
PRE_OFFBOARD_STREAM_S = 2.5     # stream setpoints before switching to OFFBOARD


# ==================== Utility ====================

def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Return yaw (rad) from quaternion in ENU frame."""
    # yaw (z axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class ArucoDetection:
    x: float
    y: float
    z: float
    distance: float
    id: int


# ==================== Controller ====================

class UAVIntegratedControllerPX4(Node):
    def __init__(
        self,
        challenge: int = 1,
        aruco_id: int = 0,
        enable_scan: bool = False,
        scan_alt: float = 3.0,
        scan_x: float = 30.0,
        scan_y: float = 20.0,
        scan_lane: float = 5.0,
        takeoff_alt: float = 3.0,
        landing_xy_tolerance: float = 0.6,
        landing_disarm_alt: float = 0.25,
        landing_descent_mps: float = 0.35,
        landing_stable_time_s: float = 0.6,
        log_dir: str = "logs",
    ) -> None:
        super().__init__("uav_integrated_controller_px4_full")

        self.challenge = challenge
        self.aruco_id = aruco_id
        self.enable_scan = enable_scan

        # ===== State =====
        self.state: State = State()
        self.ext_state: ExtendedState = ExtendedState()
        self.uav_odom: Optional[Odometry] = None
        self.ugv_odom: Optional[Odometry] = None

        self.aruco_detected: bool = False
        self.aruco_latest: Optional[ArucoDetection] = None
        self.aruco_location: Optional[Tuple[float, float]] = None  # in local ENU/map frame

        self.scan_complete: bool = False
        self.scanned_areas: List[Tuple[float, float]] = []

        self.uav_start_time: Optional[float] = None
        self.ugv_start_time: Optional[float] = None

        # ===== Logging =====
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)
        self.event_log: List[Dict[str, Any]] = []

        # ----------------- Telemetry CSV (position/velocity/stats) -----------------
        # Purpose: give you a continuous log of key state while the UAV runs (takeoff, scan, loiter, landing).
        # This is separate from event_log.json and is meant for plotting/debugging later.
        self.telemetry_enabled: bool = True
        self.telemetry_rate_hz: float = 5.0  # log rate (Hz)

        self._telemetry_csv_path: str = os.path.join(
            self.log_dir, f"uav_telemetry_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        )
        self._telemetry_csv_fp = None
        self._telemetry_csv_writer = None
        self._telemetry_rows: int = 0
        self._telemetry_last_flush: float = time.time()

        try:
            self._telemetry_csv_fp = open(self._telemetry_csv_path, "w", newline="")
            self._telemetry_csv_writer = csv.writer(self._telemetry_csv_fp)
            self._telemetry_csv_writer.writerow([
                "t_wall_s",
                "ros_time_ns",
                "connected",
                "armed",
                "mode",
                "landed_state",
                "uav_x",
                "uav_y",
                "uav_z",
                "uav_vx",
                "uav_vy",
                "uav_vz",
                "uav_yaw_deg",
                "ugv_x",
                "ugv_y",
                "ugv_z",
                "scan_enabled",
                "scan_progress_cells",
                "scan_total_cells",
                "last_target_source",
                "last_target_x",
                "last_target_y",
                "last_target_z",
            ])
            self._telemetry_csv_fp.flush()
            self.get_logger().info(f"Telemetry CSV logging to: {self._telemetry_csv_path}")
        except Exception as e:
            self.get_logger().error(f"Telemetry CSV disabled (could not open file): {e}")
            self.telemetry_enabled = False

        if self.telemetry_enabled:
            period = 1.0 / max(self.telemetry_rate_hz, 0.5)
            self._telemetry_timer = self.create_timer(period, self._telemetry_tick)
        self._log_event("NODE_START", {
            "challenge": self.challenge,
            "aruco_id": self.aruco_id,
            "enable_scan": self.enable_scan,
        })

        # ===== Scan Plan =====
        self.scan_alt = float(scan_alt)
        self.scan_x = float(scan_x)
        self.scan_y = float(scan_y)
        self.scan_lane = float(scan_lane)
        self.scan_waypoints: List[Tuple[float, float, float]] = self._generate_scan_waypoints()

        # ===== Mission Tunables =====
        self.takeoff_alt = float(takeoff_alt)
        self.landing_xy_tolerance = float(landing_xy_tolerance)
        self.landing_disarm_alt = float(landing_disarm_alt)
        self.landing_descent_mps = float(landing_descent_mps)
        self.landing_stable_time_s = float(landing_stable_time_s)

        # ===== ROS I/O =====
        # MAVROS state
        self.create_subscription(State, "/mavros/state", self._state_cb, 10)
        self.create_subscription(ExtendedState, "/mavros/extended_state", self._ext_state_cb, 10)

        # Local odometry (ENU)
        self.create_subscription(Odometry, "/mavros/local_position/odom", self._uav_odom_cb, 10)

        # UGV odometry (local)
        # - Our internal stack publishes /ugv/odom
        # - The JetAcker Gazebo stack publishes /odom
        # Subscribe to both; whichever is active will drive the latest UGV state.
        self.create_subscription(Odometry, "/ugv/odom", self._ugv_odom_cb, 10)
        self.create_subscription(Odometry, "/odom", self._ugv_odom_cb, 10)

        # ArUco detections
        self.create_subscription(String, "/aruco/detections", self._aruco_cb, 10)
        self.create_subscription(String, "/aruco/detection", self._aruco_cb, 10)

        # Setpoints to PX4 via MAVROS
        self.setpoint_pub = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)

        # Optional map pub (debug/visualization)
        self.map_pub = self.create_publisher(OccupancyGrid, "/uav/scanned_map", 10)

        # Direct comms to UGV (rules: no ground station)
        self.ugv_command_pub = self.create_publisher(String, "/uav_to_ugv/command", 10)
        # JetAcker Gazebo controllers (challenge1/2/3_controller.py) listen on /uav_target_pose.
        # Publishing here lets this UAV controller drive either:
        #   - your JSON-command based UGV stack, or
        #   - the JetAcker Gazebo challenge controllers.
        self.ugv_target_pose_pub = self.create_publisher(PoseStamped, "/uav_target_pose", 10)

        # Services
        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_cli = self.create_client(SetMode, "/mavros/set_mode")

        self.get_logger().info("Waiting for MAVROS services...")
        self._wait_for_service(self.arm_cli, "arming")
        self._wait_for_service(self.set_mode_cli, "set_mode")
        self.get_logger().info("✓ MAVROS services available")

    # ----------------- Callbacks -----------------

    def _state_cb(self, msg: State) -> None:
        self.state = msg

    def _ext_state_cb(self, msg: ExtendedState) -> None:
        self.ext_state = msg

    def _uav_odom_cb(self, msg: Odometry) -> None:
        self.uav_odom = msg

        # Keep a light-weight scan history (position samples) when scanning enabled
        if self.enable_scan and (not self.scan_complete):
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
            if not self.scanned_areas:
                self.scanned_areas.append((x, y))
            else:
                lx, ly = self.scanned_areas[-1]
                if (x - lx) ** 2 + (y - ly) ** 2 > 1.0:  # ~1m spacing
                    self.scanned_areas.append((x, y))

    def _ugv_odom_cb(self, msg: Odometry) -> None:
        self.ugv_odom = msg


def _aruco_cb(self, msg: String) -> None:
    """Handle ArUco detections from either topic:

    - /aruco/detections (preferred): {id,x,y,z,yaw_deg,frame:"body_flu"}
    - /aruco/detection  (legacy):   {id,position:{x,y,z},yaw,frame:"camera_optical"}

    We convert everything to BODY-FLU (x_fwd, y_left, z_up), then estimate the marker's
    XY location in the local ENU/map frame using current UAV odometry + yaw.
    """
    try:
        data = json.loads(msg.data)
    except Exception:
        return
    if not isinstance(data, dict):
        return

    # id
    try:
        det_id = int(data.get("id", -1))
    except Exception:
        return
    if det_id < 0:
        return

    # position (accept flat or nested)
    pos = data.get("position", None)
    if isinstance(pos, dict):
        x_in, y_in, z_in = pos.get("x"), pos.get("y"), pos.get("z")
    else:
        x_in, y_in, z_in = data.get("x"), data.get("y"), data.get("z")

    try:
        x_in = float(x_in)
        y_in = float(y_in)
        z_in = float(z_in)
    except Exception:
        return

    frame = str(data.get("frame", "")).lower().strip() or "camera_optical"

    # Normalize to BODY-FLU meters
    if frame in ("body_flu", "body", "flu"):
        x_fwd, y_left, z_up = x_in, y_in, z_in
    else:
        # Assume OpenCV optical: x=right, y=down, z=forward
        x_fwd = z_in
        y_left = -x_in
        z_up = -y_in

    dist = math.sqrt(x_fwd * x_fwd + y_left * y_left + z_up * z_up)

    # Store latest (relative) observation
    self.aruco_latest = ArucoDetection(x=x_fwd, y=y_left, z=z_up, distance=dist, id=det_id)

    # Only lock onto the configured id (you can change --aruco_id)
    if det_id != int(self.aruco_id):
        return

    # Need UAV pose to convert to map XY
    if self.uav_odom is None:
        return

    q = self.uav_odom.pose.pose.orientation
    yaw = quat_to_yaw(q.x, q.y, q.z, q.w)  # rad

    # Rotate BODY-FLU -> ENU (map) using yaw only
    dx = x_fwd * math.cos(yaw) - y_left * math.sin(yaw)
    dy = x_fwd * math.sin(yaw) + y_left * math.cos(yaw)

    uav_p = self.uav_odom.pose.pose.position
    tgt_x = float(uav_p.x + dx)
    tgt_y = float(uav_p.y + dy)

    self.aruco_location = (tgt_x, tgt_y)
    self.aruco_detected = True

    self._log_event("ARUCO_DETECTED", {
        "id": det_id,
        "rel_body_flu": [x_fwd, y_left, z_up],
        "map_xy": [tgt_x, tgt_y],
        "distance": dist,
    })

# ----------------- Core Helpers -----------------

    def _wait_for_service(self, client, name: str, timeout: float = 15.0) -> None:
        start = time.time()
        while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
            if time.time() - start > timeout:
                raise RuntimeError(f"Timeout waiting for service: {name}")
            self.get_logger().info(f"Waiting for {name}...")

    def _log_event(self, event: str, data: Optional[Dict[str, Any]] = None) -> None:
        payload = {
            "t": time.time(),
            "event": event,
            "data": data or {},
        }
        self.event_log.append(payload)

    def _save_logs(self) -> None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.log_dir, f"uav_log_ch{self.challenge}_{stamp}.json")
        with open(path, "w") as f:
            json.dump(self.event_log, f, indent=2)
        self.get_logger().info(f"Saved UAV log: {path}")

    def _publish_setpoint(self, x: float, y: float, z: float, yaw: Optional[float] = None) -> None:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        # Keep yaw neutral unless you want heading control (can be extended later)
        msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)

    def _stream_setpoint(self, x: float, y: float, z: float, duration_s: float, hz: float = OFFBOARD_SETPOINT_HZ) -> None:
        rate = self.create_rate(hz)
        start = time.time()
        while rclpy.ok() and (time.time() - start) < duration_s:
            self._publish_setpoint(x, y, z)
            rclpy.spin_once(self, timeout_sec=0.0)
            rate.sleep()

    def _distance_to(self, x: float, y: float, z: Optional[float] = None) -> float:
        if self.uav_odom is None:
            return float("inf")
        dx = x - float(self.uav_odom.pose.pose.position.x)
        dy = y - float(self.uav_odom.pose.pose.position.y)
        if z is None:
            return math.hypot(dx, dy)
        dz = z - float(self.uav_odom.pose.pose.position.z)
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _wait_for_altitude(self, target_z: float, tol: float = 0.25, timeout_s: float = 30.0) -> bool:
        start = time.time()
        rate = self.create_rate(10)
        while rclpy.ok() and (time.time() - start) < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.uav_odom is None:
                continue
            z = float(self.uav_odom.pose.pose.position.z)
            if abs(z - target_z) <= tol:
                return True
            rate.sleep()
        return False

    # ----------------- MAVROS / PX4 Helpers -----------------

    def _set_mode(self, mode: str, timeout_s: float = 5.0) -> bool:
        """Set PX4 mode using mavros/set_mode (e.g., OFFBOARD, AUTO.LAND)."""
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = str(mode)

        fut = self.set_mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_s)
        res = fut.result()

        ok = bool(res and res.mode_sent)
        self.get_logger().info(f"set_mode({mode}) -> {ok}")
        if ok:
            self._log_event("SET_MODE", {"mode": mode})
        return ok

    def _arm(self, arm: bool, timeout_s: float = 5.0) -> bool:
        req = CommandBool.Request()
        req.value = bool(arm)
        fut = self.arm_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_s)
        res = fut.result()
        ok = bool(res and res.success)
        self.get_logger().info(f"arm({arm}) -> {ok}")
        if ok:
            self._log_event("ARM", {"armed": arm})
        return ok

    def _enter_offboard_and_arm(self, x: float, y: float, z: float) -> bool:
        """
        PX4 OFFBOARD requirements:
        - publish setpoints before switching to OFFBOARD
        - keep publishing continuously while in OFFBOARD
        """
        self.get_logger().info("Streaming pre-OFFBOARD setpoints...")
        self._stream_setpoint(x, y, z, PRE_OFFBOARD_STREAM_S)

        # Try a few times to set OFFBOARD and arm
        for attempt in range(6):
            if not rclpy.ok():
                return False

            if self.state.mode != "OFFBOARD":
                self._set_mode("OFFBOARD")

            # Keep streaming while mode switches
            self._stream_setpoint(x, y, z, 0.5)

            if not self.state.armed:
                self._arm(True)

            self._stream_setpoint(x, y, z, 0.5)

            # Verify
            if self.state.mode == "OFFBOARD" and self.state.armed:
                self.get_logger().info("✓ OFFBOARD + ARMED")
                return True

            self.get_logger().warn(
                f"Retry OFFBOARD/ARM ({attempt+1}/6). "
                f"mode={self.state.mode} armed={self.state.armed}"
            )
            time.sleep(0.2)

        return False

    # ----------------- UGV Communication -----------------

    def _send_ugv_command(self, command: str, payload: Optional[Dict[str, Any]] = None) -> None:
        msg = String()
        data: Dict[str, Any] = {"command": command}
        if payload:
            data.update(payload)
        msg.data = json.dumps(data)
        self.ugv_command_pub.publish(msg)
        self._log_event("UAV_TO_UGV", {"command": command, "payload": payload or {}})

        # JetAcker Gazebo challenge controllers listen on /uav_target_pose (PoseStamped).
        # Publish an equivalent trigger so your UAV stack can drive the UGV subsystem sim
        # without rewriting their controllers.
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "map"
        if command == "START_MOVING":
            # challenge1_controller interprets:
            #   x = duration (s), y = speed (m/s)
            speed = float((payload or {}).get("speed", 0.6))
            duration = float((payload or {}).get("duration", CHALLENGE_1_TRAVEL_TIME_S))
            ps.pose.position.x = duration
            ps.pose.position.y = speed
            self.ugv_target_pose_pub.publish(ps)
        elif command == "GO_TO_DESTINATION":
            # challenge2_controller interprets:
            #   (x, y) = target (m, m)
            x = (payload or {}).get("x")
            y = (payload or {}).get("y")
            if x is not None and y is not None:
                ps.pose.position.x = float(x)
                ps.pose.position.y = float(y)
                self.ugv_target_pose_pub.publish(ps)

    # ----------------- Scan Planning -----------------

    def _generate_scan_waypoints(self) -> List[Tuple[float, float, float]]:
        """
        Generate lawnmower scan waypoints centered around the origin.
        (You can later shift these based on your actual competition start location.)
        """
        waypoints: List[Tuple[float, float, float]] = []
        x0, y0 = 0.0, 0.0
        x_min, x_max = x0 - self.scan_x / 2.0, x0 + self.scan_x / 2.0
        y_min, y_max = y0 - self.scan_y / 2.0, y0 + self.scan_y / 2.0

        y = y_min
        direction = 1
        while y <= y_max + 1e-6:
            if direction > 0:
                waypoints.append((x_min, y, self.scan_alt))
                waypoints.append((x_max, y, self.scan_alt))
            else:
                waypoints.append((x_max, y, self.scan_alt))
                waypoints.append((x_min, y, self.scan_alt))
            y += self.scan_lane
            direction *= -1
        return waypoints

    def _execute_area_scan(self, time_budget_s: float = 6 * 60) -> None:
        """Fly scan waypoints until ArUco found or budget exceeded."""
        self.get_logger().info("Starting area scan (OFFBOARD position setpoints)...")
        self._log_event("AREA_SCAN_START", {
            "waypoints": len(self.scan_waypoints),
            "scan_alt": self.scan_alt,
        })

        rate = self.create_rate(OFFBOARD_SETPOINT_HZ)
        start = time.time()

        for idx, wp in enumerate(self.scan_waypoints):
            if not rclpy.ok():
                break
            if self.aruco_detected:
                break
            if time.time() - start > time_budget_s:
                self.get_logger().warn("Scan time budget exceeded.")
                break

            self.get_logger().info(f"Scan waypoint {idx+1}/{len(self.scan_waypoints)}: {wp}")

            while rclpy.ok():
                self._publish_setpoint(*wp)

                if self._distance_to(wp[0], wp[1], wp[2]) < 1.5:
                    break
                if self.aruco_detected:
                    break
                if time.time() - start > time_budget_s:
                    break

                rclpy.spin_once(self, timeout_sec=0.0)
                rate.sleep()

        self.scan_complete = True
        self._send_map_to_ugv()
        self._log_event("AREA_SCAN_COMPLETE", {
            "aruco_found": self.aruco_detected,
            "samples": len(self.scanned_areas),
        })

    def _send_map_to_ugv(self) -> None:
        """Send scan data to UGV (nested in 'data' for UGV parser compatibility)."""
        map_data = {
            "scanned_positions": self.scanned_areas,
            "scan_complete": self.scan_complete,
            "total_samples": len(self.scanned_areas),
            "scan_params": {"scan_x": self.scan_x, "scan_y": self.scan_y, "scan_lane": self.scan_lane, "scan_alt": self.scan_alt},
        }
        self._send_ugv_command("MAP_DATA", {"data": map_data})
        self.get_logger().info(f"Sent MAP_DATA to UGV: {len(self.scanned_areas)} samples")

    # ----------------- Mission Primitives -----------------

    def _hold_for_min_flight_time(self, hold_x: float, hold_y: float, hold_z: float) -> None:
        start = time.time()
        rate = self.create_rate(OFFBOARD_SETPOINT_HZ)
        while rclpy.ok() and (time.time() - start) < MIN_FLIGHT_TIME_S:
            self._publish_setpoint(hold_x, hold_y, hold_z)
            rclpy.spin_once(self, timeout_sec=0.0)
            rate.sleep()


def _track_ugv_and_land(self, time_limit_s: float) -> bool:
    """Track the UGV in OFFBOARD and descend for a landing.

    This is a simple controller that:
      1) Follows UGV XY (from /ugv/odom or /odom).
      2) Decreases Z at a controlled rate.
      3) Disarms once we're stable over the target for a short time.

    Why your old version didn't disarm reliably:
      - It required z <= 0.10m, but your UGV pad + model often sits >0.10m above
        the local origin, so the condition never triggered.
    """
    self.get_logger().info("Tracking UGV and descending for landing...")
    self._log_event("LANDING_START", {})

    rate_hz = OFFBOARD_SETPOINT_HZ
    rate = self.create_rate(rate_hz)
    start = time.time()

    xy_tol = float(getattr(self, "landing_xy_tolerance", 0.6))
    disarm_alt = float(getattr(self, "landing_disarm_alt", 0.25))
    descent_mps = float(getattr(self, "landing_descent_mps", 0.35))
    stable_time_s = float(getattr(self, "landing_stable_time_s", 0.6))
    stable_needed = max(1, int(rate_hz * stable_time_s))

    stable_hits = 0

    while rclpy.ok() and (time.time() - start) < float(time_limit_s):
        rclpy.spin_once(self, timeout_sec=0.0)

        if self.uav_odom is None or self.ugv_odom is None:
            rate.sleep()
            continue

        uav_p = self.uav_odom.pose.pose.position
        ugv_p = self.ugv_odom.pose.pose.position

        ugv_x = float(ugv_p.x)
        ugv_y = float(ugv_p.y)

        cur_z = float(uav_p.z)

        # Controlled descent
        dt = 1.0 / rate_hz
        target_z = max(disarm_alt, cur_z - descent_mps * dt)

        # Follow UGV XY and commanded Z
        self._publish_setpoint(ugv_x, ugv_y, target_z)

        d_xy = self._distance_to(ugv_x, ugv_y)

        # Optional extra gate: vertical speed small
        vz = 0.0
        try:
            vz = float(self.uav_odom.twist.twist.linear.z)
        except Exception:
            vz = 0.0

        if d_xy < xy_tol and cur_z <= (disarm_alt + 0.08) and abs(vz) < 0.6:
            stable_hits += 1
        else:
            stable_hits = 0

        if stable_hits >= stable_needed:
            self._log_event("LANDED_ON_UGV_EST", {"d_xy": d_xy, "z": cur_z, "vz": vz})
            self.get_logger().info("✓ Landing condition met. Disarming...")
            # Try direct disarm first
            self._arm(False)

            # Give it a moment to take effect
            t0 = time.time()
            while rclpy.ok() and (time.time() - t0) < 2.0:
                rclpy.spin_once(self, timeout_sec=0.05)
                if not bool(self.state.armed):
                    return True

            # Fallback: switch to AUTO.LAND then try disarm again
            self.get_logger().warn("Disarm did not take immediately; switching to AUTO.LAND and retrying disarm.")
            self._set_mode("AUTO.LAND")
            t1 = time.time()
            while rclpy.ok() and (time.time() - t1) < 6.0:
                rclpy.spin_once(self, timeout_sec=0.1)
                if not bool(self.state.armed):
                    return True
                # Keep nudging above UGV while AUTO.LAND transitions
                self._publish_setpoint(ugv_x, ugv_y, disarm_alt)
                if self.uav_odom is not None and float(self.uav_odom.pose.pose.position.z) <= (disarm_alt + 0.15):
                    self._arm(False)
                rate.sleep()

            return not bool(self.state.armed)

        rate.sleep()

    self.get_logger().warn("Landing timeout. Switching to AUTO.LAND as a failsafe.")
    self._log_event("LANDING_TIMEOUT", {})
    self._set_mode("AUTO.LAND")
    return False

# ----------------- Challenges -----------------

    def challenge_1(self) -> bool:
        """
        Challenge 1 sequence:
        - UAV takeoff from UGV, min alt 4ft, min flight time 5s
        - UGV starts moving after UAV launch
        - UAV lands on moving UGV within 7 minutes
        - Travel together for 10 seconds
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("CHALLENGE 1: UAV launch & land on moving UGV (PX4)")
        self.get_logger().info("=" * 60)

        # Wait for odometry
        if not self._wait_for_odom():
            return False

        self.uav_start_time = time.time()
        self._log_event("UAV_START", {"t": self.uav_start_time})

        # Takeoff target
        takeoff_x, takeoff_y, takeoff_z = 0.0, 0.0, max(float(getattr(self, 'takeoff_alt', 3.0)), MIN_ALTITUDE_M)

        if not self._enter_offboard_and_arm(takeoff_x, takeoff_y, takeoff_z):
            self.get_logger().error("Failed to enter OFFBOARD and arm.")
            return False

        # Climb / stabilize
        self._stream_setpoint(takeoff_x, takeoff_y, takeoff_z, 2.0)
        self._wait_for_altitude(takeoff_z)

        # Start UGV (rules: after UAV launch)
        self.ugv_start_time = time.time()
        self._send_ugv_command("START_MOVING", {"speed": UGV_MIN_SPEED_MS})
        self._log_event("UGV_START_SENT", {"t": self.ugv_start_time})

        # Hold minimum flight time at min altitude
        self._hold_for_min_flight_time(takeoff_x, takeoff_y, takeoff_z)

        # Land on moving UGV
        if not self._track_ugv_and_land(CHALLENGE_1_TIME_LIMIT_S):
            return False

        # Travel together for 10 seconds (hold close to UGV at low altitude)
        travel_start = time.time()
        rate = self.create_rate(OFFBOARD_SETPOINT_HZ)
        while rclpy.ok() and (time.time() - travel_start) < CHALLENGE_1_TRAVEL_TIME_S:
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.ugv_odom is not None:
                self._publish_setpoint(
                    float(self.ugv_odom.pose.pose.position.x),
                    float(self.ugv_odom.pose.pose.position.y),
                    0.1,
                )
            rate.sleep()

        self._log_event("CHALLENGE_1_COMPLETE", {})
        self.get_logger().info("✓ CHALLENGE 1 COMPLETE")
        return True

    def challenge_2(self) -> bool:
        """
        Challenge 2:
        - UAV takes off and searches for ArUco marker
        - UAV communicates destination (within 5ft accuracy requirement is a system-level goal)
        - UGV starts moving only after destination received (UGV enforces)
        - UAV lands on moving UGV within 15 minutes total
        - Travel together for 10 seconds
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("CHALLENGE 2: ArUco detection + navigation (PX4)")
        self.get_logger().info("=" * 60)

        if not self._wait_for_odom():
            return False

        search_z = max(self.scan_alt if self.enable_scan else 3.0, MIN_ALTITUDE_M)

        self.uav_start_time = time.time()
        self._log_event("UAV_START", {"t": self.uav_start_time})

        if not self._enter_offboard_and_arm(0.0, 0.0, search_z):
            self.get_logger().error("Failed to enter OFFBOARD and arm.")
            return False

        self._stream_setpoint(0.0, 0.0, search_z, 2.0)
        self._wait_for_altitude(search_z)

        # Minimum flight time
        self._hold_for_min_flight_time(0.0, 0.0, search_z)

        # Search for ArUco
        if self.enable_scan:
            self._execute_area_scan()
        else:
            # Simple square search
            pattern = [
                (5, 0, search_z), (5, 5, search_z), (0, 5, search_z),
                (-5, 5, search_z), (-5, 0, search_z), (-5, -5, search_z),
                (0, -5, search_z), (5, -5, search_z),
            ]
            for wp in pattern:
                if self.aruco_detected:
                    break
                self._stream_setpoint(*wp, duration_s=3.0)

        if not self.aruco_detected or self.aruco_location is None:
            self.get_logger().error("✗ ArUco not found.")
            self._log_event("ARUCO_NOT_FOUND", {})
            return False

        # Send destination to UGV (rules: UGV starts after this)
        self.ugv_start_time = time.time()
        self._send_ugv_command("GO_TO_DESTINATION", {
            "x": float(self.aruco_location[0]),
            "y": float(self.aruco_location[1]),
            "speed": UGV_MIN_SPEED_MS,
        })
        self._log_event("DEST_SENT_TO_UGV", {"t": self.ugv_start_time, "dest": self.aruco_location})

        # Land on moving UGV (overall time limit)
        elapsed = time.time() - self.uav_start_time
        remaining = max(10.0, CHALLENGE_2_3_TIME_LIMIT_S - elapsed)
        if not self._track_ugv_and_land(remaining):
            return False

        # Travel together for 10 seconds
        travel_start = time.time()
        rate = self.create_rate(OFFBOARD_SETPOINT_HZ)
        while rclpy.ok() and (time.time() - travel_start) < CHALLENGE_2_3_TRAVEL_TIME_S:
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.ugv_odom is not None:
                self._publish_setpoint(
                    float(self.ugv_odom.pose.pose.position.x),
                    float(self.ugv_odom.pose.pose.position.y),
                    0.1,
                )
            rate.sleep()

        self._log_event("CHALLENGE_2_COMPLETE", {})
        self.get_logger().info("✓ CHALLENGE 2 COMPLETE")
        return True

    def challenge_3(self) -> bool:
        """Challenge 3 = Challenge 2 + obstacle avoidance for UGV."""
        self._send_ugv_command("ENABLE_OBSTACLE_AVOIDANCE", {"enabled": True})
        self._log_event("OBSTACLE_AVOIDANCE_ENABLED", {"enabled": True})
        return self.challenge_2()


    # ----------------- Telemetry helpers -----------------

    def _telemetry_tick(self) -> None:
        # Never let telemetry logging crash the mission.
        try:
            if not self.telemetry_enabled or self._telemetry_csv_writer is None:
                return

            now_ros = self.get_clock().now().nanoseconds
            t_wall = time.time()

            connected = bool(getattr(self, 'state', None).connected) if getattr(self, 'state', None) else False
            armed = bool(getattr(self, 'state', None).armed) if getattr(self, 'state', None) else False
            mode = str(getattr(self, 'state', None).mode) if getattr(self, 'state', None) else ""
            landed_state = int(getattr(self, 'ext_state', None).landed_state) if getattr(self, 'ext_state', None) else -1

            # UAV pose/vel from MAVROS odom (preferred: includes twist)
            uav_x = uav_y = uav_z = float("nan")
            uav_vx = uav_vy = uav_vz = float("nan")
            uav_yaw_deg = float("nan")
            if self.uav_odom is not None:
                p = self.uav_odom.pose.pose.position
                uav_x, uav_y, uav_z = p.x, p.y, p.z

                tw = self.uav_odom.twist.twist
                uav_vx, uav_vy, uav_vz = tw.linear.x, tw.linear.y, tw.linear.z

                # yaw from quaternion
                q = self.uav_odom.pose.pose.orientation
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                uav_yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))

            # UGV pose (if available)
            ugv_x = ugv_y = ugv_z = float("nan")
            if getattr(self, "ugv_odom", None) is not None:
                up = self.ugv_odom.pose.pose.position
                ugv_x, ugv_y, ugv_z = up.x, up.y, up.z

            # Scan progress
            scan_enabled = bool(self.enable_scan)
            scan_progress_cells = int(getattr(self, "scanned_cells", 0))
            scan_total_cells = int(getattr(self, "total_cells", 0))

            # Most recent target estimate (if your pipeline provides it)
            last_source = ""
            last_tx = last_ty = last_tz = float("nan")
            if hasattr(self, "last_target_pose") and self.last_target_pose is not None:
                last_source = getattr(self, "last_target_source", "unknown")
                tp = self.last_target_pose
                last_tx, last_ty, last_tz = tp["x"], tp["y"], tp["z"]

            self._telemetry_csv_writer.writerow([
                f"{t_wall:.6f}",
                str(now_ros),
                int(connected),
                int(armed),
                mode,
                str(landed_state),
                f"{uav_x:.3f}" if math.isfinite(uav_x) else "",
                f"{uav_y:.3f}" if math.isfinite(uav_y) else "",
                f"{uav_z:.3f}" if math.isfinite(uav_z) else "",
                f"{uav_vx:.3f}" if math.isfinite(uav_vx) else "",
                f"{uav_vy:.3f}" if math.isfinite(uav_vy) else "",
                f"{uav_vz:.3f}" if math.isfinite(uav_vz) else "",
                f"{uav_yaw_deg:.2f}" if math.isfinite(uav_yaw_deg) else "",
                f"{ugv_x:.3f}" if math.isfinite(ugv_x) else "",
                f"{ugv_y:.3f}" if math.isfinite(ugv_y) else "",
                f"{ugv_z:.3f}" if math.isfinite(ugv_z) else "",
                int(scan_enabled),
                scan_progress_cells,
                scan_total_cells,
                last_source,
                f"{last_tx:.3f}" if math.isfinite(last_tx) else "",
                f"{last_ty:.3f}" if math.isfinite(last_ty) else "",
                f"{last_tz:.3f}" if math.isfinite(last_tz) else "",
            ])
            self._telemetry_rows += 1

            # Flush periodically so you don't lose everything on a crash
            if (time.time() - self._telemetry_last_flush) > 2.0:
                self._telemetry_csv_fp.flush()
                self._telemetry_last_flush = time.time()

        except Exception as e:
            # Avoid log spam: only warn occasionally
            try:
                self.get_logger().warn(f"Telemetry tick error (non-fatal): {e}")
            except Exception:
                pass

    def destroy_node(self) -> bool:
        # Ensure files are closed cleanly.
        try:
            if getattr(self, "_telemetry_csv_fp", None) is not None:
                try:
                    self._telemetry_csv_fp.flush()
                except Exception:
                    pass
                try:
                    self._telemetry_csv_fp.close()
                except Exception:
                    pass
        except Exception:
            pass
        return super().destroy_node()

    # ----------------- Startup / Wait helpers -----------------

    def _wait_for_odom(self, timeout_s: float = 15.0) -> bool:
        start = time.time()
        while rclpy.ok() and (time.time() - start) < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.uav_odom is not None:
                return True
        self.get_logger().error("No UAV odometry received (is MAVROS running?).")
        return False

    # ----------------- Main -----------------

    def run(self) -> bool:
        try:
            if self.challenge == 1:
                ok = self.challenge_1()
            elif self.challenge == 2:
                ok = self.challenge_2()
            elif self.challenge == 3:
                ok = self.challenge_3()
            else:
                self.get_logger().error(f"Invalid challenge: {self.challenge}")
                ok = False

            self._save_logs()
            return ok

        except Exception as e:
            self.get_logger().error(f"Unhandled exception: {e}")
            import traceback
            traceback.print_exc()
            self._log_event("EXCEPTION", {"error": str(e)})
            self._save_logs()
            return False


def main() -> None:
    parser = argparse.ArgumentParser(description="UAV Integrated Controller (PX4 OFFBOARD via MAVROS) - FULL")
    parser.add_argument("--challenge", type=int, default=1, choices=[1, 2, 3])
    parser.add_argument("--aruco_id", type=int, default=0)
    parser.add_argument("--scan", action="store_true", help="Enable lawnmower scan for Challenges 2 & 3")
    parser.add_argument("--scan_alt", type=float, default=3.0)
    parser.add_argument("--scan_x", type=float, default=30.0)
    parser.add_argument("--scan_y", type=float, default=20.0)
    parser.add_argument("--scan_lane", type=float, default=5.0)
    parser.add_argument("--takeoff_alt", type=float, default=3.0, help="Takeoff altitude in meters (challenge1 uses this instead of MIN_ALTITUDE)")
    parser.add_argument("--landing_xy_tol", type=float, default=0.6)
    parser.add_argument("--landing_disarm_alt", type=float, default=0.25)
    parser.add_argument("--landing_descent_mps", type=float, default=0.35)
    parser.add_argument("--landing_stable_time", type=float, default=0.6)
    parser.add_argument("--log_dir", type=str, default="logs")
    # Parse ONLY non-ROS args so this node can still be launched with --ros-args remaps
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = UAVIntegratedControllerPX4(
        challenge=args.challenge,
        aruco_id=args.aruco_id,
        enable_scan=args.scan,
        scan_alt=args.scan_alt,
        scan_x=args.scan_x,
        scan_y=args.scan_y,
        scan_lane=args.scan_lane,
        takeoff_alt=args.takeoff_alt,
        landing_xy_tolerance=args.landing_xy_tol,
        landing_disarm_alt=args.landing_disarm_alt,
        landing_descent_mps=args.landing_descent_mps,
        landing_stable_time_s=args.landing_stable_time,
        log_dir=args.log_dir,
    )

    try:
        ok = node.run()
        if ok:
            node.get_logger().info("✓✓✓ MISSION SUCCESS ✓✓✓")
        else:
            node.get_logger().warn("Mission ended without success.")
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()