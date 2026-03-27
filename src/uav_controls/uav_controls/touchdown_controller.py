#!/usr/bin/env python3
"""
touchdown_controller_px4_competition_v2.py

Mission-level controller for Operation Touchdown (PX4 + MAVROS + ROS2).

Goals (what this script is designed to do)
1) Take off safely using PX4 OFFBOARD.
2) Execute an "area scan" (lawnmower) in the challenge bounds.
3) Build a simple map *from the scan path* (placeholder mapping in sim) and publish it.
4) Detect ArUco markers (via aruco_detector.py JSON output) and compute target location.
5) Publish the target location to the UGV subsystem (/uav_target_pose).
6) Rendezvous and attempt a precision landing on the UGV landing pad.
7) Handle errors/timeouts and fail safe to AUTO.LAND when needed.

IMPORTANT reality check (why things “work separately but not together”)
- If your UGV launch spawns a *decorative* UAV inside the JetAcker URDF,
  PX4 will NOT control that decorative UAV. PX4 controls the PX4 x500 model.
  If those are in different Gazebo instances (or two different UAV models),
  then “UAV doesn’t land on UGV” is expected.
- You must ensure ONE Gazebo world contains BOTH:
  * the UGV model you see driving, AND
  * the PX4-controlled UAV model (x500)
  and that MAVROS is connected to that PX4 instance.

Run order (recommended)
Terminal A: Start ONE Gazebo world that includes BOTH vehicles.
Terminal B: Start PX4 SITL (if not already started by the world launcher).
Terminal C: Start MAVROS and confirm /mavros/state connected.
Terminal D: Start vision (aruco_detector.py) and UGV nodes.
Terminal E: Start THIS node.

This node does NOT require custom message definitions.

"""

from __future__ import annotations

import csv
import json
import math
import os
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String, Bool

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


# ---------------------------
# Helpers
# ---------------------------

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def quat_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw/2.0), w=math.cos(yaw/2.0))


@dataclass
class ArucoDetection:
    det_id: int
    x_fwd: float
    y_left: float
    z_up: float
    yaw_deg: float
    stamp_sec: float


# ---------------------------
# Mission Node
# ---------------------------

class TouchdownControllerPX4(Node):
    def __init__(self) -> None:
        super().__init__('touchdown_controller_px4')

        # -------- Parameters (safe defaults; tune per your field) --------
        self.declare_parameter('setpoint_rate_hz', 20.0)

        # Takeoff / scan
        self.declare_parameter('takeoff_alt_m', 3.0)           # ~10 ft
        self.declare_parameter('hover_alt_m', 3.0)
        self.declare_parameter('scan_alt_m', 3.0)
        self.declare_parameter('scan_width_m', 20.0)
        self.declare_parameter('scan_height_m', 20.0)
        self.declare_parameter('lane_spacing_m', 4.0)
        self.declare_parameter('waypoint_accept_m', 0.7)
        self.declare_parameter('scan_timeout_s', 180.0)

        # Target + landing
        self.declare_parameter('target_marker_id', 0)          # change to your rules/marker ID
        self.declare_parameter('landing_marker_id', 0)         # marker on UGV pad
        self.declare_parameter('max_aruco_age_s', 0.5)

        self.declare_parameter('align_kp', 0.6)                # horizontal correction gain
        self.declare_parameter('align_max_step_m', 0.5)        # limit per update
        self.declare_parameter('align_accept_m', 0.25)         # must be within this to descend
        self.declare_parameter('descend_rate_mps', 0.25)       # controlled descent in OFFBOARD
        self.declare_parameter('final_land_alt_m', 0.35)       # switch to AUTO.LAND near this altitude

        # Topics
        self.declare_parameter('ugv_odom_topic', '/odom')
        self.declare_parameter('aruco_topic', '/aruco/detections')
        self.declare_parameter('aruco_topic_legacy', '/aruco/detection')
        self.declare_parameter('kill_switch_topic', '/kill_switch')

        # “Bridge” topics expected by UGV subsystem
        self.declare_parameter('uav_target_pose_topic', '/uav_target_pose')
        self.declare_parameter('uav_mission_plan_topic', '/uav_mission_plan')
        self.declare_parameter('uav_map_topic', '/uav/map')
        self.declare_parameter('uav_telemetry_topic', '/uav/telemetry')

        # Mapping
        self.declare_parameter('map_resolution_m', 1.0)
        self.declare_parameter('map_margin_m', 5.0)

        # Logging
        self.declare_parameter('log_dir', os.getcwd())
        self.declare_parameter('telemetry_rate_hz', 10.0)  # CSV telemetry sampling rate

        # -------- Read params --------
        self.sp_rate = float(self.get_parameter('setpoint_rate_hz').value)

        self.takeoff_alt = float(self.get_parameter('takeoff_alt_m').value)
        self.hover_alt = float(self.get_parameter('hover_alt_m').value)
        self.scan_alt = float(self.get_parameter('scan_alt_m').value)
        self.scan_w = float(self.get_parameter('scan_width_m').value)
        self.scan_h = float(self.get_parameter('scan_height_m').value)
        self.lane = float(self.get_parameter('lane_spacing_m').value)
        self.wp_accept = float(self.get_parameter('waypoint_accept_m').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout_s').value)

        self.target_id = int(self.get_parameter('target_marker_id').value)
        self.landing_id = int(self.get_parameter('landing_marker_id').value)
        self.max_aruco_age = float(self.get_parameter('max_aruco_age_s').value)

        self.align_kp = float(self.get_parameter('align_kp').value)
        self.align_max_step = float(self.get_parameter('align_max_step_m').value)
        self.align_accept = float(self.get_parameter('align_accept_m').value)
        self.descend_rate = float(self.get_parameter('descend_rate_mps').value)
        self.final_land_alt = float(self.get_parameter('final_land_alt_m').value)

        self.ugv_odom_topic = str(self.get_parameter('ugv_odom_topic').value)
        self.aruco_topic = str(self.get_parameter('aruco_topic').value)
        self.aruco_topic_legacy = str(self.get_parameter('aruco_topic_legacy').value)
        self.kill_topic = str(self.get_parameter('kill_switch_topic').value)

        self.topic_target_pose = str(self.get_parameter('uav_target_pose_topic').value)
        self.topic_plan = str(self.get_parameter('uav_mission_plan_topic').value)
        self.topic_map = str(self.get_parameter('uav_map_topic').value)
        self.topic_tel = str(self.get_parameter('uav_telemetry_topic').value)

        self.map_res = float(self.get_parameter('map_resolution_m').value)
        self.map_margin = float(self.get_parameter('map_margin_m').value)
        self.log_dir = str(self.get_parameter('log_dir').value)

        # -------- QoS --------
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE

        # -------- MAVROS I/O --------
        self.mav_state: State = State()
        self.local_pose: Optional[PoseStamped] = None
        self.local_odom: Optional[Odometry] = None

        self.create_subscription(State, '/mavros/state', self._on_mav_state, qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._on_local_pose, qos)
        self.create_subscription(Odometry, '/mavros/local_position/odom', self._on_local_odom, qos)

        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos)
        self.arm_srv = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_srv = self.create_client(SetMode, '/mavros/set_mode')

        # -------- External data --------
        self.ugv_odom: Optional[Odometry] = None
        self.aruco_last: Dict[int, ArucoDetection] = {}
        self.killed = False

        self.create_subscription(Odometry, self.ugv_odom_topic, self._on_ugv_odom, qos)
        self.create_subscription(String, self.aruco_topic, self._on_aruco, 10)
        self.create_subscription(String, self.aruco_topic_legacy, self._on_aruco, 10)
        self.create_subscription(Bool, self.kill_topic, self._on_kill, qos)

        # -------- Bridge outputs --------
        self.pub_target = self.create_publisher(PoseStamped, self.topic_target_pose, qos)
        self.pub_plan = self.create_publisher(PoseArray, self.topic_plan, qos)
        self.pub_map = self.create_publisher(OccupancyGrid, self.topic_map, qos)
        self.pub_tel = self.create_publisher(String, self.topic_tel, qos)

        # -------- Internal setpoint streaming --------
        self.setpoint: PoseStamped = PoseStamped()
        self.have_setpoint = False
        self.sp_timer = self.create_timer(1.0 / max(self.sp_rate, 1.0), self._publish_setpoint)

        # -------- Mission timers --------
        self.step_timer = self.create_timer(0.2, self._step_mission)  # 5 Hz mission step
        self.telemetry_rate_hz = float(self.get_parameter('telemetry_rate_hz').value)
        self._telemetry_timer = self.create_timer(1.0 / max(self.telemetry_rate_hz, 1.0), self._telemetry_tick)

        # -------- Mission state --------
        self.state = 'WAIT_FCU'
        self._csv_log(event='transition:WAIT_FCU')
        self.state_enter = time.time()
        self.takeoff_origin: Optional[Tuple[float, float, float]] = None

        self.scan_waypoints: List[Tuple[float, float, float]] = []
        self.scan_idx = 0
        self.scan_start_time = None
        self.target_found = False
        self.target_global_xy: Optional[Tuple[float, float]] = None

        # Landing state
        self.descend_target_z: Optional[float] = None

        # -------- Logging --------
        os.makedirs(self.log_dir, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        self.log_path = os.path.join(self.log_dir, f'uav_touchdown_log_{ts}.csv')

        # ---- Telemetry CSV logger ----
        os.makedirs(self.log_dir, exist_ok=True)
        self._csv_fp = None
        self._csv_writer = None
        try:
            is_new = (not os.path.exists(self.log_path)) or (os.path.getsize(self.log_path) == 0)
            self._csv_fp = open(self.log_path, 'a', newline='')
            self._csv_writer = csv.writer(self._csv_fp)
            if is_new:
                self._csv_writer.writerow([
                    't_utc', 'event',
                    'armed', 'mode', 'connected',
                    'uav_x', 'uav_y', 'uav_z', 'uav_vx', 'uav_vy', 'uav_vz',
                    'aruco_seen', 'aruco_id', 'rel_x', 'rel_y', 'rel_z',
                    'target_err_xy', 'target_err_z', 'descent_vz_cmd'
                ])
                self._csv_fp.flush()
        except Exception as e:
            self.get_logger().warning(f'Could not open touchdown telemetry CSV at {self.log_path}: {e}')

        with open(self.log_path, 'w', encoding='utf-8') as f:
            f.write('t,state,mode,armed,uav_x,uav_y,uav_z,uav_vx,uav_vy,uav_vz,uav_yaw_deg,sp_x,sp_y,sp_z,err_sp_xy,err_sp_z,ugv_x,ugv_y,ugv_z,err_ugv_xy,err_ugv_z,aruco_id,aruco_x_fwd,aruco_y_left\n')

        # -------- Map (simple “coverage map”) --------
        self._map: Optional[OccupancyGrid] = None
        self._map_init_done = False

        self.get_logger().info(
            f'TouchdownControllerPX4 ready. '
            f'takeoff_alt={self.takeoff_alt}m scan={self.scan_w}x{self.scan_h}m lane={self.lane}m '
            f'aruco_topic={self.aruco_topic} ugv_odom={self.ugv_odom_topic} log={self.log_path}'
        )

    # --------------------------
    # Callbacks
    # --------------------------
    def _on_mav_state(self, msg: State) -> None:
        self.mav_state = msg

    def _on_local_pose(self, msg: PoseStamped) -> None:
        self.local_pose = msg

    def _on_local_odom(self, msg: Odometry) -> None:
        self.local_odom = msg

    def _on_ugv_odom(self, msg: Odometry) -> None:
        self.ugv_odom = msg

    def _on_kill(self, msg: Bool) -> None:
        if msg.data and not self.killed:
            self.killed = True
            self.get_logger().error('KILL SWITCH received -> switching to AUTO.LAND')
            self._try_set_mode('AUTO.LAND')

    def _on_aruco(self, msg: String) -> None:
        """Parse ArUco detections.

        Supports two JSON payload styles:
          1) Flat: {id: int, x: float, y: float, z: float, yaw: deg}
          2) Nested: {id: int, position: {x,y,z}, yaw: deg}

        And two input frames (param aruco_input_frame):
          - 'camera_optical' (OpenCV): x=right, y=down, z=forward
          - 'body_flu': x=forward, y=left, z=up
        """
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Bad ArUco JSON (decode): {e}")
            return
        if not isinstance(data, dict):
            return

        try:
            det_id = int(data.get("id", -1))
        except Exception:
            det_id = -1
        if det_id < 0:
            return

        # Extract raw vector
        pos = data.get("position", None)
        if isinstance(pos, dict):
            x_in = pos.get("x", None)
            y_in = pos.get("y", None)
            z_in = pos.get("z", None)
        else:
            x_in = data.get("x", None)
            y_in = data.get("y", None)
            z_in = data.get("z", None)

        try:
            x_in = float(x_in)
            y_in = float(y_in)
            z_in = float(z_in)
        except Exception:
            self.get_logger().warn("Bad ArUco JSON (missing/invalid x,y,z)")
            return

        # Yaw: accept 'yaw_deg', 'yaw' (deg), or 'yaw_rad' (rad)
        yaw_val = data.get("yaw_deg", None)
        if yaw_val is None:
            yaw_val = data.get("yaw", None)
        if yaw_val is None:
            yaw_val = data.get("yaw_rad", 0.0)
        try:
            yaw_val = float(yaw_val)
        except Exception:
            yaw_val = 0.0

        # If the key was yaw_rad or value looks like radians, convert to degrees
        if "yaw_rad" in data or (abs(yaw_val) <= math.pi + 0.2 and "yaw_deg" not in data and "yaw" not in data):
            yaw_deg = float(yaw_val * 180.0 / math.pi)
        else:
            yaw_deg = float(yaw_val)

        frame = str(getattr(self, "aruco_input_frame", "camera_optical")).lower().strip()
        if frame == "camera_optical":
            # OpenCV optical frame: x=right, y=down, z=forward
            x_fwd = z_in
            y_left = -x_in
            z_up = -y_in
        elif frame == "body_flu":
            x_fwd, y_left, z_up = x_in, y_in, z_in
        else:
            self.get_logger().warn(f"Unknown aruco_input_frame='{frame}', assuming body_flu")
            x_fwd, y_left, z_up = x_in, y_in, z_in

        det = ArucoDetection(
            det_id=det_id,
            x_fwd=float(x_fwd),
            y_left=float(y_left),
            z_up=float(z_up),
            yaw_deg=float(yaw_deg),
            stamp_sec=time.time(),
        )
        self.aruco_last[det.det_id] = det
    # --------------------------
    # MAVROS helpers
    # --------------------------
    def _publish_setpoint(self) -> None:
        if not self.have_setpoint:
            return
        self.setpoint.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_pub.publish(self.setpoint)

    def _set_sp(self, x: float, y: float, z: float, yaw: Optional[float] = None) -> None:
        if yaw is None and self.local_pose is not None:
            yaw = quat_to_yaw(self.local_pose.pose.orientation)
        if yaw is None:
            yaw = 0.0
        sp = PoseStamped()
        sp.header.frame_id = 'map'
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.pose.position.x = float(x)
        sp.pose.position.y = float(y)
        sp.pose.position.z = float(z)
        sp.pose.orientation = yaw_to_quat(float(yaw))
        self.setpoint = sp
        self.have_setpoint = True

    def _try_arm(self, arm: bool = True, timeout: float = 2.0) -> bool:
        if not self.arm_srv.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('arming service not available')
            return False
        req = CommandBool.Request()
        req.value = bool(arm)
        fut = self.arm_srv.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None or not res.success:
            self.get_logger().warn('arm/disarm rejected')
            return False
        return True

    def _try_set_mode(self, mode: str, timeout: float = 2.0) -> bool:
        if not self.mode_srv.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('set_mode service not available')
            return False
        req = SetMode.Request()
        req.custom_mode = str(mode)
        fut = self.mode_srv.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None or not res.mode_sent:
            self.get_logger().warn(f'mode {mode} rejected')
            return False
        return True

    # --------------------------
    # Data helpers
    # --------------------------
    def _uav_xyz(self) -> Optional[Tuple[float, float, float]]:
        if self.local_pose is None:
            return None
        p = self.local_pose.pose.position
        return (float(p.x), float(p.y), float(p.z))


    def _uav_vel(self) -> Optional[Tuple[float, float, float]]:
        if self.local_odom is None:
            return None
        v = self.local_odom.twist.twist.linear
        return (float(v.x), float(v.y), float(v.z))

    def _uav_yaw(self) -> Optional[float]:
        if self.local_pose is None:
            return None
        return quat_to_yaw(self.local_pose.pose.orientation)

    def _ugv_xyz(self) -> Optional[Tuple[float, float, float]]:
        if self.ugv_odom is None:
            return None
        p = self.ugv_odom.pose.pose.position
        return (float(p.x), float(p.y), float(p.z))

    def _get_aruco(self, det_id: int) -> Optional[ArucoDetection]:
        det = self.aruco_last.get(det_id)
        if det is None:
            return None
        if (time.time() - det.stamp_sec) > self.max_aruco_age:
            return None
        return det

    def _body_flu_to_enu(self, dx_fwd: float, dy_left: float) -> Optional[Tuple[float, float]]:
        yaw = self._uav_yaw()
        if yaw is None:
            return None
        de = math.cos(yaw) * dx_fwd + (-math.sin(yaw)) * dy_left
        dn = math.sin(yaw) * dx_fwd + ( math.cos(yaw)) * dy_left
        return (de, dn)

    # --------------------------
    # Mapping (placeholder coverage grid)
    # --------------------------
    def _init_map(self, origin_xy: Tuple[float, float]) -> None:
        w = self.scan_w + 2.0 * self.map_margin
        h = self.scan_h + 2.0 * self.map_margin
        res = self.map_res
        width = int(math.ceil(w / res))
        height = int(math.ceil(h / res))

        origin_x = origin_xy[0] - w / 2.0
        origin_y = origin_xy[1] - h / 2.0

        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution = float(res)
        grid.info.width = int(width)
        grid.info.height = int(height)
        grid.info.origin.position.x = float(origin_x)
        grid.info.origin.position.y = float(origin_y)
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation = yaw_to_quat(0.0)
        grid.data = [-1] * (width * height)  # unknown

        self._map = grid
        self._map_init_done = True

    def _map_mark_visited(self, x: float, y: float) -> None:
        if not self._map_init_done or self._map is None:
            return
        gx = int((x - self._map.info.origin.position.x) / self._map.info.resolution)
        gy = int((y - self._map.info.origin.position.y) / self._map.info.resolution)
        if gx < 0 or gy < 0 or gx >= self._map.info.width or gy >= self._map.info.height:
            return
        idx = gy * self._map.info.width + gx
        self._map.data[idx] = 0  # free/visited

    def _publish_map(self) -> None:
        if self._map is None:
            return
        self._map.header.stamp = self.get_clock().now().to_msg()
        self.pub_map.publish(self._map)

    # --------------------------
    # Mission core
    # --------------------------
    def _enter(self, new_state: str) -> None:
        self.state = new_state
        self.state_enter = time.time()
        self.get_logger().info(f'--> {new_state}')

    def _log(self) -> None:
        # One-line CSV log for post-flight analysis (safe even if inputs aren't ready yet).
        t = time.time()

        uav = self._uav_xyz()
        uav_v = self._uav_vel()

        yaw_deg = float('nan')
        try:
            yaw_deg = math.degrees(self._uav_yaw())
        except Exception:
            pass

        sp_x = sp_y = sp_z = float('nan')
        err_sp_xy = err_sp_z = float('nan')
        if self.setpoint is not None:
            sp_x = float(self.setpoint.pose.position.x)
            sp_y = float(self.setpoint.pose.position.y)
            sp_z = float(self.setpoint.pose.position.z)

        if uav is not None and not math.isnan(sp_x):
            err_sp_xy = math.hypot(uav[0] - sp_x, uav[1] - sp_y)
            err_sp_z = uav[2] - sp_z

        ugv = self._ugv_xyz()
        err_ugv_xy = err_ugv_z = float('nan')
        if uav is not None and ugv is not None:
            err_ugv_xy = math.hypot(uav[0] - ugv[0], uav[1] - ugv[1])
            err_ugv_z = uav[2] - ugv[2]

        det_id = -1
        det_x = float('nan')
        det_y = float('nan')
        if self.last_det is not None:
            det_id = int(self.last_det.marker_id)
            det_x = float(self.last_det.x_fwd)
            det_y = float(self.last_det.y_left)

        try:
            uav_x = uav[0] if uav else float('nan')
            uav_y = uav[1] if uav else float('nan')
            uav_z = uav[2] if uav else float('nan')
            uav_vx = uav_v[0] if uav_v else float('nan')
            uav_vy = uav_v[1] if uav_v else float('nan')
            uav_vz = uav_v[2] if uav_v else float('nan')

            ugv_x = ugv[0] if ugv else float('nan')
            ugv_y = ugv[1] if ugv else float('nan')
            ugv_z = ugv[2] if ugv else float('nan')

            row = (
                f"{t:.3f},{self.state},{self._flight_mode()},{int(self.armed)},"
                f"{uav_x:.3f},{uav_y:.3f},{uav_z:.3f},{uav_vx:.3f},{uav_vy:.3f},{uav_vz:.3f},{yaw_deg:.2f},"
                f"{sp_x:.3f},{sp_y:.3f},{sp_z:.3f},{err_sp_xy:.3f},{err_sp_z:.3f},"
                f"{ugv_x:.3f},{ugv_y:.3f},{ugv_z:.3f},{err_ugv_xy:.3f},{err_ugv_z:.3f},"
                f"{det_id},{det_x:.3f},{det_y:.3f}"
            )

            with open(self.log_path, "a") as f:
                f.write(row + "\n")
        except Exception:
            # Don't let logging break flight logic.
            pass

    def _step_mission(self) -> None:
        if self.killed:
            self._log()
            return

        if self.local_pose is None:
            self._log()
            return

        xyz = self._uav_xyz()
        if xyz is None:
            self._log()
            return

        # small telemetry
        if int(time.time() * 2) % 5 == 0:
            self.pub_tel.publish(String(data=f'state={self.state} mode={self.mav_state.mode} armed={self.mav_state.armed}'))

        # update map coverage
        if self._map_init_done:
            self._map_mark_visited(xyz[0], xyz[1])

        if self.state == 'WAIT_FCU':
            if self.mav_state.connected:
                self._set_sp(xyz[0], xyz[1], max(xyz[2], 0.3), yaw=self._uav_yaw())
                if not self._map_init_done:
                    self._init_map((xyz[0], xyz[1]))
                self._enter('STREAM_SETPOINTS')
            else:
                if (time.time() - self.state_enter) > 5.0:
                    self.get_logger().warn('Waiting for FCU connection... (check MAVROS fcu_url / PX4 ports)')
                    self.state_enter = time.time()

        elif self.state == 'STREAM_SETPOINTS':
            if (time.time() - self.state_enter) > 2.0:
                self._enter('SET_OFFBOARD_AND_ARM')

        elif self.state == 'SET_OFFBOARD_AND_ARM':
            ok_mode = self._try_set_mode('OFFBOARD')
            ok_arm = self._try_arm(True)
            if ok_mode and ok_arm:
                self.takeoff_origin = xyz
                self._build_scan_waypoints()
                self.scan_start_time = time.time()
                self._enter('TAKEOFF')
            else:
                if (time.time() - self.state_enter) > 8.0:
                    self.get_logger().error('Could not enter OFFBOARD+ARM. Switching to AUTO.LAND.')
                    self._try_set_mode('AUTO.LAND')
                    self._enter('ABORT')

        elif self.state == 'TAKEOFF':
            x0, y0, _ = self.takeoff_origin if self.takeoff_origin else xyz
            self._set_sp(x0, y0, self.takeoff_alt, yaw=self._uav_yaw())
            if abs(xyz[2] - self.takeoff_alt) < 0.5:
                self._enter('AREA_SCAN')

        elif self.state == 'AREA_SCAN':
            if self.scan_start_time and (time.time() - self.scan_start_time) > self.scan_timeout:
                self.get_logger().warn('Scan timed out -> proceeding to rendezvous/landing attempt.')
                self._enter('RENDEZVOUS_UGV')
            else:
                self._run_scan_step()
                if int(time.time()) % 1 == 0:
                    self._publish_map()
                if self.target_found and self.target_global_xy:
                    self._publish_target_pose(self.target_global_xy[0], self.target_global_xy[1])
                    self._enter('RENDEZVOUS_UGV')

        elif self.state == 'RENDEZVOUS_UGV':
            ugv = self._ugv_xyz()
            if ugv is None:
                x0, y0, _ = self.takeoff_origin if self.takeoff_origin else xyz
                self._set_sp(x0, y0, self.hover_alt, yaw=self._uav_yaw())
            else:
                self._set_sp(ugv[0], ugv[1], self.hover_alt, yaw=self._uav_yaw())
                if math.hypot(xyz[0]-ugv[0], xyz[1]-ugv[1]) < 1.0:
                    self.descend_target_z = self.hover_alt
                    self._enter('ALIGN_AND_DESCEND')

        elif self.state == 'ALIGN_AND_DESCEND':
            ok = self._precision_align_step()
            if not ok:
                if (time.time() - self.state_enter) > 20.0:
                    self.get_logger().error('No reliable landing alignment -> AUTO.LAND at current position.')
                    self._try_set_mode('AUTO.LAND')
                    self._enter('ABORT')
            else:
                if self.descend_target_z is None:
                    self.descend_target_z = xyz[2]
                if xyz[2] <= self.final_land_alt:
                    self.get_logger().info('Switching to AUTO.LAND for final touchdown.')
                    self._try_set_mode('AUTO.LAND')
                    self._enter('AUTO_LAND')
                else:
                    dt = 0.2
                    self.descend_target_z = max(self.final_land_alt, self.descend_target_z - self.descend_rate * dt)
                    self.setpoint.pose.position.z = float(self.descend_target_z)

        elif self.state == 'AUTO_LAND':
            if xyz[2] < 0.2:
                self._try_arm(False)
                self._enter('COMPLETE')

        elif self.state == 'ABORT':
            if xyz[2] < 0.2:
                self._try_arm(False)
                self._enter('COMPLETE')

        elif self.state == 'COMPLETE':
            self._publish_map()

        self._log()

    # --------------------------
    # Scan planner + runner
    # --------------------------
    def _build_scan_waypoints(self) -> None:
        if self.takeoff_origin is None:
            return
        x0, y0, _ = self.takeoff_origin

        x_min = x0 - self.scan_w/2.0
        x_max = x0 + self.scan_w/2.0
        y_min = y0 - self.scan_h/2.0
        y_max = y0 + self.scan_h/2.0

        waypoints: List[Tuple[float, float, float]] = []
        y = y_min
        direction = 1
        while y <= y_max + 1e-6:
            if direction > 0:
                waypoints.append((x_min, y, self.scan_alt))
                waypoints.append((x_max, y, self.scan_alt))
            else:
                waypoints.append((x_max, y, self.scan_alt))
                waypoints.append((x_min, y, self.scan_alt))
            y += self.lane
            direction *= -1

        self.scan_waypoints = waypoints
        self.scan_idx = 0

        # Publish scan plan (PoseArray)
        pa = PoseArray()
        pa.header.frame_id = 'map'
        pa.header.stamp = self.get_clock().now().to_msg()
        for (x, y, z) in waypoints:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.position.z = float(z)
            p.orientation = yaw_to_quat(0.0)
            pa.poses.append(p)
        self.pub_plan.publish(pa)

    def _run_scan_step(self) -> None:
        if not self.scan_waypoints:
            self._build_scan_waypoints()
            return

        self._check_target_detection()

        x, y, z = self.scan_waypoints[self.scan_idx]
        self._set_sp(x, y, z, yaw=self._uav_yaw())

        xyz = self._uav_xyz()
        if xyz is None:
            return
        if math.hypot(xyz[0]-x, xyz[1]-y) < self.wp_accept and abs(xyz[2]-z) < 1.0:
            self.scan_idx += 1
            if self.scan_idx >= len(self.scan_waypoints):
                self.get_logger().info('Scan complete.')
                self._enter('RENDEZVOUS_UGV')

    def _check_target_detection(self) -> None:
        det = self._get_aruco(self.target_id)
        if det is None:
            return
        xyz = self._uav_xyz()
        if xyz is None:
            return
        d_enu = self._body_flu_to_enu(det.x_fwd, det.y_left)
        if d_enu is None:
            return
        target_x = xyz[0] + d_enu[0]
        target_y = xyz[1] + d_enu[1]

        self.target_found = True
        self.target_global_xy = (target_x, target_y)

    def _publish_target_pose(self, x: float, y: float) -> None:
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation = yaw_to_quat(0.0)
        self.pub_target.publish(msg)

    # --------------------------
    # Landing / precision alignment
    # --------------------------
    def _precision_align_step(self) -> bool:
        xyz = self._uav_xyz()
        if xyz is None:
            return False

        det = self._get_aruco(self.landing_id)
        if det is not None:
            d_enu = self._body_flu_to_enu(det.x_fwd, det.y_left)
            if d_enu is None:
                return False

            err_e, err_n = d_enu[0], d_enu[1]
            dist = math.hypot(err_e, err_n)

            # correction step towards marker
            step_e = clamp(self.align_kp * err_e, -self.align_max_step, self.align_max_step)
            step_n = clamp(self.align_kp * err_n, -self.align_max_step, self.align_max_step)

            new_x = xyz[0] + step_e
            new_y = xyz[1] + step_n
            new_z = float(self.descend_target_z if self.descend_target_z is not None else xyz[2])
            self._set_sp(new_x, new_y, new_z, yaw=self._uav_yaw())

            # only allow descent when close enough
            return dist < 3.0

        # Fallback: use UGV odom
        ugv = self._ugv_xyz()
        if ugv is None:
            return False

        new_z = float(self.descend_target_z if self.descend_target_z is not None else xyz[2])
        self._set_sp(ugv[0], ugv[1], new_z, yaw=self._uav_yaw())
        return True


    def _csv_log(self, event: str, descent_vz_cmd: float = float('nan')) -> None:
        """Write a single telemetry snapshot to CSV (best-effort)."""
        if self._csv_writer is None:
            return
        try:
            t_utc = datetime.utcnow().isoformat(timespec='milliseconds') + 'Z'
            armed = bool(self._state.armed) if self._state is not None else False
            mode = str(self._state.mode) if self._state is not None else ''
            connected = bool(self._state.connected) if self._state is not None else False

            x = y = z = vx = vy = vz = float('nan')
            if self._uav_odom is not None:
                x = float(self._uav_odom.pose.pose.position.x)
                y = float(self._uav_odom.pose.pose.position.y)
                z = float(self._uav_odom.pose.pose.position.z)
                vx = float(self._uav_odom.twist.twist.linear.x)
                vy = float(self._uav_odom.twist.twist.linear.y)
                vz = float(self._uav_odom.twist.twist.linear.z)

            aruco_seen = bool(self._last_aruco is not None)
            aruco_id = int(self._last_aruco_id) if self._last_aruco_id is not None else -1
            rx = ry = rz = float('nan')
            if self._last_aruco is not None:
                rx, ry, rz = float(self._last_aruco[0]), float(self._last_aruco[1]), float(self._last_aruco[2])

            err_xy = float('nan')
            err_z = float('nan')
            if self._last_aruco is not None:
                err_xy = float((rx**2 + ry**2) ** 0.5)
                err_z = float(rz)

            self._csv_writer.writerow([
                t_utc, event,
                int(armed), mode, int(connected),
                x, y, z, vx, vy, vz,
                int(aruco_seen), aruco_id, rx, ry, rz,
                err_xy, err_z, float(descent_vz_cmd),
            ])
            self._csv_fp.flush()
        except Exception:
            pass

    def _telemetry_tick(self) -> None:
        """Periodic CSV telemetry tick."""
        self._csv_log(event=f'tick:{self.state}')

    def destroy_node(self):
        try:
            if hasattr(self, '_csv_fp') and self._csv_fp is not None:
                self._csv_fp.close()
        except Exception:
            pass
        super().destroy_node()

def main() -> None:
    rclpy.init()
    node = TouchdownControllerPX4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()