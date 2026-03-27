#!/usr/bin/env python3
"""
aruco_detector_improved.py

ROS 2 ArUco detector with a robust JSON output format that matches the UAV touchdown controller.

Publishes:
  - /aruco/detections (std_msgs/String): JSON dict with fields:
        { "id": <int>, "x": <float>, "y": <float>, "z": <float>, "yaw_deg": <float>, "frame": "body_flu" }
    where (x,y,z) are in BODY FLU: x=forward, y=left, z=up (meters).
  - /aruco/detection (std_msgs/String): Backward-compatible JSON dict:
        { "id": <int>, "position": {"x": <float>, "y": <float>, "z": <float>}, "yaw": <float>, "frame": "camera_optical" }

If OpenCV/cv_bridge are not available OR no images arrive, you can enable GPS fallback:
  - fallback_to_gps:=true
which synthesizes a "detection" using UAV local pose and UGV odom (good for sim bring-up).

Notes:
- For OpenCV pose estimation you need camera intrinsics from /camera/camera_info.
- The detector assumes the camera optical frame convention (OpenCV): x=right, y=down, z=forward.
  It converts that to BODY FLU for /aruco/detections.

"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String

try:
    import cv2  # type: ignore
    from cv_bridge import CvBridge  # type: ignore
    from sensor_msgs.msg import Image, CameraInfo  # type: ignore
except Exception:  # pragma: no cover
    cv2 = None
    CvBridge = None
    Image = None
    CameraInfo = None

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    dist: list[float]


def quat_to_yaw(q) -> float:
    # geometry_msgs Quaternion -> yaw (ENU, radians)
    # yaw about +Z
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ArucoDetectorImproved(Node):
    def __init__(self) -> None:
        super().__init__("aruco_detector_improved")

        # Parameters
        self.declare_parameter("marker_id", 0)
        self.declare_parameter("marker_length_m", 0.18)  # meters
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("detections_topic", "/aruco/detections")
        self.declare_parameter("legacy_topic", "/aruco/detection")

        self.declare_parameter("fallback_to_gps", True)
        self.declare_parameter("uav_pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("ugv_odom_topic", "/ugv/odom")
        self.declare_parameter("pad_offset_x", 0.0)  # ENU meters in UGV frame (assume UGV frame aligned to ENU)
        self.declare_parameter("pad_offset_y", 0.0)
        self.declare_parameter("pad_offset_z", 0.0)

        self.marker_id = int(self.get_parameter("marker_id").value)
        self.marker_length = float(self.get_parameter("marker_length_m").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.det_topic = str(self.get_parameter("detections_topic").value)
        self.legacy_topic = str(self.get_parameter("legacy_topic").value)

        self.fallback_to_gps = bool(self.get_parameter("fallback_to_gps").value)
        self.uav_pose_topic = str(self.get_parameter("uav_pose_topic").value)
        self.ugv_odom_topic = str(self.get_parameter("ugv_odom_topic").value)
        self.pad_offset = (
            float(self.get_parameter("pad_offset_x").value),
            float(self.get_parameter("pad_offset_y").value),
            float(self.get_parameter("pad_offset_z").value),
        )

        self.pub_det = self.create_publisher(String, self.det_topic, 10)
        self.pub_legacy = self.create_publisher(String, self.legacy_topic, 10)

        # State for CV path
        self._bridge = CvBridge() if CvBridge is not None else None
        self._intr: Optional[Intrinsics] = None
        self._last_img_time: float = 0.0

        # State for GPS fallback
        self._uav_pose: Optional[PoseStamped] = None
        self._ugv_odom: Optional[Odometry] = None
        self._uav_pose_stamp: float = 0.0
        self._ugv_odom_stamp: float = 0.0
        self._last_pub_stamp: float = 0.0

        # QoS: BEST_EFFORT is typical for high-rate sim topics (camera, odom, pose)
        self._qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Convenience flag used by the optional heartbeat
        self.use_camera = (cv2 is not None and self._bridge is not None)

        # Subscriptions
        # Camera subscriptions (supports image_topic='auto' and camera_info_topic='auto')
        self._cam_attached = False
        self._cam_attach_warn_counter = 0
        if (self.camera_info_topic == 'auto') or (self.image_topic == 'auto'):
            self.get_logger().info("Camera topics set to 'auto' — searching for Image/CameraInfo topics...")
            self._cam_attach_timer = self.create_timer(0.5, self._try_attach_camera)
        else:
            self._attach_camera(self.image_topic, self.camera_info_topic)

        if self.fallback_to_gps:
            self.create_subscription(PoseStamped, self.uav_pose_topic, self._uav_pose_cb, self._qos_be)
            self.create_subscription(Odometry, self.ugv_odom_topic, self._ugv_odom_cb, self._qos_be)

        # Publish timer for fallback (also helps if CV is intermittent)
        self.create_timer(0.1, self._fallback_timer_cb)

        if cv2 is None or self._bridge is None:
            self.get_logger().warn(
                "OpenCV/cv_bridge not available. Running in GPS-fallback mode only (if enabled)."
            )

    def _attach_camera(self, image_topic: str, caminfo_topic: str) -> None:
        if getattr(self, '_cam_attached', False):
            return
        if CameraInfo is not None and caminfo_topic:
            self.create_subscription(CameraInfo, caminfo_topic, self._caminfo_cb, 10)
        if Image is not None and self._bridge is not None and image_topic:
            self.create_subscription(Image, image_topic, self._image_cb, 10)
        self._cam_attached = True
        self.get_logger().info(f"Subscribed to camera topics: image={image_topic}  camera_info={caminfo_topic}")

    def _try_attach_camera(self) -> None:
        if getattr(self, '_cam_attached', False):
            return
        topics = self.get_topic_names_and_types()
        img_candidates = []
        cam_candidates = []
        for tname, ttypes in topics:
            if any(tt == 'sensor_msgs/msg/Image' for tt in ttypes):
                img_candidates.append(tname)
            if any(tt == 'sensor_msgs/msg/CameraInfo' for tt in ttypes):
                cam_candidates.append(tname)

        def pick(cands):
            pref = [t for t in cands if ('image' in t.lower()) and ('compressed' not in t.lower())]
            if pref:
                return sorted(pref)[0]
            if cands:
                return sorted(cands)[0]
            return None

        img_topic = self.image_topic if self.image_topic != 'auto' else pick(img_candidates)
        cam_topic = self.camera_info_topic if self.camera_info_topic != 'auto' else pick(cam_candidates)

        if img_topic is None:
            self._cam_attach_warn_counter += 1
            if self._cam_attach_warn_counter % 6 == 0:
                self.get_logger().warning("No sensor_msgs/Image topics found yet. Is the sim camera running?")
            return

        # camera_info is optional; we can still run with fallback enabled
        if cam_topic is None:
            self.get_logger().warning("No CameraInfo topic found. ArUco distance may be less accurate (or fallback-only).")
            cam_topic = ''

        self._attach_camera(img_topic, cam_topic)

    # ---------- Callbacks ----------
    def _caminfo_cb(self, msg) -> None:
        try:
            k = msg.k
            d = list(msg.d) if hasattr(msg, "d") else []
            self._intr = Intrinsics(fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5]), dist=d)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse CameraInfo: {e}")

    def _image_cb(self, msg) -> None:
        self._last_img_time = time.time()
        if cv2 is None or self._bridge is None:
            return
        if self._intr is None:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        # Detect ArUco
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            aruco = cv2.aruco  # type: ignore
            dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
            parameters = aruco.DetectorParameters()
            detector = aruco.ArucoDetector(dictionary, parameters)
            corners, ids, _ = detector.detectMarkers(gray)
        except Exception as e:
            self.get_logger().warn(f"OpenCV ArUco detect failed: {e}")
            return

        if ids is None or len(ids) == 0:
            return

        # Pick the first matching marker_id, else first marker
        picked = 0
        ids_list = [int(i[0]) for i in ids]
        if self.marker_id in ids_list:
            picked = ids_list.index(self.marker_id)
        det_id = ids_list[picked]

        # Pose estimation
        try:
            intr = self._intr
            camera_matrix = [[intr.fx, 0, intr.cx], [0, intr.fy, intr.cy], [0, 0, 1]]
            dist_coeffs = intr.dist if len(intr.dist) > 0 else [0, 0, 0, 0, 0]
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(  # type: ignore
                [corners[picked]], self.marker_length, camera_matrix, dist_coeffs
            )
            t = tvec[0][0]  # meters, in camera optical frame: x right, y down, z forward
            x_right, y_down, z_fwd = float(t[0]), float(t[1]), float(t[2])

            # Convert to BODY FLU (assume camera optical is aligned with body forward):
            x_fwd = z_fwd
            y_left = -x_right
            z_up = -y_down

            # Rough yaw estimate from rvec -> rotation matrix
            R, _ = cv2.Rodrigues(rvec[0][0])  # type: ignore
            # yaw about camera Z; convert to degrees
            yaw_rad = math.atan2(R[1, 0], R[0, 0])
            yaw_deg = float(yaw_rad * 180.0 / math.pi)

        except Exception as e:
            self.get_logger().warn(f"OpenCV pose estimate failed: {e}")
            return

        # Publish both formats
        self._publish(det_id, x_fwd, y_left, z_up, yaw_deg, (x_right, y_down, z_fwd))

    def _uav_pose_cb(self, msg: PoseStamped) -> None:
        self._uav_pose = msg
        self._uav_pose_stamp = time.time()

    def _ugv_odom_cb(self, msg: Odometry) -> None:
        self._ugv_odom = msg
        self._ugv_odom_stamp = time.time()

    def _fallback_timer_cb(self) -> None:
        # If images are flowing recently, let CV drive (already publishes)
        if cv2 is not None and (time.time() - self._last_img_time) < 0.5:
            return
        if not self.fallback_to_gps:
            return
        if self._uav_pose is None or self._ugv_odom is None:
            return

        # Compute relative pad vector in world ENU
        up = self._uav_pose.pose.position
        uq = self._uav_pose.pose.orientation
        gp = self._ugv_odom.pose.pose.position

        pad_x = gp.x + self.pad_offset[0]
        pad_y = gp.y + self.pad_offset[1]
        pad_z = gp.z + self.pad_offset[2]

        rel_x = pad_x - up.x
        rel_y = pad_y - up.y
        rel_z = pad_z - up.z

        yaw = quat_to_yaw(uq)

        # World ENU -> Body FLU
        x_fwd = math.cos(yaw) * rel_x + math.sin(yaw) * rel_y
        y_left = -math.sin(yaw) * rel_x + math.cos(yaw) * rel_y
        z_up = rel_z

        self._publish(self.marker_id, x_fwd, y_left, z_up, 0.0, None)

    # ---------- Helpers ----------
    def _publish(
        self,
        det_id: int,
        x_fwd: float,
        y_left: float,
        z_up: float,
        yaw_deg: float,
        raw_optical: Optional[Tuple[float, float, float]],
    ) -> None:
        # New: body_flu flat
        msg_det = {
            "id": int(det_id),
            "x": float(x_fwd),
            "y": float(y_left),
            "z": float(z_up),
            "yaw_deg": float(yaw_deg),
            "frame": "body_flu",
            "t": time.time(),
        }
        self.pub_det.publish(String(data=json.dumps(msg_det)))
        self._last_pub_stamp = time.time()

        # Legacy: camera_optical nested
        if raw_optical is not None:
            xr, yd, zf = raw_optical
        else:
            # If we don't have raw optical, fabricate a consistent one
            # (inverse mapping from body_flu to optical, assuming aligned axes)
            xr, yd, zf = (-y_left), (-z_up), (x_fwd)

        msg_legacy = {
            "id": int(det_id),
            "position": {"x": float(xr), "y": float(yd), "z": float(zf)},
            "yaw": float(yaw_deg),
            "frame": "camera_optical",
            "t": time.time(),
        }
        self.pub_legacy.publish(String(data=json.dumps(msg_legacy)))


    def _status_tick(self) -> None:
        """Periodic heartbeat so you can see the node is alive even if no camera detections."""
        now = time.time()
        age_uav = (now - self._uav_pose_stamp) if getattr(self, '_uav_pose_stamp', None) else None
        age_ugv = (now - self._ugv_odom_stamp) if getattr(self, '_ugv_odom_stamp', None) else None
        age_det = (now - self._last_pub_stamp) if getattr(self, '_last_pub_stamp', None) else None
        self.get_logger().info(
            f"HB use_camera={self.use_camera} fallback_to_gps={self.fallback_to_gps} "
            f"uav_pose_age={age_uav if age_uav is not None else 'NA'} "
            f"ugv_odom_age={age_ugv if age_ugv is not None else 'NA'} "
            f"last_pub_age={age_det if age_det is not None else 'NA'} "
            f"topics: img={self.image_topic} uav_pose={self.uav_pose_topic} ugv_odom={self.ugv_odom_topic}"
        )


def main() -> None:
    rclpy.init()
    node = ArucoDetectorImproved()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()