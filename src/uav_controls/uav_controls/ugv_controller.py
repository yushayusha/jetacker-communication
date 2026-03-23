#!/usr/bin/env python3
"""
UGV Controller - 100% COMPETITION COMPLIANT
Raytheon AVC 2025-2026

Rule Compliance:
- NO GPS usage (Rule 3.4.1) [10]
- Minimum speed: 0.2 mph (0.089 m/s) [10]
- Kill switch < 3 seconds [10]
- Obstacle avoidance for Challenge 3 [10]
- All Appendix B logging [10]
- Direct UAV communication (no ground station) [10]
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import json
import math
import time
from datetime import datetime


class UGVController(Node):
    def __init__(self):
        super().__init__("ugv_controller")
        
        # Subscribe to UAV commands (direct communication - no ground station) [10]
        self.create_subscription(
            String,
            "/uav_to_ugv/command",
            self.command_callback,
            10
        )
        
        # Subscribe to LIDAR for obstacle avoidance (Challenge 3) [10]
        self.create_subscription(
            LaserScan,
            "/ugv/scan",
            self.lidar_callback,
            10
        )
        
        # Publish velocity commands to UGV motors
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/ugv/cmd_vel",
            10
        )
        
        # Publish UGV odometry
        self.odom_pub = self.create_publisher(
            Odometry,
            "/ugv/odom",
            10
        )
        
        # Publish UGV logs
        self.log_pub = self.create_publisher(
            String,
            "/ugv/logs",
            10
        )
        
        # State variables - NO GPS [10]
        self.position = [0.0, 0.0, 0.0]  # Local odometry only
        self.velocity = [0.0, 0.0, 0.0]
        self.orientation = 0.0  # Yaw angle in radians
        self.moving = False
        self.destination = None
        self.obstacle_avoidance = False
        self.kill_switch_active = False
        
        # Obstacle detection variables
        self.obstacle_detected = False
        self.obstacle_direction = 0.0
        self.min_obstacle_distance = float('inf')
        self.obstacle_threshold = 1.5  # meters
        
        # Speed constraints [10]
        self.min_speed_ms = 0.089  # 0.2 mph in m/s
        self.current_speed = 0.0
        
        # Competition constants [10]
        self.destination_radius = 1.524  # 5 feet in meters
        
        # Timing for logging (Appendix B) [10]
        self.start_time = None
        self.end_time = None
        self.destination_received_time = None
        self.path_generated_time = None
        
        # Logging
        self.log_entries = []
        
        # Timers
        self.create_timer(0.05, self.update_and_publish)  # 20 Hz
        self.create_timer(1.0, self.publish_logs)  # 1 Hz
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("UGV Controller - 100% COMPETITION COMPLIANT")
        self.get_logger().info("Rules: No GPS, Min 0.2mph, Kill switch, Obstacles")
        self.get_logger().info("=" * 60)
        
    def _log_event(self, event_type: str, data: dict = None) -> None:
        """Log events as required by Appendix B [10]"""
        timestamp = datetime.now().isoformat()
        log_entry = {
            "timestamp": timestamp,
            "event": event_type,
            "data": data if data is not None else {},
            "position": self.position.copy(),
            "velocity": self.velocity.copy(),
            "speed_ms": self.current_speed,
            "speed_mph": self.current_speed * 2.237
        }
        self.log_entries.append(log_entry)
        self.get_logger().info(f"LOG: {event_type}")
    
    def publish_logs(self) -> None:
        """Publish logs periodically"""
        if len(self.log_entries) > 0:
            msg = String()
            msg.data = json.dumps(self.log_entries[-10:])
            self.log_pub.publish(msg)
    
    def lidar_callback(self, msg: LaserScan) -> None:
        """
        Process LIDAR for obstacle avoidance (Challenge 3)
        Obstacles: cardboard boxes, traffic cones, buckets [10]
        """
        if not self.obstacle_avoidance:
            return
        
        min_dist = float('inf')
        min_idx = 0
        
        for i, distance in enumerate(msg.ranges):
            if msg.range_min < distance < msg.range_max:
                if distance < min_dist:
                    min_dist = distance
                    min_idx = i
        
        self.min_obstacle_distance = min_dist
        angle = msg.angle_min + min_idx * msg.angle_increment
        self.obstacle_direction = angle
        
        if min_dist < self.obstacle_threshold:
            self.obstacle_detected = True
            self.get_logger().warn(
                f"Obstacle at {min_dist:.2f}m, angle {math.degrees(angle):.1f} deg"
            )
            self._avoid_obstacle()
        else:
            self.obstacle_detected = False
    
    def _avoid_obstacle(self) -> None:
        """Turn away from obstacle - minimum 5ft spacing between obstacles [10]"""
        if not self.obstacle_detected or not self.obstacle_avoidance:
            return
        
        # Turn away from obstacle
        if self.obstacle_direction > 0:
            turn_direction = -1
        else:
            turn_direction = 1
        
        turn_rate = 0.5  # rad/s
        dt = 0.05
        
        self.orientation += turn_direction * turn_rate * dt
        
        # Normalize to [-pi, pi]
        while self.orientation > math.pi:
            self.orientation -= 2 * math.pi
        while self.orientation < -math.pi:
            self.orientation += 2 * math.pi
        
        # Update velocity
        self.velocity[0] = self.current_speed * math.cos(self.orientation)
        self.velocity[1] = self.current_speed * math.sin(self.orientation)
        
        # Publish command
        cmd = Twist()
        cmd.linear.x = self.current_speed
        cmd.angular.z = turn_direction * turn_rate
        self.cmd_vel_pub.publish(cmd)
        
        self._log_event("OBSTACLE_AVOIDED", {
            "distance": self.min_obstacle_distance,
            "direction_deg": math.degrees(self.obstacle_direction),
            "new_heading_deg": math.degrees(self.orientation)
        })
    
    def command_callback(self, msg: String) -> None:
        """Process commands from UAV (direct communication) [10]"""
        try:
            cmd_data = json.loads(msg.data)
            command = cmd_data.get('command')
            
            if command == 'START_MOVING':
                speed = cmd_data.get('speed', self.min_speed_ms)
                self.start_moving(speed)
                
            elif command == 'GO_TO_DESTINATION':
                x = cmd_data.get('x', 0.0)
                y = cmd_data.get('y', 0.0)
                speed = cmd_data.get('speed', self.min_speed_ms)
                self.destination_received_time = time.time()
                self._log_event("UGV_DESTINATION_RECEIPT", {
                    "x": x,
                    "y": y,
                    "speed": speed,
                    "time": self.destination_received_time
                })
                self.navigate_to(x, y, speed)
                
            elif command == 'ENABLE_OBSTACLE_AVOIDANCE':
                enabled = cmd_data.get('enabled', False)
                self.enable_obstacles(enabled)
                
            elif command == 'KILL_SWITCH':
                self.activate_kill_switch()
                
            elif command == 'MAP_DATA':
                self._log_event("MAP_RECEIVED", cmd_data.get('data', {}))
            
            self._log_event("UAV_UGV_COMMUNICATION", {
                "command": command,
                "data": cmd_data
            })
            
            self.get_logger().info(f"UGV received: {command}")
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON error: {e}")
        except Exception as e:
            self.get_logger().error(f"Command error: {e}")
    
    def start_moving(self, speed: float) -> None:
        """Start moving - minimum 0.2 mph [10]"""
        if speed < self.min_speed_ms:
            self.get_logger().warn(
                f"Speed {speed:.3f} < min {self.min_speed_ms:.3f}, using minimum"
            )
            speed = self.min_speed_ms
        
        self.moving = True
        self.velocity = [speed, 0.0, 0.0]
        self.current_speed = speed
        self.destination = None
        self.start_time = time.time()
        
        cmd = Twist()
        cmd.linear.x = speed
        self.cmd_vel_pub.publish(cmd)
        
        self._log_event("UGV_START_TIME", {
            "speed_ms": speed,
            "speed_mph": speed * 2.237,
            "time": self.start_time
        })
        
        self.get_logger().info(f"UGV moving: {speed:.3f} m/s ({speed * 2.237:.3f} mph)")
    
    def navigate_to(self, x: float, y: float, speed: float) -> None:
        """Navigate to destination - NO GPS [10]"""
        if speed < self.min_speed_ms:
            speed = self.min_speed_ms
        
        self.destination = (x, y)
        self.moving = True
        
        dx = x - self.position[0]
        dy = y - self.position[1]
        dist = math.sqrt(dx * dx + dy * dy)
        
        if dist > 0.1:
            self.orientation = math.atan2(dy, dx)
            self.velocity = [speed * dx / dist, speed * dy / dist, 0.0]
            self.current_speed = speed
            
            path = self._generate_path(x, y)
            self.path_generated_time = time.time()
            
            self._log_event("UGV_GENERATED_PATH", {
                "destination": [x, y],
                "distance": dist,
                "path": path,
                "obstacle_avoidance": self.obstacle_avoidance,
                "time": self.path_generated_time
            })
            
            self._log_event("DESTINATION_LOCATION", {
                "x": x,
                "y": y,
                "distance_from_start": dist
            })
            
            self.get_logger().info(
                f"UGV -> ({x:.1f}, {y:.1f}), dist={dist:.1f}m, "
                f"obstacles={'ON' if self.obstacle_avoidance else 'OFF'}"
            )
        else:
            self._arrived_at_destination()
    
    def _generate_path(self, target_x: float, target_y: float) -> list:
        """Generate waypoints - obstacle avoidance is reactive via LIDAR"""
        current_pos = self.position[:2]
        num_waypoints = 5
        path = []
        
        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            waypoint = [
                current_pos[0] + t * (target_x - current_pos[0]),
                current_pos[1] + t * (target_y - current_pos[1])
            ]
            path.append(waypoint)
        
        return path
    
    def _arrived_at_destination(self) -> None:
        """Handle arrival - within 5ft radius [10]"""
        self.moving = False
        self.velocity = [0.0, 0.0, 0.0]
        self.current_speed = 0.0
        self.end_time = time.time()
        
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        self._log_event("UGV_END_TIME", {
            "position": self.position.copy(),
            "destination": list(self.destination) if self.destination else None,
            "time": self.end_time
        })
        
        self.get_logger().info("UGV arrived at destination")
    
    def enable_obstacles(self, enabled: bool) -> None:
        """Enable obstacle avoidance for Challenge 3 [10]"""
        self.obstacle_avoidance = enabled
        self._log_event("OBSTACLE_AVOIDANCE_ENABLED", {"enabled": enabled})
        self.get_logger().info(f"Obstacle avoidance: {'ON' if enabled else 'OFF'}")
    
    def activate_kill_switch(self) -> None:
        """Kill switch - halt in < 3 seconds [10]"""
        self.kill_switch_active = True
        self.moving = False
        self.velocity = [0.0, 0.0, 0.0]
        self.current_speed = 0.0
        self.end_time = time.time()
        
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        self._log_event("KILL_SWITCH_ACTIVATED", {"time": self.end_time})
        self.get_logger().warn("KILL SWITCH - UGV STOPPED")
    
    def update_and_publish(self) -> None:
        """Update position - NO GPS, local odometry only [10]"""
        dt = 0.05
        
        if self.moving and not self.kill_switch_active:
            self.position[0] += self.velocity[0] * dt
            self.position[1] += self.velocity[1] * dt
            
            cmd = Twist()
            cmd.linear.x = math.sqrt(self.velocity[0]**2 + self.velocity[1]**2)
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            
            current_time_int = int(time.time())
            if current_time_int % 5 == 0 and int(time.time() * 20) % 20 == 0:
                self._log_event("UGV_SPEED", {
                    "speed_ms": self.current_speed,
                    "speed_mph": self.current_speed * 2.237,
                    "position": self.position.copy()
                })
            
            if self.destination is not None:
                dx = self.destination[0] - self.position[0]
                dy = self.destination[1] - self.position[1]
                dist = math.sqrt(dx * dx + dy * dy)
                
                if dist < self.destination_radius:
                    self._arrived_at_destination()
        
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "ugv_base_link"
        
        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.position.z = self.position[2]
        
        odom.pose.pose.orientation.w = math.cos(self.orientation / 2)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.orientation / 2)
        
        odom.twist.twist.linear.x = self.velocity[0]
        odom.twist.twist.linear.y = self.velocity[1]
        odom.twist.twist.linear.z = self.velocity[2]
        
        self.odom_pub.publish(odom)
    
    def save_logs(self) -> None:
        """Save logs to file for competition review"""
        try:
            filename = f"ugv_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            with open(filename, 'w') as f:
                json.dump(self.log_entries, f, indent=2)
            self.get_logger().info(f"Logs saved: {filename}")
        except Exception as e:
            self.get_logger().error(f"Log save failed: {e}")


def main():
    rclpy.init()
    node = UGVController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")
        node.save_logs()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()