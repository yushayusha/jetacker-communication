#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, BatteryState, NavSatFix, MagneticField
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, Float32

from pymavlink import mavutil
import math

class PixhawkBridge(Node):
    def __init__(self):
        super().__init__('pixhawk_bridge')

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/uav/imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/uav/imu/mag', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/uav/battery', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/uav/gps', 10)
        self.state_pub = self.create_publisher(String, '/uav/state', 10)
        self.flow_pub = self.create_publisher(TwistStamped, '/pixhawk/flow', 10)
        self.quality_pub = self.create_publisher(Float32, '/pixhawk/flow_quality', 10)


        # MAVLink connection
        self.get_logger().info("Connecting to Flight Controller...")
        #self.master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
        self.master = mavutil.mavlink_connection('udpin:', baud=57600)
        self.master.wait_heartbeat()
        self.get_logger().info("Connected")

        # Timer loop
        self.timer = self.create_timer(0.01, self.read_mavlink)  # 100 Hz

    def read_mavlink(self):
        # see https://mavlink.io/en/messages/common.html
        msg = self.master.recv_match(blocking=False)
        if msg is None:
            return

        mtype = msg.get_type()

        # ---------------- IMU ----------------
        if mtype == "RAW_IMU":
            imu = Imu()

            # accel (mg → m/s^2)
            imu.linear_acceleration.x = float(msg.xacc)
            imu.linear_acceleration.y = float(msg.yacc)
            imu.linear_acceleration.z = float(msg.zacc)

            # gyro (mdps → rad/s)
            imu.angular_velocity.x = float(msg.xgyro)
            imu.angular_velocity.y = float(msg.ygyro)
            imu.angular_velocity.z = float(msg.zgyro)
            self.imu_pub.publish(imu)

            mag = MagneticField()
            mag.header = imu.header
            mag.magnetic_field.x = float(msg.xmag)
            mag.magnetic_field.y = float(msg.ymag)
            mag.magnetic_field.z = float(msg.zmag)
            self.mag_pub.publish(mag)

        # ---------------- BATTERY ----------------
        elif mtype == "SYS_STATUS":
            batt = BatteryState()

            batt.voltage = msg.voltage_battery / 1000.0 
            batt.current = msg.current_battery / 100.0
            batt.percentage = msg.battery_remaining / 100.0

            self.battery_pub.publish(batt)

        # ---------------- GPS ----------------
        elif mtype == "GLOBAL_POSITION_INT":
            gps = NavSatFix()

            gps.latitude = msg.lat / 1e7
            gps.longitude = msg.lon / 1e7
            gps.altitude = msg.alt / 1000.0

            self.gps_pub.publish(gps)

        # ---------------- STATE ----------------
        elif mtype == "HEARTBEAT":
            state = String()
            state.data = f"Mode: {msg.custom_mode} Armed: {msg.base_mode}"
            self.state_pub.publish(state)
        # ---------------- OPTICAL FLOW ----------------
        elif mtype == "OPTICAL_FLOW_RAD":
            flow = TwistStamped()
            dt = msg.integration_time_us / 1e6
            if dt == 0:
                return
            distance = msg.distance  # meters
            vx = (msg.integrated_x / dt) * distance
            vy = (msg.integrated_y / dt) * distance

            flow.header.stamp = self.get_clock().now().to_msg()
            flow.twist.linear.x = vx
            flow.twist.linear.y = vy

            self.flow_pub.publish(flow)

            flow_quality = Float32()
            flow_quality.data = float(msg.quality)
            self.quality_pub.publish(flow_quality)

def main():
    rclpy.init()
    node = PixhawkBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()