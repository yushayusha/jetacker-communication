#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, BatteryState, NavSatFix, MagneticField
from std_msgs.msg import String

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

        # MAVLink connection
        self.get_logger().info("Connecting to Pixhawk...")
        self.master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
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
            imu.linear_acceleration.x = msg.xacc * 9.80665 / 1000.0
            imu.linear_acceleration.y = msg.yacc * 9.80665 / 1000.0
            imu.linear_acceleration.z = msg.zacc * 9.80665 / 1000.0

            # gyro (mdps → rad/s)
            imu.angular_velocity.x = msg.xgyro * math.pi / 180.0 / 1000.0
            imu.angular_velocity.y = msg.ygyro * math.pi / 180.0 / 1000.0
            imu.angular_velocity.z = msg.zgyro * math.pi / 180.0 / 1000.0
            self.imu_pub.publish(imu)

            mag = MagneticField()
            mag.header = imu.header
            mag.magnetic_field.x = float(msg.xmag)
            mag.magnetic_field.y = float(msg.ymag)
            mag.magnetic_field.z = float(msg.zmag)
            self.mag_pub.publish(mag)

        # ---------------- BATTERY ----------------
        elif mtype == "BATTERY_STATUS":
            batt = BatteryState()
            data = msg.to_dict()

            batt.voltage = data.voltages
            batt.current = data.current_battery
            batt.percentage = data.battery_remaining

            self.battery_pub.publish(batt)

        # ---------------- GPS ----------------
        elif mtype == "GPS_RAW_INT":
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


        elif msg.get_type() == "ATTITUDE":
            self.get_logger().info(msg.roll, msg.pitch, msg.yaw)

def main():
    rclpy.init()
    node = PixhawkBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()