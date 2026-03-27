#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from pymavlink import mavutil
import math
import numpy as np

class PixhawkCombinedImu(Node):
    def __init__(self):
        super().__init__('pixhawk_combined_imu')

        self.pub = self.create_publisher(Imu, '/pixhawk/imu_all', 10)

        # Pixhawk connection
        self.master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()

        print(
            f"Heartbeat from system "
            f"(system {self.master.target_system} component {self.master.target_component})"
        )

        # Timer
        self.timer = self.create_timer(0.02, self.read_imu)  # 50 Hz

        # Request IMU stream
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            50,
            1
        )

    def read_imu(self):
        msg = self.master.recv_match(
            type=["RAW_IMU", "SCALED_IMU2", "SCALED_IMU3"],
            blocking=False
        )

        if msg is None:
            return

        data = msg.to_dict()
        imu = Imu()

        # Gyro scaling
        if data["mavpackettype"] == "RAW_IMU":
            gyro_scale = (1.0 / 16.4) * (math.pi / 180.0)
            imu.angular_velocity.x = data["xgyro"] * gyro_scale
            imu.angular_velocity.y = data["ygyro"] * gyro_scale
            imu.angular_velocity.z = data["zgyro"] * gyro_scale

            accel_scale = 9.80665 / 16384.0
            imu.linear_acceleration.x = data["xacc"] * accel_scale
            imu.linear_acceleration.y = data["yacc"] * accel_scale
            imu.linear_acceleration.z = data["zacc"] * accel_scale

            # Store magnetometer inside orientation.covariance

        else:  # SCALED_IMU2 or SCALED_IMU3
            imu.angular_velocity.x = data["xgyro"] * math.pi/180.0
            imu.angular_velocity.y = data["ygyro"] * math.pi/180.0
            imu.angular_velocity.z = data["zgyro"] * math.pi/180.0

            imu.linear_acceleration.x = data["xacc"] * 0.001 * 9.80665
            imu.linear_acceleration.y = data["yacc"] * 0.001 * 9.80665
            imu.linear_acceleration.z = data["zacc"] * 0.001 * 9.80665

            # Store magnetometer inside orientation.covariance

        self.pub.publish(imu)

def main(args=None):
    rclpy.init(args=args)
    node = PixhawkCombinedImu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()