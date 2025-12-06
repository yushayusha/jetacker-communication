#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

class CombinedImuSubscriber(Node):
    
    def __init__(self):
        super().__init__('combined_imu_subscriber')

        self.imu_data = None
        self.mag_data = None
        self.create_subscription(Imu, 'imu/data_raw', self.imu_callback, 10)
        self.create_subscription(MagneticField, 'imu/mag', self.mag_callback, 10)


    def imu_callback(self, msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        print("Accel:", ax, ay, az)
        print("Gyro:", gx, gy, gz)

    def mag_callback(self, msg):

        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y
        mz = msg.magnetic_field.z

        print("Mag:", mx, my, mz)

def main(args=None):
    rclpy.init(args=args)
    node = CombinedImuSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()