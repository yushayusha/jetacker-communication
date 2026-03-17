from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import numpy as np
# -------------------------------
# ROS 2 UGV Nodes
# -------------------------------
class TwistSubscriber(Node):
    def __init__(self, buffer_size=200):
        super().__init__('twist_gui_subscriber')
        self.latest_msg = Twist()

        self.buffer_size = buffer_size
        self.linear_buffer = np.zeros(buffer_size)
        self.angular_buffer = np.zeros(buffer_size)
        self.index = 0

        self.create_subscription(Twist, '/controller/cmd_vel', self.callback, 10)

    def callback(self, msg):
        self.latest_msg = msg
        i = self.index % self.buffer_size
        self.linear_buffer[i] = msg.linear.x
        self.angular_buffer[i] = msg.angular.z
        self.index += 1
    
    def get_buffers(self):
        count = min(self.index, self.buffer_size)
        x = np.arange(count)
        lin = self.linear_buffer[:count]
        ang = self.angular_buffer[:count]
        return x, lin, ang

class OdomSubscriber(Node):
    def __init__(self, buffer_size=2000):
        super().__init__('odom_gui_subscriber')
        self.positions = np.zeros((buffer_size, 2))  # store x, y
        self.index = 0
        self.buffer_size = buffer_size
        self.create_subscription(Odometry, '/odom', self.callback, 10)

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        i = self.index % self.buffer_size
        self.positions[i] = [x, y]
        self.index += 1

    def get_positions(self):
        count = min(self.index, self.buffer_size)
        return self.positions[:count, :]

