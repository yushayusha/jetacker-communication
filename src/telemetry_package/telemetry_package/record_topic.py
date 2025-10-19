import rclpy
import subprocess
import os
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MinimalSubscriber(Node):
    
    # Define the output folder (absolute or relative)
    #timestr = time.strftime("%Y%m%d-%H%M%S")
    #output_folder = "bagfiles/" + timestr

    # Record all topics into that folder
    #command = ["ros2", "bag", "record", "-a", "-o", output_folder]

    #process = subprocess.Popen(command)
    #print(f"Recording rosbag to: {output_folder}")
    #try:
    #    process.wait()  # wait until Ctrl+C
    #except KeyboardInterrupt:
     #   print("Stopping recording...")
      #  process.terminate()


    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Twist,                # message type
            '/controller/cmd_vel',     # topic name
            self.listener_callback,# callback function
            10                     # QoS queue size
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Linear X: "{msg.linear.x}"')
        self.get_logger().info(f'Linear Y: "{msg.linear.y}"')
        self.get_logger().info(f'Angular Z: "{msg.angular.z}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()