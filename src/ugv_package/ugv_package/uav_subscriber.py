import rclpy
import subprocess
import os
from rclpy.node import Node
from geographic_msgs.msg import GeoPoseStamped
import time

class MinimalSubscriber(Node):
    
    # Define the output folder (absolute or relative)
    timestr = time.strftime("%Y%m%d-%H%M%S")
    output_folder = "bagfiles/" + timestr

    # Record all topics into that folder
    command = ["ros2", "bag", "record", "-a", "-o", output_folder]

    process = subprocess.Popen(command)
    print(f"Recording rosbag to: {output_folder}")
    try:
        process.wait()  # wait until Ctrl+C
    except KeyboardInterrupt:
        print("Stopping recording...")
        process.terminate()


    def __init__(self):
        super().__init__('minimal_subscriber')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.subscription = self.create_subscription(
            GeoPoseStamped,                # message type
            '/ap/geopose/filtered',     # topic name
            self.listener_callback,# callback function
            qos_profile=qos_policy                     # QoS queue size
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Orientation X: "{msg.pose.orientation.x}"')
        self.get_logger().info(f'Orientation Y: "{msg.pose.orientation.y}"')
        self.get_logger().info(f'Orientation Z: "{msg.pose.orientation.z}"')
        self.get_logger().info(f'Orientation W: "{msg.pose.orientation.w}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()