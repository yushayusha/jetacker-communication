import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


def wait_for_specific_subscriber(self, target_node_name):
    while rclpy.ok():
        subs = self.get_subscriptions_info_by_topic('file_topic')

        for sub in subs:
            if sub.node_name == target_node_name:
                self.get_logger().info(f"Found subscriber: {target_node_name}")
                return

        self.get_logger().info("Waiting for specific subscriber...")
        rclpy.spin_once(self, timeout_sec=0.2)

class FilePublisher(Node):
    def __init__(self):
        super().__init__('file_pub')

        qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub = self.create_publisher(UInt8MultiArray, 'file_topic', qos)

        # Read file as bytes
        with open('waypoints.csv', 'rb') as f:
            data = f.read()

        msg = UInt8MultiArray()
        msg.data = list(data)

        self.wait_for_specific_subscriber('file_sub')

        self.pub.publish(msg)
        print(self.pub.get_subscription_count())
        self.get_logger().info('File published')

        rclpy.shutdown()   # publish once then exit
    
    def wait_for_specific_subscriber(self, target_node_name):
        while rclpy.ok():
            subs = self.get_subscriptions_info_by_topic('file_topic')

            for sub in subs:
                if sub.node_name == target_node_name:
                    self.get_logger().info(f"Found subscriber: {target_node_name}")
                    return

            self.get_logger().info("Waiting for specific subscriber...")
            rclpy.spin_once(self, timeout_sec=0.2)



def main():
    rclpy.init()
    FilePublisher()

if __name__ == '__main__':
    main()