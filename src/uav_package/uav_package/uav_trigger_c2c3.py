import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class UAVTrigger(Node):
    def __init__(self):
        super().__init__('uav_trigger_node')

        self.publisher_ = self.create_publisher(
            String,
            '/uav_to_ugv/command',
            10
        )

        self.timer = self.create_timer(0.5, self.check_and_publish)
        self.sent = False

    def check_and_publish(self):
        if self.sent:
            return

        sub_count = self.publisher_.get_subscription_count()

        if sub_count == 0:
            self.get_logger().info("Waiting for subscriber...")
            return

        self.get_logger().info(f"Subscriber detected ({sub_count}), publishing...")

        msg = String()
        msg.data = json.dumps({"command": "NAVIGATE_TO"})

        self.publisher_.publish(msg)
        self.get_logger().info("Command sent")

        self.sent = True

        # Optional: shut down after publishing once
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = UAVTrigger()
    rclpy.spin(node)


if __name__ == '__main__':
    main()