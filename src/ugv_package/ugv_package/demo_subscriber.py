import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PIL import Image as PILImage

class RawImageSubscriber(Node):
    def __init__(self):
        super().__init__('raw_image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'raw_image',
            self.listener_callback,
            10
        )
        self.counter = 0  # to save multiple images uniquely

    def listener_callback(self, msg):
        # Reconstruct the image from raw bytes
        pil_image = PILImage.frombytes(
            'RGB', 
            (msg.width, msg.height), 
            bytes(msg.data)
        )

        # Save the image
        filename = f'received_image_{self.counter}.png'
        pil_image.save(filename)
        self.get_logger().info(f'Saved image as {filename}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = RawImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()