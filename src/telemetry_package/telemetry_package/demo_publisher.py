import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PIL import Image as PILImage

class RawImagePublisher(Node):
    def __init__(self, image_path):
        super().__init__('raw_image_publisher')
        self.publisher_ = self.create_publisher(Image, 'raw_image', 10)
        self.timer = self.create_timer(1.0, self.publish_image)
        self.image_path = image_path

        # Load image once
        pil_image = PILImage.open(self.image_path)
        pil_image = pil_image.convert('RGB')  # ensure RGB format
        self.width, self.height = pil_image.size
        self.data = pil_image.tobytes()  # raw bytes

    def publish_image(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'

        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = self.width * 3  # 3 bytes per pixel for RGB

        msg.data = self.data

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published image: {self.width}x{self.height}')

        

def main(args=None):
    rclpy.init(args=args)
    node = RawImagePublisher("demo.png")
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()