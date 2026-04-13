import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy




class FileSubscriber(Node):
    def __init__(self):
        super().__init__('file_sub')

        qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.sub = self.create_subscription(UInt8MultiArray, 'file_topic', self.callback, qos)

    def callback(self, msg):
        data = bytes(msg.data)
        with open('mission.csv', 'wb') as f:
            f.write(data)
        print("Received file, size:", len(data))

def main():
    rclpy.init()
    node = FileSubscriber()
    rclpy.spin(node)

if __name__ == '__main__':
    main()