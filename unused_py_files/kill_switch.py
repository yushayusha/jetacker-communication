import sys, tty, termios, rclpy, subprocess
from rclpy.node import Node

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)  # read one character
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class KillSwitchNode(Node):
    def __init__(self):
        super().__init__('one_key_node')
        self.get_logger().info("Press 'X' to kill Gazebo.")
        key = getch()
        if key == 'x':
            self.get_logger().info("You pressed X! Killing Gazebo")
            subprocess.run("ps aux | grep ign | grep -v grep | awk \'{ print \"sudo kill -9\", $2 }\' | sh", shell=True, check=True)
        else:
            self.get_logger().info(f"You pressed '{key}', not 'X' — exiting.")

def main(args=None):
    rclpy.init(args=args)
    node = KillSwitchNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()