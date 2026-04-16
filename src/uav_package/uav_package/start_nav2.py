import rclpy
from rclpy.node import Node
import time
import paramiko


class Nav2(Node):
    def __init__(self):
        super().__init__('nav2_node')

        # Run once using a timer (avoids blocking init)
        self.timer = self.create_timer(1.0, self.run_once)
        self.has_run = False

    def run_once(self):
        if self.has_run:
            return

        self.has_run = True

        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            client.connect("192.168.149.1", username="pi", password="raspberrypi")

            shell = client.invoke_shell()
            time.sleep(2)

            # Enter container interactively
            shell.send(
                "export DISPLAY=:0 && cd jetacker_pi && ./navigation.sh \n"
            )

            time.sleep(2)

            output = shell.recv(65535).decode()
            print(output)

        except Exception as e:
            self.get_logger().error(f"SSH failed: {e}")

        finally:
            client.close()

        # Optional: shutdown node after running once
        self.get_logger().info("Finished execution, shutting down")
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Nav2()
    rclpy.spin(node)


if __name__ == '__main__':
    main()