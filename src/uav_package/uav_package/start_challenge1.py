import rclpy
from rclpy.node import Node
import time
import paramiko


class Challenge1(Node):
    def __init__(self):
        super().__init__('challenge1_node')

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
                "zsh -c 'docker exec -it -u ubuntu -w /home/ubuntu a738b3409c59 "
                "/bin/zsh -c \"source ~/.zshrc; sleep 5; "
                "xterm -e zsh -c \\\"ros2 run comms get_csv\\\" & "
                "sleep 5; "
                "xterm -e zsh -c \\\"~/start_challenge1.sh /home/ubuntu/mission.csv\\\"\"' \n"
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
    node = Challenge1()
    rclpy.spin(node)


if __name__ == '__main__':
    main()