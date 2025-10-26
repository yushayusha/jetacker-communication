import sys
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtUiTools import QUiLoader
from PySide6.QtCore import QFile, QTimer

# -------------------------------
# ROS 2 Node: subscribes to /cmd_vel
# -------------------------------
class TwistSubscriber(Node):
    def __init__(self):
        super().__init__('twist_gui_subscriber')
        self.latest_msg = Twist()
        self.create_subscription(Twist, '/controller/cmd_vel', self.callback, 10)

    def callback(self, msg):
        self.latest_msg = msg

# -------------------------------
# Qt GUI Window
# -------------------------------
class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

        # Load the .ui file
        ui_file = QFile("robot_dashboard.ui")   
        loader = QUiLoader()
        self.ui = loader.load(ui_file, self)
        self.setCentralWidget(self.ui)
        self.setWindowTitle("Twist Monitor")
        self.ui.show()

        # Timer to update GUI
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_labels)
        self.timer.start(100)  # update every 100 ms

    def update_labels(self):
        msg = self.ros_node.latest_msg
        self.ui.lbl_linear.setText(f"Linear Velocity: {msg.linear.x:.2f}")
        self.ui.lbl_angular.setText(f"Angular Velocity: {msg.angular.z:.2f}")

# -------------------------------
# ROS + Qt Integration
# -------------------------------
def ros_spin(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = TwistSubscriber()

    # Run ROS in background thread
    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    # Start Qt GUI
    app = QApplication(sys.argv)
    window = MainWindow(node)
    app.exec()

    # Cleanup
    rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()