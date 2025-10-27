import sys
import threading
import signal
import rclpy
from rclpy.node import Node
from pathlib import Path
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from PySide6.QtWidgets import QApplication, QMainWindow, QMessageBox
from PySide6.QtUiTools import QUiLoader
from PySide6.QtCore import QFile, QTimer

import numpy as np
import pyqtgraph as pg
import subprocess
import os
import time


# -------------------------------
# ROS 2 Node: subscribes to /controller/cmd_vel
# -------------------------------
class TwistSubscriber(Node):
    def __init__(self, buffer_size=200):
        super().__init__('twist_gui_subscriber')
        self.latest_msg = Twist()

        self.buffer_size = buffer_size
        self.linear_buffer = np.zeros(buffer_size)
        self.angular_buffer = np.zeros(buffer_size)
        self.index = 0

        self.create_subscription(Twist, '/controller/cmd_vel', self.callback, 10)

    def callback(self, msg):
        self.latest_msg = msg
        i = self.index % self.buffer_size
        self.linear_buffer[i] = msg.linear.x
        self.angular_buffer[i] = msg.angular.z
        self.index += 1
    
    def get_buffers(self):
        count = min(self.index, self.buffer_size)
        x = np.arange(count)
        lin = self.linear_buffer[:count]
        ang = self.angular_buffer[:count]
        return x, lin, ang

class OdomSubscriber(Node):
    def __init__(self, buffer_size=2000):
        super().__init__('odom_gui_subscriber')
        self.positions = np.zeros((buffer_size, 2))  # store x, y
        self.index = 0
        self.buffer_size = buffer_size
        self.create_subscription(Odometry, '/odom', self.callback, 10)

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        i = self.index % self.buffer_size
        self.positions[i] = [x, y]
        self.index += 1

    def get_positions(self):
        count = min(self.index, self.buffer_size)
        return self.positions[:count, :]


# -------------------------------
# Qt GUI Window
# -------------------------------
class MainWindow(QMainWindow):
    def __init__(self, twist_node, odom_node):
        super().__init__()
        self.twist_node = twist_node
        self.odom_node = odom_node
        self.bag_process = None

        # Load the .ui file
        ui_file = QFile("robot_dashboard.ui")   
        loader = QUiLoader()
        self.ui = loader.load(ui_file, self)
        self.setCentralWidget(self.ui)

                # --- PyQtGraph setup ---
        self.plot_widget = self.ui.findChild(pg.GraphicsLayoutWidget, "plot_widget")
        if not self.plot_widget:
            # If the UI only had a generic QWidget, wrap it manually
            layout = self.ui.plot_widget.layout()
            self.plot_widget = pg.GraphicsLayoutWidget()
            layout.addWidget(self.plot_widget)

        self.plot = self.plot_widget.addPlot(title="Twist Velocities")
        self.plot.showGrid(x=True, y=True)
        self.plot.addLegend()

        self.curve_lin = self.plot.plot(pen='b', name='Linear x')
        self.curve_ang = self.plot.plot(pen='r', name='Angular z')

        self.ui.btn_start_record.clicked.connect(self.start_recording)
        self.ui.btn_stop_record.clicked.connect(self.stop_recording)
        self.ui.btn_shutdown.clicked.connect(self.shutdown_all)
        
                # ---- Odometry plot setup ----
        self.odom_plot = pg.PlotWidget(title="Odometry (X-Y Path)")
        self.odom_plot.setLabel('left', 'Y (m)')
        self.odom_plot.setLabel('bottom', 'X (m)')
        self.odom_plot.showGrid(x=True, y=True)
        self.odom_plot.setAspectLocked(True)  # keep equal aspect ratio
        self.odom_curve = self.odom_plot.plot(pen='g')
        self.ui.odom_plot_widget.layout().addWidget(self.odom_plot)



        self.ui.show()
        # Timer to update GUI
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_labels)
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(100)  # update every 100 ms
        
    def update_labels(self):
        msg = self.twist_node.latest_msg
        self.ui.lbl_linear.setText(f"Linear Velocity: {msg.linear.x:.2f}")
        self.ui.lbl_angular.setText(f"Angular Velocity: {msg.angular.z:.2f}")

    def start_recording(self):
        if self.bag_process is not None:
            QMessageBox.warning(self, "Recording", "Bag recording already running.")
            return

        # Create unique folder name
        timestr = time.strftime("%Y%m%d_%H%M%S")
        bag_dir = "rosbags/bag_"+ timestr +"/"

        # Choose topics (modify as needed)
        topics = ["/slam_toolbox/scan_visualization", "/slam_toolbox/feedback", "/slam_toolbox/graph_visualization", "/slam_toolbox/update", "/tf", "/tf_static", "/controller/cmd_vel"]

        # Construct command
        cmd = ["ros2", "bag", "record", "-o", bag_dir] + topics
        try:
            self.bag_process = subprocess.Popen(cmd)
            print(f"[INFO] Recording started → {bag_dir}")
            QMessageBox.information(self, "Recording Started", f"Recording to {bag_dir}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to start rosbag: {e}")

    def stop_recording(self):
        if self.bag_process is None:
            QMessageBox.information(self, "Recording", "No recording is currently running.")
            return

        self.bag_process.terminate()
        self.bag_process.wait(timeout=2)
        print("[INFO] Recording stopped.")
        QMessageBox.information(self, "Recording Stopped", "Rosbag recording stopped.")
        self.bag_process = None

    def update_gui(self):
        msg = self.twist_node.latest_msg

        # Update plot
        x, lin, ang = self.twist_node.get_buffers()
        self.curve_lin.setData(x, lin)
        self.curve_ang.setData(x, ang)

        # Update odometry plot
        pos = self.odom_node.get_positions()
        if pos.shape[0] > 1:
            self.odom_curve.setData(pos[:, 0], pos[:, 1])
    
    def shutdown_all(self):
        reply = QMessageBox.question(
            self,
            "Confirm Shutdown",
            "Are you sure you want to shut down all ROS 2 nodes?",
            QMessageBox.Yes | QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return

        print("Shutting down all ROS 2 nodes...")

        # Stop timers or other GUI loops
        if hasattr(self, "timer"):
            self.timer.stop()

        # Shut down ROS 2
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"Warning: ROS shutdown failed: {e}")

        # Close the GUI
        self.close()
        sys.exit(0)

# -------------------------------
# ROS + Qt Integration
# -------------------------------
def ros_spin(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    
    twist_node = TwistSubscriber()
    odom_node = OdomSubscriber()

    twist_thread = threading.Thread(target=ros_spin, args=(twist_node,), daemon=True)
    odom_thread = threading.Thread(target=ros_spin, args=(odom_node,), daemon=True)
    twist_thread.start()
    odom_thread.start()

    # Start Qt GUI
    app = QApplication(sys.argv)
    window = MainWindow(twist_node, odom_node)
    window.show()


     # --- Allow Ctrl+C to work while Qt is running ---
    signal.signal(signal.SIGINT, lambda *args: graceful_shutdown(app, twist_node, odom_node, twist_thread, odom_thread))
    timer = QTimer()
    timer.start(100)
    timer.timeout.connect(lambda: None)  # let Qt process signals

    app.exec()

    # Cleanup
    graceful_shutdown(app, twist_node, odom_node, twist_thread, odom_thread)

if __name__ == '__main__':
    main()

def graceful_shutdown(app, twist_node, odom_node, twist_thread = None, odom_thread = None):
    print("\n[Ctrl+C] Shutting down ROS 2 and GUI gracefully…")

    # Stop GUI timers
    for w in app.topLevelWidgets():
        if hasattr(w, "timer"):
            w.timer.stop()

    # Shut down ROS 2
    try:
        rclpy.shutdown()
    except Exception as e:
        print(f"Warning during ROS shutdown: {e}")

    # Wait for background threads
    if twist_thread and twist_thread.is_alive():
        twist_thread.join(timeout=2.0)
    
    if odom_thread and odom_thread.is_alive():
        odom_thread.join(timeout=2.0)

    app.quit()
    sys.exit(0)