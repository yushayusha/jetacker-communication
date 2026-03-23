import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from PySide6.QtWidgets import QApplication, QMainWindow, QMessageBox
from PySide6.QtUiTools import QUiLoader
from PySide6.QtCore import QFile, QTimer

import pyqtgraph as pg
import subprocess
import time

#Import nodes that subscribe to vehicle data
from telemetry_package.uav_subscribers import *
from telemetry_package.ugv_subscribers import *


# -------------------------------
# Qt GUI Window
# -------------------------------
class MainWindow(QMainWindow):
    def __init__(self, nodelist):
        super().__init__()
        self.twist_node = nodelist[0]
        self.odom_node = nodelist[1]
        self.pixhawk_imu_node = nodelist[3]
        self.bag_process = None
        self.challenge1_process = None

        # Load the .ui file
        ui_file = QFile("robot_dashboard.ui")   
        loader = QUiLoader()
        self.ui = loader.load(ui_file, self)
        self.setCentralWidget(self.ui)
        self.ui.show()

         # --- UGV Twist Graph setup ---
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
        self.ui.btn_start_challenge1.clicked.connect(self.start_challenge_1)
        
                # ---- Odometry plot setup ----
        self.odom_plot = pg.PlotWidget(title="Odometry (X-Y Path)")
        self.odom_plot.setLabel('left', 'Y (m)')
        self.odom_plot.setLabel('bottom', 'X (m)')
        self.odom_plot.showGrid(x=True, y=True)
        self.odom_plot.setAspectLocked(True)  # keep equal aspect ratio
        self.odom_curve = self.odom_plot.plot(pen='g')
        self.ui.odom_plot_widget.layout().addWidget(self.odom_plot)

        # Timer to update GUI
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_labels)
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(100)  # update every 100 ms
        
    def update_labels(self):
        msg = self.twist_node.latest_msg
        self.ui.lbl_linear.setText(f"Linear Velocity: {msg.linear.x:.2f}")
        self.ui.lbl_angular.setText(f"Angular Velocity: {msg.angular.z:.2f}")

        if self.pixhawk_imu_node.imu_data:
            imu = self.pixhawk_imu_node.imu_data
            self.ui.ax_label.setText(f"Linear Acceleration X:{imu.linear_acceleration.x:.2f}")
            self.ui.ay_label.setText(f"Linear Acceleration Y:{imu.linear_acceleration.y:.2f}")
            self.ui.az_label.setText(f"Linear Acceleration Z:{imu.linear_acceleration.z:.2f}")
            self.ui.gx_label.setText(f"Angular Velocity X:{imu.angular_velocity.x:.2f}")
            self.ui.gy_label.setText(f"Angular Velocity Y:{imu.angular_velocity.y:.2f}")
            self.ui.gz_label.setText(f"Angular Velocity Z:{imu.angular_velocity.z:.2f}")

        if self.pixhawk_imu_node.mag_data:
            mag = self.pixhawk_imu_node.mag_data
            self.ui.mx_label.setText(f"Magnetic Field X:{mag.magnetic_field.x:.2f}")
            self.ui.my_label.setText(f"Magnetic Field Y: {mag.magnetic_field.y:.2f}")
            self.ui.mz_label.setText(f"Magnetic Field Z:{mag.magnetic_field.z:.2f}")


    def start_recording(self):
        if self.bag_process is not None:
            QMessageBox.warning(self, "Recording", "Bag recording already running.")
            return

        # Create unique folder name
        timestr = time.strftime("%Y%m%d_%H%M%S")
        bag_dir = "rosbags/bag_"+ timestr +"/"

        # Choose topics (modify as needed)
        # topics = ["/slam_toolbox/scan_visualization", "/slam_toolbox/feedback", "/slam_toolbox/graph_visualization", "/slam_toolbox/update", "/tf", "/tf_static", "/controller/cmd_vel", "imu/data_raw", "imu/mag"]
        topics = ["/slam_toolbox/scan_visualization", "/slam_toolbox/feedback", "/slam_toolbox/graph_visualization", "/slam_toolbox/update", "/tf", "/tf_static", "/controller/cmd_vel", "imu/data_raw", "imu/mag"]

        # Construct command 
        cmd = ["ros2", "bag", "record", "-o", bag_dir] + topics
        try:
            self.bag_process = subprocess.Popen(cmd)
            print(f"[INFO] Recording started → {bag_dir}")
            QMessageBox.information(self, "Recording Started", f"Recording to {bag_dir}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to start rosbag: {e}")

    def start_challenge_1(self):
        if self.challenge1_process is not None:
            QMessageBox.warning(self, "Challenge 1", "Challenge 1 is already running.")
            return

        cmd = ["ros2", "run", "telemetry_package", "fsm_execute"]
        try:
            self.challenge1_process = subprocess.Popen(cmd)
            QMessageBox.information(self, "Starting Challnege 1", f"Challenge 1 Started")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to start challenge 1: {e}")

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

def main():
    rclpy.init()
    
    nodelist = [TwistSubscriber(), OdomSubscriber(), PixhawkImuPublisher(),PixhawkImuSubscriber()]    

    executor = SingleThreadedExecutor()
    for i in nodelist:
        executor.add_node(i)


    # Start Qt GUI
    app = QApplication(sys.argv)
    
    window = MainWindow(nodelist)
    window.show()

     # --- Allow Ctrl+C to work while Qt is running ---
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: executor.spin_once(timeout_sec=0.05))
    ros_timer.start(50)

    app.exec()

    # Cleanup
    executor.shutdown()
    rclpy.shutdown()
    for i in nodelist:
        i.destroy_node()

if __name__ == '__main__':
    main()
