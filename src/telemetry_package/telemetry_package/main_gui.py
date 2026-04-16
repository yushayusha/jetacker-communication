import sys
import threading
import signal
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from pathlib import Path
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField, BatteryState

from PySide6.QtWidgets import QApplication, QMainWindow, QMessageBox
from PySide6.QtUiTools import QUiLoader
from PySide6.QtCore import QFile, QTimer

import numpy as np
import pyqtgraph as pg
import subprocess
import os
import time
import paramiko

# -------------------------------
# ROS 2 UAV Nodes
# -------------------------------

class PixhawkImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_gui_subscriber')
        self.imu_data = None
        self.mag_data = None
        self.create_subscription(Imu, 'uav/imu/data_raw', self.imu_callback, 10)
        self.create_subscription(MagneticField, 'uav/imu/mag', self.mag_callback, 10)

    def imu_callback(self, msg):
        self.imu_data = msg

    def mag_callback(self, msg):
        self.mag_data = msg

class PixhawkBatterySubscriber(Node):
    def __init__(self):
        super().__init__('battery_gui_subscriber')
        self.data = None
        self.create_subscription(BatteryState, '/uav/battery', self.callback, 10)

    def callback(self, msg):
        self.data = msg


# -------------------------------
# ROS 2 UGV Nodes
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
    def __init__(self, nodelist):
        super().__init__()
        self.twist_node = nodelist[0]
        self.odom_node = nodelist[1]
        self.pixhawk_imu_node = nodelist[2]
        self.pixhawk_battery_node = nodelist[3]
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
        self.ui.btn_start_challenge2.clicked.connect(self.start_challenge_2)
        self.ui.btn_start_challenge3.clicked.connect(self.start_challenge_3)
        
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

        if self.pixhawk_battery_node.data:
            battery = self.pixhawk_battery_node.data
            self.ui.battery_label.setText(f"Battery:{battery.percentage:.2f}")


    def start_challenge_1(self):

        #connect to UAV
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect("192.168.149.192", username="pi", password="raspberrypi")
        #client.connect("10.42.0.1", username="pi", password="raspberrypi")
        shell = client.invoke_shell()
        time.sleep(1)

        # Enter container interactively
        shell.send("cd ros2_ws \n")
        time.sleep(1)
        shell.send("source start_challenge1.sh \n")
        time.sleep(10)
        output = shell.recv(65535).decode()
        print(output)
        
        client.close() 

    def start_challenge_2(self):

        #connect to UAV
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect("192.168.149.192", username="pi", password="raspberrypi")
        #client.connect("10.42.0.1", username="pi", password="raspberrypi")
        shell = client.invoke_shell()
        time.sleep(1)

        # Enter container interactively
        shell.send("cd ros2_ws \n")
        time.sleep(1)
        shell.send("source start_challenge2.sh \n")
        time.sleep(10)
        output = shell.recv(65535).decode()
        print(output)
        
        client.close() 

    def start_challenge_3(self):

        #connect to UAV
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect("192.168.149.192", username="pi", password="raspberrypi")
        #client.connect("10.42.0.1", username="pi", password="raspberrypi")
        shell = client.invoke_shell()
        time.sleep(1)

        # Enter container interactively
        shell.send("cd ros2_ws \n")
        time.sleep(1)
        shell.send("source start_challenge3.sh \n")
        time.sleep(10)
        output = shell.recv(65535).decode()
        print(output)
        
        client.close() 


    def start_recording(self):
        if self.bag_process is not None:
            QMessageBox.warning(self, "Recording", "Bag recording already running.")
            return

        # Create unique folder name
        timestr = time.strftime("%Y%m%d_%H%M%S")
        bag_dir = "rosbags/bag_"+ timestr +"/"

        # Choose topics (modify as needed)
        topics = ["/slam_toolbox/scan_visualization", "/slam_toolbox/feedback", "/slam_toolbox/graph_visualization", "/slam_toolbox/update", "/tf", "/tf_static", "/controller/cmd_vel", "imu/data_raw", "imu/mag"]

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
            "Are you sure you want to shut down all UGV processes?",
            QMessageBox.Yes | QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return

        print("Shutting down all ROS 2 nodes...")

        # Shut down UGV
        try:
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            client.connect("192.168.149.1", username="pi", password="raspberrypi")

            shell = client.invoke_shell()
            time.sleep(2)

            # Enter container interactively
            shell.send(
                "zsh -c 'docker exec -it -u ubuntu -w /home/ubuntu a738b3409c59 "
                "/bin/zsh -c \"pkill -9 -f ros2 && pkill -9 -f python3 && pkill -9 -f sllidar\"' \n"
            )

            time.sleep(2)

            output = shell.recv(65535).decode()
            print(output)
        except Exception as e:
            print(f"Warning: ROS shutdown failed: {e}")
    

# -------------------------------
# ROS + Qt Integration
# -------------------------------

def main():
    rclpy.init()
    
    nodelist = [TwistSubscriber(), OdomSubscriber(),PixhawkImuSubscriber(),PixhawkBatterySubscriber()]    

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
    ros_timer.stop()
    rclpy.shutdown()
    for i in nodelist:
        i.destroy_node()

if __name__ == '__main__':
    main()
