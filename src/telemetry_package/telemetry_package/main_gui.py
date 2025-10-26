import sys
import threading
import signal
import rclpy
from rclpy.node import Node
from pathlib import Path
from geometry_msgs.msg import Twist

from PySide6.QtWidgets import QApplication, QMainWindow, QMessageBox
from PySide6.QtUiTools import QUiLoader
from PySide6.QtCore import QFile, QTimer

import numpy as np
import pyqtgraph as pg

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

        self.ui.btn_shutdown.clicked.connect(self.shutdown_all)

        self.ui.show()
        # Timer to update GUI
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_labels)
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(100)  # update every 100 ms
        
    def update_labels(self):
        msg = self.ros_node.latest_msg
        self.ui.lbl_linear.setText(f"Linear Velocity: {msg.linear.x:.2f}")
        self.ui.lbl_angular.setText(f"Angular Velocity: {msg.angular.z:.2f}")

    def update_gui(self):
        msg = self.ros_node.latest_msg

        # Update plot
        x, lin, ang = self.ros_node.get_buffers()
        self.curve_lin.setData(x, lin)
        self.curve_ang.setData(x, ang)
    
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
    node = TwistSubscriber()

    # Run ROS in background thread
    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    # Start Qt GUI
    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()

     # --- Allow Ctrl+C to work while Qt is running ---
    signal.signal(signal.SIGINT, lambda *args: graceful_shutdown(app, node, ros_thread))
    timer = QTimer()
    timer.start(100)
    timer.timeout.connect(lambda: None)  # let Qt process signals

    app.exec()

    # Cleanup
    graceful_shutdown(app, node, ros_thread)

if __name__ == '__main__':
    main()

def graceful_shutdown(app, node, ros_thread=None):
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
    if ros_thread and ros_thread.is_alive():
        ros_thread.join(timeout=2.0)

    app.quit()
    sys.exit(0)