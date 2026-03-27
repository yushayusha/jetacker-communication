
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from pymavlink import mavutil
import time

# Connect to flight controller via mavlink
master = None

while master is None:

    print("Attempting MAVLink connection...")
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=9600)

    print("Waiting for heartbeat...")
    master.wait_heartbeat(timeout=3)

    print(
        f"Heartbeat from system "
        f"(system {master.target_system} component {master.target_component})"
    )

"""     except Exception as e:
        print(f"Connection failed: {e}")
        print("Retrying in 3 seconds...")
        master = None
        time.sleep(3)

 """
class PixhawkImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz

    def timer_callback(self):
        imu_data = get_imu_data()
        if imu_data is None:
            return

        ax, ay, az, gx, gy, gz, mx, my, mz = imu_data

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Fill linear acceleration (m/s^2)
        imu_msg.linear_acceleration.x = float(ax)
        imu_msg.linear_acceleration.y = float(ay)
        imu_msg.linear_acceleration.z = float(az)

        # Fill angular velocity (rad/s)
        imu_msg.angular_velocity.x = float(gx)
        imu_msg.angular_velocity.y = float(gy)
        imu_msg.angular_velocity.z = float(gz)

        # Orientation unknown
        imu_msg.orientation_covariance[0] = -1  # indicates unknown

        self.imu_pub.publish(imu_msg)

        # Publish magnetometer separately
        mag_msg = MagneticField()
        mag_msg.header = imu_msg.header
        mag_msg.magnetic_field.x = float(mx)
        mag_msg.magnetic_field.y = float(my)
        mag_msg.magnetic_field.z = float(mz)
        self.mag_pub.publish(mag_msg)

def get_imu_data():
    # Request the RAW_IMU stream
    msg = None
    try:
        if master.target_system == 0 or master.target_component == 0:
            return 0, 0, 0, 0, 0, 0, 0, 0, 0
        

        else: 
            master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
            50,
            1,
            msg = master.recv_match(type='RAW_IMU', blocking=True)
        )

        print("RAW_SENSORS stream requested.")

    except ConnectionError as e:
        print(e)

    except Exception as e:
        print(f"MAVLink request_data_stream_send failed: {e}")

    if msg:
        # Convert raw sensor values to SI units
        ax = msg.xacc * 9.81 / 1000  # assuming milli-g
        ay = msg.yacc * 9.81 / 1000
        az = msg.zacc * 9.81 / 1000

        gx = msg.xgyro * (3.14159 / 180 / 1000)  # assuming millidegrees/sec
        gy = msg.ygyro * (3.14159 / 180 / 1000)
        gz = msg.zgyro * (3.14159 / 180 / 1000)

        mx = msg.xmag  # raw units, may need calibration
        my = msg.ymag
        mz = msg.zmag

        return ax, ay, az, gx, gy, gz, mx, my, mz
    return None


class PixhawkImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_gui_subscriber')
        self.imu_data = None
        self.mag_data = None
        self.create_subscription(Imu, 'imu/data_raw', self.imu_callback, 10)
        self.create_subscription(MagneticField, 'imu/mag', self.mag_callback, 10)

    def imu_callback(self, msg):
        self.imu_data = msg

    def mag_callback(self, msg):
        self.mag_data = msg

