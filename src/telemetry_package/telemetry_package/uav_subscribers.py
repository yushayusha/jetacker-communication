
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
