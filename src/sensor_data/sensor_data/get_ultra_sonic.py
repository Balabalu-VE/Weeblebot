import rclpy
from rclpy.node import Node

from messages.msg import UltraSonicdata
from gpiozero import DistanceSensor

class UltraSonicPublisher(Node):

    def __init__(self):
        super().__init__('ultra_sonic_publisher')

        self.sensorFR = DistanceSensor(echo=21, trigger=20)
        self.sensorBR = DistanceSensor(echo=18, trigger=4)

        self.publisher_ = self.create_publisher(UltraSonicdata, 'ultra_sonic_data', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = UltraSonicdata()
         
        # Read the ultra sonic values
        msg.front_right = self.sensorFR.distance * 100
        msg.back_right = self.sensorBR.distance * 100

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    imu_publisher = UltraSonicPublisher()

    rclpy.spin(imu_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_publisher.destroy_node()
    rclpy.shutdown()

