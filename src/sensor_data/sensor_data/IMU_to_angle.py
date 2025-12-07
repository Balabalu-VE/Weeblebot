import rclpy
from rclpy.node import Node

from messages.msg import IMUdata
from std_msgs.msg import Float32
import math

class IMUToAngle(Node):
    def __init__(self):
        super().__init__('imu_to_angle')

        self.angle = 0.0
        self.last_time = None
        self.initialized = False

        self.alpha = 0.91  # Complementary filter coefficient

        self.subscription = self.create_subscription(
            IMUdata,
            'imu_data',
            self.imu_callback,
            10)
        
        self.angle_publisher = self.create_publisher(Float32, 'angle', 10)

        self.get_logger().info('IMU to Angle Node has been started.')

    def imu_callback(self, msg):
        now = self.get_clock().now()
        if not self.initialized:
            self.last_time = now
            self.initialized = True
            return

        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0:
            return

        # Calculate pitch from accelerometer
        acc_x = msg.acc_x
        acc_y = msg.acc_y
        acc_z = msg.acc_z
        pitch_acc = math.atan2(acc_x, acc_z)

        # Integrate gyroscope data to get pitch rate
        gyro_y = math.radians(msg.gyro_y)
        pitch_gyro = self.angle + gyro_y * dt

        # Complementary filter to combine both estimates
        self.angle = self.alpha * pitch_gyro + (1 - self.alpha) * pitch_acc

        # Publish the angle
        angle_msg = Float32()
        angle_msg.data = self.angle
        self.angle_publisher.publish(angle_msg)

        self.get_logger().info(f'Published Angle: {self.angle:.6f} radians')

def main(args=None):
    rclpy.init(args=args)

    imu_to_angle = IMUToAngle()

    rclpy.spin(imu_to_angle)

    imu_to_angle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()