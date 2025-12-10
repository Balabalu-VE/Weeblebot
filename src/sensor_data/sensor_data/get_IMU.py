import rclpy
from rclpy.node import Node

from messages.msg import IMUdata
import mpu6050


class IMUPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        
        self.mpu6050 = mpu6050.mpu6050(0x68)

        self.publisher_ = self.create_publisher(IMUdata, 'imu_data', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = IMUdata()
         # Read the accelerometer values
        accelerometer_data = self.mpu6050.get_accel_data()

        msg.acc_x = accelerometer_data['x']
        msg.acc_y = accelerometer_data['y']
        msg.acc_z = accelerometer_data['z']
        

        # Read the gyroscope values
        gyroscope_data = self.mpu6050.get_gyro_data()

        msg.gyro_x = gyroscope_data['x']
        msg.gyro_y = gyroscope_data['y']
        msg.gyro_z = gyroscope_data['z']

        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    imu_publisher = IMUPublisher()

    rclpy.spin(imu_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()