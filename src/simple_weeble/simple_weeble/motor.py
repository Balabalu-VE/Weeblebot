import rclpy
from rclpy.node import Node
from messages.msg import Motor
from dual_tb9051ftg_rpi import motors, MAX_SPEED
import struct
import os

class MotorSubscriber(Node):

    def __init__(self):
        super().__init__('motor_subscriber')
        self.subscription = self.create_subscription(
            Motor,
            'motor',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.left_motor)
        self.get_logger().info('I heard: "%s"' % msg.right_motor)
        motors.motor1.setSpeed(msg.left_motor)
        motors.motor2.setSpeed(msg.right_motor)

    def raiseIfFault():
        if motors.motor1.getFault():
            raise DriverFault(1)
        if motors.motor2.getFault():
            raise DriverFault(2)


def main(args=None):
    rclpy.init(args=args)

    motor_subscriber = MotorSubscriber()

    rclpy.spin(motor_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motors.forceStop()
    motor_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
