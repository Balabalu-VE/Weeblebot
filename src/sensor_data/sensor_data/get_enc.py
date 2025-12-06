import rclpy
from rclpy.node import Node
from gpiozero import RotaryEncoder

from messages.msg import Encdata

class EncPublisher(Node):

    def __init__(self):
        super().__init__('enc_publisher')

        self.left_enc = RotaryEncoder(17, 27, max_steps=0, bounce_time=None)
        self.right_enc = RotaryEncoder(16, 19, max_steps=0, bounce_time=None)

        self.publisher_ = self.create_publisher(Encdata, 'enc_topic', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Encdata()

        msg.left_enc = float(self.left_enc.steps)
        msg.right_enc = float(self.right_enc.steps)

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    enc_publisher = EncPublisher()

    rclpy.spin(enc_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    enc_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()