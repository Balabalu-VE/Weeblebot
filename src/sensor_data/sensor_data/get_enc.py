import rclpy
from rclpy.node import Node
from gpiozero import RotaryEncoder

from messages.msg import Encdata

class EncPublisher(Node):

    def __init__(self):
        super().__init__('enc_publisher')

        self.left_pos_prev = 0
        self.right_pos_prev = 0

        self.left_enc = RotaryEncoder(17, 27, max_steps=0, bounce_time=None)
        self.right_enc = RotaryEncoder(16, 19, max_steps=0, bounce_time=None)

        self.publisher_ = self.create_publisher(Encdata, 'enc_topic', 10)
        self.publisher_enc_vel = self.create_publisher(Encdata, 'enc_velocity', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0

    #16 counts per revolution
    # 100:1 gear reduction
    # 1 revolution of wheel is 1600 counts.
    # 1 revolution is 2*PI radians

    def timer_callback(self):
        msg = Encdata()
        msg_vel = Encdata()


        left_vel =(((self.left_enc.steps / 1600) * 2 * 3.14159) - self.left_pos_prev )  / (self.timer_period)  # Velocity = delta position / delta time
        right_vel =(((self.right_enc.steps  / 1600) * 2 * 3.14159) - self.right_pos_prev )  / (self.timer_period)  # Velocity = delta position / delta time

        msg_vel.left_enc = float(left_vel)
        msg_vel.right_enc = float(right_vel)

        self.publisher_enc_vel.publish(msg_vel)
        self.get_logger().info('Publishing Velocity: "%s"' % msg_vel)
        

        msg.left_enc = float((self.left_enc.steps / 1600) * 2 * 3.14159)  # Convert counts to radians
        msg.right_enc = float((self.right_enc.steps  / 1600) * 2 * 3.14159) # Convert counts to radians
        
        #Store previous positions for velocity calculation
        self.left_pos_prev = msg.left_enc
        self.right_pos_prev = msg.right_enc


        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing positon: "%s"' % msg)
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