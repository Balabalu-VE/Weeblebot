import rclpy
from rclpy.node import Node
import math

from messages.msg import UltraSonicdata
from gpiozero import DistanceSensor
from dual_tb9051ftg_rpi import motors, MAX_SPEED


DESIRED_DISTANCE = .5 #m
KP = .1

class WallFollowingPublisher(Node):

    def __init__(self):
        super().__init__('ultra_sonic_publisher')
        
        self.subscription = self.create_subscription(
            UltraSonicdata,
            'ultra_sonic_data',
            self.listener_callback,
            10)
        self.FR_US = DESIRED_DISTANCE*math.sqrt(2.01)
        self.BR_US = DESIRED_DISTANCE*math.sqrt(2.01)
        timer_period = .05
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        front_r_45 = self.FR_US/100 #convert to m
        back_r_45 = self.BR_US/100 #convert to m
        front_error = front_r_45 - DESIRED_DISTANCE*math.sqrt(2)
        back_error = back_r_45 - DESIRED_DISTANCE*math.sqrt(2)

        #Take Root Mean Square
        dist_error = math.sqrt( (front_r_45/math.sqrt(2) - DESIRED_DISTANCE)**2 + (back_r_45/math.sqrt(2) - DESIRED_DISTANCE)**2 )
        dist_error = dist_error*((front_r_45/math.sqrt(2) - DESIRED_DISTANCE) + (back_r_45/math.sqrt(2) - DESIRED_DISTANCE))/abs(((front_r_45/math.sqrt(2) - DESIRED_DISTANCE) + (back_r_45/math.sqrt(2) - DESIRED_DISTANCE)))
        #self.get_logger().info("Distance errpr: "+ str(dist_error))

        right_speed = (0.5 - front_error*KP + back_error*KP - dist_error*KP*2)* MAX_SPEED/2 
        left_speed = (0.5 + front_error*KP - back_error*KP +dist_error*KP*2)* MAX_SPEED/2
        
        #self.get_logger().info("Front Error: " + str(front_error))
        #self.get_logger().info("Back error: "+ str(back_error))
        self.get_logger().info("Right speed " + str(right_speed/MAX_SPEED) + " Left speed " + str(left_speed/MAX_SPEED))
        #self.get_logger().info("MAX_SPEED: " + str(MAX_SPEED))

        motors.motor1.setSpeed(right_speed)
        motors.motor2.setSpeed(left_speed)


    def listener_callback(self, msg):
        self.FR_US = msg.front_right
        self.BR_US = msg.back_right
    def stop_motors(self): 
        motors.motor1.setSpeed(0.0)
        motors.motor2.setSpeed(0.0)

def main(args=None):
    rclpy.init(args=args)

    wall_following = WallFollowingPublisher()

    rclpy.spin(wall_following)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_following.stop_motors()
    wall_following.destroy_node()
    rclpy.shutdown()
