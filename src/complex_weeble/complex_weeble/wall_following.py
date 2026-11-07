import rclpy
from rclpy.node import Node
import math

from messages.msg import UltraSonicdata
from std_msgs.msg import String
from gpiozero import DistanceSensor
from dual_tb9051ftg_rpi import motors, MAX_SPEED

from collections import deque

import sys
import select
import termios
import tty

DESIRED_DISTANCE = .55#m
KP = 0.5
KP_dis = .6
KD = .1
KI = 0

class WallFollowingPublisher(Node):

    def __init__(self):
        super().__init__('ultra_sonic_publisher')
        
        self.subscription = self.create_subscription(
            String,
            'detected_room',
            self.detected_room_callback,
            10)

        self.subscription = self.create_subscription(
                    UltraSonicdata,
                    'ultra_sonic_data',
                    self.listener_callback,
                    10)
        
        self.BR_US = DESIRED_DISTANCE*math.sqrt(2.01)
        self.FR_US = DESIRED_DISTANCE*math.sqrt(2.01)
        self.timer_period = .1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        # self.timer = self.create_timer(self.timer_period, self.timer_callback)

        #PID Variables
        self.front_error_prev = 0
        self.back_error_prev = 0
        self.dist_error_prev = 0
        
        self.front_error_int = 0
        self.back_error_int = 0
        self.dist_error_int = 0

        self.front_history = deque(maxlen=1)
        self.back_history = deque(maxlen=1)

        self.found_room = False
        self.detected_room = ""

    def timer_callback(self):
        #Convert to meters and clamp
        front_r_45 = min(self.FR_US / 100, 0.9)
        back_r_45 = min(self.BR_US / 100, 0.9)

        # Store the newest readings
        self.front_history.append(front_r_45)
        self.back_history.append(back_r_45)

        # Compute moving average of the last 10 readings
        front_r_45 = sum(self.front_history) / len(self.front_history)
        back_r_45 = sum(self.back_history) / len(self.back_history)

        #self.get_logger().info("front_r_45 " + str(front_r_45) + "  back_r_45 " + str(back_r_45))
        
        front_error =(front_r_45 - DESIRED_DISTANCE*math.sqrt(2.01))
        back_error = (back_r_45 - DESIRED_DISTANCE*math.sqrt(2.01))
        # front_error = math.copysign(front_error**2, front_error)
        # back_error = math.copysign(back_error**2, back_error)


        #Take Root Mean Square
        dist_error = math.sqrt( (front_r_45/math.sqrt(2) - DESIRED_DISTANCE)**2 + (back_r_45/math.sqrt(2) - DESIRED_DISTANCE)**2 )
        dist_error = dist_error*((front_r_45/math.sqrt(2) - DESIRED_DISTANCE) + (back_r_45/math.sqrt(2) - DESIRED_DISTANCE))/abs(((front_r_45/math.sqrt(2) - DESIRED_DISTANCE) + (back_r_45/math.sqrt(2) - DESIRED_DISTANCE)))
        #self.get_logger().info("Distance errpr: "+ str(dist_error))

        right_speed = (0.4 - front_error*KP + back_error*KP - dist_error*KP_dis + (front_error*.3/DESIRED_DISTANCE))* MAX_SPEED *.7
        left_speed = (0.4 + front_error*KP - back_error*KP + dist_error*KP_dis + (front_error*.3/DESIRED_DISTANCE))* MAX_SPEED *.7
        
        #self.get_logger().info("Front Error: " + str(front_error))
        #self.get_logger().info("Back error: "+ str(back_error))
        #self.get_logger().info("Right speed " + str(right_speed/MAX_SPEED) + " Left speed " + str(left_speed/MAX_SPEED))
        #self.get_logger().info("MAX_SPEED: " + str(MAX_SPEED))

        if(self.detected_room == ""):
            motors.motor1.setSpeed(left_speed)
            motors.motor2.setSpeed(-right_speed)
        else:
            motors.motor1.setSpeed(0.0)
            motors.motor2.setSpeed(0.0)

            if not self.found_room:
                self.get_logger().info(
                f"Room found: '{self.detected_room}'. Press any key to continue."
                )
                self.found_room = True

            if self.keyboard_pressed():
                self.detected_room = ""
                self.found_room = False
                self.get_logger().info("Key pressed. Resuming wall following.")    
    

    # def timer_callback_PID(self):
    #     #Convert to meters and clamp
    #     front_r_45 = min(self.FR_US / 100, 0.9)
    #     back_r_45 = min(self.BR_US / 100, 0.9)

    #     # Store the newest readings
    #     self.front_history.append(front_r_45)
    #     self.back_history.append(back_r_45)

    #     # Compute moving average of the last 10 readings
    #     front_r_45 = sum(self.front_history) / len(self.front_history)
    #     back_r_45 = sum(self.back_history) / len(self.back_history)

    #     self.get_logger().info("front_r_45 " + str(front_r_45) + "  back_r_45 " + str(back_r_45))
        
    #     front_error =(front_r_45 - DESIRED_DISTANCE*math.sqrt(2.01))
    #     back_error = (back_r_45 - DESIRED_DISTANCE*math.sqrt(2.01))
    #     #front_error = math.copysign(front_error**2, front_error)
    #     #back_error = math.copysign(back_error**2, back_error)

    #     #Take Root Mean Square
    #     dist_error = math.sqrt( (front_r_45/math.sqrt(2) - DESIRED_DISTANCE)**2 + (back_r_45/math.sqrt(2) - DESIRED_DISTANCE)**2 )
    #     dist_error = dist_error*((front_r_45/math.sqrt(2) - DESIRED_DISTANCE) + (back_r_45/math.sqrt(2) - DESIRED_DISTANCE))/abs(((front_r_45/math.sqrt(2) - DESIRED_DISTANCE) + (back_r_45/math.sqrt(2) - DESIRED_DISTANCE)))
    #     self.get_logger().info("Distance errpr: "+ str(dist_error))

    #     self.front_error_int += front_error * self.timer_period
    #     self.back_error_int += back_error * self.timer_period
    #     self.dist_error_int += dist_error * self.timer_period

    #     front_term = front_error*KP + ((front_error - self.front_error_prev)/ self.timer_period)*KD + self.front_error_int*KI
    #     back_term = back_error*KP + ((back_error - self.back_error_prev)/ self.timer_period)*KD + self.back_error_int*KI
    #     dist_term = dist_error*KP_dis + ((dist_error - self.dist_error_prev)/ self.timer_period)*KD + self.dist_error_int*KI
    #     right_speed = ((0.4) - front_term + back_term - dist_term + (dist_error*.1/DESIRED_DISTANCE))* MAX_SPEED * .6
    #     left_speed = ((0.4) + front_term - back_term + dist_term + (dist_error*.1/DESIRED_DISTANCE))* MAX_SPEED * .6
        
    #     self.get_logger().info("Front Error: " + str(front_error))
    #     self.get_logger().info("Back error: "+ str(back_error))
    #     self.get_logger().info("PID Right speed " + str(right_speed/MAX_SPEED) + " Left speed " + str(left_speed/MAX_SPEED))
    #     #self.get_logger().info("MAX_SPEED: " + str(MAX_SPEED))

    #     motors.motor1.setSpeed(left_speed)
    #     motors.motor2.setSpeed(-right_speed)

    #     self.front_error_prev = front_error
    #     self.back_error_prev = back_error
    #     self.dist_error_prev = dist_error
        
    def keyboard_pressed(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.read(1)
            return True
        return False

    def detected_room_callback(self, msg):
        self.detected_room = msg.data

    def listener_callback(self, msg):
        self.FR_US = msg.front_right
        self.BR_US = msg.back_right
    def stop_motors(self): 
        motors.motor1.setSpeed(0)
        motors.motor2.setSpeed(0)

def main(args=None):
    rclpy.init(args=args)

    wall_following = WallFollowingPublisher()
    

    #Enabling terminal input
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        rclpy.spin(wall_following)
    except KeyboardInterrupt:
        pass
    finally:
        print("STOPPING MOTORS")
        wall_following.stop_motors()
        wall_following.destroy_node()
        rclpy.shutdown()
