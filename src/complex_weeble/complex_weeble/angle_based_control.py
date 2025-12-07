import rclpy
from rclpy.node import Node
from messages.msg import DPad, Encdata
from dual_tb9051ftg_rpi import motors, MAX_SPEED
import math

LENGTH = 0.125  # Length of pendulum in meters
MAX_ANGLE_RAD = math.radians(22.332997294)  # Maximum angle in radians
RADIUS = 0.2794  # Wheel radius in meters
KP = 50.0  # Proportional gain
KD = 10.0  # Derivative gain
MAX_SPPED_RAD = math.sqrt(2*9.81*LENGTH*(math.cos(0) - math.cos(MAX_ANGLE_RAD))) / RADIUS

class MotorSubscriber(Node):

    def __init__(self):
        super().__init__('motor_subscriber')
        self.subscription = self.create_subscription(
            DPad,
            'dpad',
            self.get_DPAD_input,
            10)
        self.speed_subscription = self.create_subscription(
            Encdata,
            'enc_velocity',
            self.get_current_velocity,
            10)
        self.enc_speed = 0.0
        self.des_speed = 0.0

        self.angle_subscription = self.create_subscription(
            Encdata,
            'pitch',
            self.get_weeble_angle,
            10)
        self.angle = 0.0

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.subscription  # prevent unused variable warning

    def timer_callback(self):
        if(self.forward or self.back):
            control_signal = self.PD_control()
            self.move_motors(control_signal)

    def move_motors(self, control_signal):
        #Normalize desired speed to 0-1 and then multiply by MAX_SPEED
        motors.motor1.setSpeed(int(control_signal / MAX_SPPED_RAD * -MAX_SPEED)) # Motor 1 is left which is negative direction
        motors.motor2.setSpeed(int(control_signal / MAX_SPPED_RAD * MAX_SPEED))

    def forceStop(self):
        motors.motor1.setSpeed(0)
        motors.motor2.setSpeed(0)

    def get_DPAD_input(self, msg):
        self.forward = False
        self.back = False
        self.Left = False
        self.Right = False
        if(msg.dpad_up):
            self.forward = True
        elif(msg.dpad_down):
            self.back = True
        elif(msg.dpad_left):
            self.Left = True
        elif(msg.dpad_right):
            self.Right = True

    def get_current_velocity(self, msg):
        self.enc_speed = (msg.left_enc + msg.right_enc) / 2
        self.get_logger().info('Left Encoder Velocity: "%s"' % msg.left_enc)
        self.get_logger().info('Right Encoder Velocity: "%s"' % msg.right_enc)
        
    def get_weeble_angle(self, msg):
        self.angle = msg.angle
    
    def get_desired_speed(self):
        self.des_speed = math.sqrt(2*9.81*LENGTH*(math.cos(self.angle) - math.cos(MAX_ANGLE_RAD))) / RADIUS

    def PD_control(self) -> float:
        self.get_desired_speed()
        vel_error = self.des_speed - self.enc_speed
        derivative = -self.enc_speed

        control_signal = KP * vel_error + KD * derivative

        # Clamp control signal to max speed
        control_signal = max(min(control_signal, MAX_SPPED_RAD), -MAX_SPPED_RAD)

        return control_signal

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
