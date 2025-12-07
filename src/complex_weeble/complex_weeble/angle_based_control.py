import rclpy
from rclpy.node import Node
from messages.msg import DPad, Encdata
from dual_tb9051ftg_rpi import motors, MAX_SPEED
import math

LENGTH = 41*0.0254  # Length of pendulum in meters
MAX_ANGLE_RAD = math.radians(22.332997294)  # Maximum angle in radians
RADIUS = 5.5*0.0254  # Wheel radius in meters
KP = .005  # Proportional gain
KD = 0.0001  # Derivative gain
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

        self.forward = False
        self.back = False
        self.Left = False
        self.Right = False

        self.angle_subscription = self.create_subscription(
            Encdata,
            'pitch',
            self.get_weeble_angle,
            10)
        self.angle = 0.0

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.absolute_speed = 0.0
        
        self.subscription  # prevent unused variable warning

    def timer_callback(self):
        if(self.forward or self.back):
            self.absolute_speed += self.PD_control()
            if(self.forward):
                self.get_logger().info('Absolute Velocity: "%s"' % (self.absolute_speed))
                self.move_motors(self.absolute_speed)
            elif(self.back):
                self.get_logger().info('Absolute Velocity: "%s"' % (self.absolute_speed))
                self.move_motors(-self.absolute_speed)
        else:
            self.absolute_speed = 0.0
            self.move_motors(0)

    def move_motors(self, control_signal):
        #Normalize desired speed to 0-1 and then multiply by MAX_SPEED
        #self.get_logger().info('Left Encoder Velocity: "%s"' % int(control_signal / MAX_SPPED_RAD))
        #self.get_logger().info('Right Encoder Velocity: "%s"' % int(control_signal / MAX_SPPED_RAD * MAX_SPEED))
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
        self.enc_speed = (msg.left_enc) 
        #self.get_logger().info('Left Encoder Velocity: "%s"' % msg.left_enc)
        #self.get_logger().info('Right Encoder Velocity: "%s"' % msg.right_enc)
        
    def get_weeble_angle(self, msg):
        self.angle = msg.angle
    
    def get_desired_speed(self) -> float:
        return math.sqrt(2*9.81*LENGTH*(math.cos(self.angle) - math.cos(MAX_ANGLE_RAD))) / RADIUS

    def PD_control(self) -> float:
        des_speed = self.get_desired_speed()
        vel_error = des_speed - abs(self.enc_speed)
        derivative = -abs(self.enc_speed)

        control_signal = KP * vel_error + KD * derivative

        self.get_logger().info(f'Des speed: "{des_speed}", Enc speed: "{abs(self.enc_speed)}"')
        #self.get_logger().info('Control signal: "%s"' % control_signal)

        # # Clamp control signal to max speed
        # control_signal = max(min(control_signal, MAX_SPPED_RAD), -MAX_SPPED_RAD)

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
