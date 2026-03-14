import rclpy
from rclpy.node import Node
from messages.msg import DPad, Encdata
from dual_tb9051ftg_rpi import motors, MAX_SPEED
from std_msgs.msg import Float32
import math

LENGTH = 41*0.0254  # Length of pendulum in meters
MAX_ANGLE_RAD = math.radians(22.332997294)  # Maximum angle in radians
RADIUS = 5.5*0.0254  # Wheel radius in meters
KP = .008  # Proportional gain
KD = 0.00008  # Derivative gain
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
            Float32,
            'ukf_angle',
            self.get_weeble_angle,
            10)
        self.angle = 0.0

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.absolute_speed = 0.0

        self.logged_des_vel = []
        self.logged_actual_vel = []
        self.logged_vel_error = []
        self.logged_time = []
        
        self.subscription  # prevent unused variable warning

    def timer_callback(self):
        if(self.forward or self.back):
            self.absolute_speed += self.PD_control()
            if(self.forward):
                #self.get_logger().info('Absolute Velocity: "%s"' % (self.absolute_speed))
                self.move_motors(self.absolute_speed)
            elif(self.back):
                #self.get_logger().info('Absolute Velocity: "%s"' % (self.absolute_speed))
                self.move_motors(-self.absolute_speed)
        elif(self.Left or self.Right):
            turn_speed = .5
            if(self.Left):
                motors.motor1.setSpeed(int(turn_speed * MAX_SPEED))
                motors.motor2.setSpeed(int(turn_speed * MAX_SPEED))
            elif(self.Right):
                motors.motor1.setSpeed(int(-turn_speed * MAX_SPEED))
                motors.motor2.setSpeed(int(-turn_speed * MAX_SPEED))
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
        self.angle = msg.data
    
    def get_desired_speed(self) -> float:
        self.get_logger().info(f'Weeble Angle: "{math.degrees(self.angle)}" Weeble Max Velocity Radians/s: "{0.75*math.sqrt(2*9.81*LENGTH*(math.cos(self.angle) - math.cos(MAX_ANGLE_RAD)))}')
        return 0.75*math.sqrt(2*9.81*LENGTH*(math.cos(self.angle) - math.cos(MAX_ANGLE_RAD))) / RADIUS

    def PD_control(self) -> float:
        des_speed = self.get_desired_speed()
        vel_error = des_speed - abs(self.enc_speed)
        derivative = -abs(self.enc_speed)

        control_signal = KP * vel_error + KD * derivative

        #self.get_logger().info(f'Des speed: "{des_speed}", Enc speed: "{abs(self.enc_speed)}"')
        #self.get_logger().info('Vel_error: "%s"' % vel_error)

        self.logged_des_vel.append(self.get_desired_speed()*RADIUS)
        self.logged_actual_vel.append(abs(self.enc_speed)*RADIUS)
        self.logged_vel_error.append(abs(vel_error)*RADIUS)
        self.logged_time.append(self.get_clock().now().nanoseconds * 1e-9)

        # # Clamp control signal to max speed
        #control_signal = max(min(control_signal, MAX_SPPED_RAD), -MAX_SPPED_RAD)

        return control_signal
    def destroy_node(self):
        import numpy as np
        self.get_logger().info(
            f'Destroying node and saving log...')

        np.savez("vel_log.npz",
                    des_vel=np.array(self.logged_des_vel),
                    act_vel=np.array(self.logged_actual_vel),
                    vel_error=np.array(self.logged_vel_error),
                    t=np.array(self.logged_time))

        self.get_logger().info("Saved log to vel_log.npz")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    motor_subscriber = MotorSubscriber()

    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    try:
        rclpy.spin(motor_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        motors.forceStop()
        motor_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
