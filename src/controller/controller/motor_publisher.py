import rclpy
from rclpy.node import Node
from dual_tb9051ftg_rpi import motors, MAX_SPEED
import struct
import os
from messages.msg import Motor
import time

JS_EVENT_FORMAT = "IhBB"  # uint32, int16, uint8, uint8
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)

# Axis mappings (may vary by controller)
AXIS_LEFT_Y = 1
AXIS_RIGHT_Y = 3

# Store current axis values
axis_values = {}

class MotorPublisher(Node):

    def __init__(self):
        super().__init__('motor_publisher')
        self.publisher_ = self.create_publisher(Motor, 'motor', 1)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        device = "/dev/input/js0"

        # Wait until the controller appears
        while not os.path.exists(device):
            print(f"Waiting for controller at {device}...")
            time.sleep(1)

        # Once available, open it
        self.jsdev = open(device, 'rb')
        print(f"Controller connected. Reading from {device}...")   


    def timer_callback(self):
        self.read_joystick()

    def read_joystick(self):
        try:     
                event = self.jsdev.read(JS_EVENT_SIZE)
                
                if event:
                    time_ms, value, event_type, number = struct.unpack(JS_EVENT_FORMAT, event)

                    if event_type & 0x02:  # 0x02 = JS_EVENT_AXIS
                        axis_values[number] = value / 32767.0  # Normalize

                        left_y = axis_values.get(AXIS_LEFT_Y, 0.0)
                        right_y = axis_values.get(AXIS_RIGHT_Y, 0.0)
                
                        #while True:
                        
                        #print(f"Left Y: {left_y:.3f} | Right Y: {right_y:.3f}", end="\r")
                        msg = Motor()
                        msg.left_motor = left_y*MAX_SPEED*-1
                        msg.right_motor = right_y*MAX_SPEED
                        self.publisher_.publish(msg)

        except (FileNotFoundError, OSError):
            print("Joystick device not found at /dev/input/js0. Is the controller connected?")
            # Controller disconnected or device missing
            print("Controller disconnected. Publishing zeros until it reconnects...")
            msg = Motor()
            msg.left_motor = 0.0
            msg.right_motor = 0.0
            self.publisher_.publish(msg)

            # Try to reconnect
            import os, time
            while not os.path.exists("/dev/input/js0"):
                time.sleep(1)
            try:
                self.jsdev = open("/dev/input/js0", 'rb')
                print("Controller reconnected.")
            except Exception as e:
                print(f"Error reopening controller: {e}")
        except KeyboardInterrupt:
            print("\nExiting.")

def main(args=None):
    rclpy.init(args=args)

    motor_publisher = MotorPublisher()

    rclpy.spin(motor_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
