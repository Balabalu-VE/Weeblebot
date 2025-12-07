import rclpy
from rclpy.node import Node
import struct
import os
from messages.msg import DPad
import time

JS_EVENT_FORMAT = "IhBB"  # uint32, int16, uint8, uint8
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)

# Axis mappings (may vary by controller)
AXIS_LEFT_Y = 1
AXIS_RIGHT_Y = 3
DPAD_UP = 7
DPAD_DOWN = 7
DPAD_LEFT = 6
DPAD_RIGHT = 6

# Store current axis values
axis_values = {}

class MotorPublisher(Node):

    def __init__(self):
        super().__init__('motor_publisher')
        self.publisher_ = self.create_publisher(DPad, 'dpad', 1)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.timer_pub = self.create_timer(timer_period, self.publish_value)
        device = "/dev/input/js0"
        self.dpad_down = 0
        self.dpad_up = 0
        self.dpad_right = 0
        self.dpad_left = 0

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
        msg = DPad()
        try:
            event = self.jsdev.read(JS_EVENT_SIZE)

            if event:
                time_ms, value, event_type, number = struct.unpack(JS_EVENT_FORMAT, event)
                #self.get_logger().info('Event Type: "%s"' % event_type)
                #self.get_logger().info('Number: "%s"' % number)
                #self.get_logger().info('value: "%s"' % value)
                number = int(number)
                value = int(value) #Cast into int to avoid issues with comparisons
                # Up is -32767, Down is 32767
                # Right is 32767, Left is -32767
                # 0x01 = JS_EVENT_BUTTON
                #self.get_logger().info('Boolean result: "%d"' % value>0)
                if event_type & 0x02:  # Axis (D-Pad) event
                    is_pressed = value != 0

                    # Reset all D-Pad fields first
                    msg.dpad_up = 0
                    msg.dpad_down = 0
                    msg.dpad_left = 0
                    msg.dpad_right = 0

                    # Vertical D-Pad
                    if number == DPAD_UP:
                        if value > 0:
                            msg.dpad_down = 1
                            #self.get_logger().info("D-Pad Down pressed")
                        elif value < 0:
                            msg.dpad_up = 1
                            #self.get_logger().info("D-Pad Up pressed")

                    # Horizontal D-Pad
                    elif number == DPAD_RIGHT:
                        if value > 0:
                            msg.dpad_right = 1
                            #self.get_logger().info("D-Pad Right pressed")
                        elif value < 0:
                            msg.dpad_left = 1
                            #self.get_logger().info("D-Pad Left pressed")

                # Publish final message
                #self.get_logger().info(f"Publishing: {msg}")
                self.publisher_.publish(msg)

    

        except (FileNotFoundError, OSError):
            print("Joystick device not found at /dev/input/js0. Is the controller connected?")
            # Controller disconnected or device missing
            print("Controller disconnected. Publishing zeros until it reconnects...")
            msg = DPad()
            msg.dpad_up = 0.0
            msg.dpad_down = 0.0
            msg.dpad_right = 0.0
            msg.dpad_left = 0.0
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
