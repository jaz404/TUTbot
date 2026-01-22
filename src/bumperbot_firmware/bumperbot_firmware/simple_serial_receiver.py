#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from std_msgs.msg import String
import serial
'''
This node publishes to serial_receiver topic that it recieves from arduino via serial communication.
'''
class SimpleSerialReceiver(Node):
    def __init__(self):
        super().__init__('simple_serial_receiver') ## name for the node

        self.pub_ = self.create_publisher(String, "serial_receiver", 10)

        self.frequency_ = 0.01 # 100hz

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        # Get parameters
        self.serial_port_ = self.get_parameter('serial_port').value
        self.baudrate_ = self.get_parameter('baudrate').value

        # Serial requires port and baudrate so using parameters here
        self.arduino_ = serial.Serial(self.serial_port_, self.baudrate_) 

        self.timer_ = self.create_timer(self.frequency_, self.timerCallback) # 0.01 freq = 100 hz

    def timerCallback(self):
        if rclpy.ok() and self.arduino_.isOpen():
            data = self.arduino_.readline()
            # publish the message
            try: # try to decode with utf-8
                data = data.decode("utf-8").strip()
            except:
                return

            msg = String()
            msg.data = data
            self.pub_.publish(msg)
    

def main():
    rclpy.init()
    simple_serial_receiver = SimpleSerialReceiver()
    rclpy.spin(simple_serial_receiver)
    simple_serial_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()