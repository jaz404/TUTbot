#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from std_msgs.msg import String
import serial
'''
This node subscribes to the serial_transmitter topic and sends the message to the arduino via serial communication.
'''
class SimpleSerialTransmitter(Node):
    def __init__(self):
        super().__init__('simple_serial_transmitter')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        # Get parameters
        self.serial_port_ = self.get_parameter('serial_port').value
        self.baudrate_ = self.get_parameter('baudrate').value

        # Serial requires port and baudrate so using parameters here
        self.arduino_ = serial.Serial(self.serial_port_, self.baudrate_, timeout=0.1) 

        self.sub_ = self.create_subscription(String, "serial_transmitter", self.subCallback, 10)
        self.get_logger().info("Simple serial transmitter started")

    def subCallback(self, msg):
        self.arduino_.write(msg.data.encode("utf-8"))

def main():
    rclpy.init()
    simple_serial_transmitter = SimpleSerialTransmitter()
    rclpy.spin(simple_serial_transmitter)
    simple_serial_transmitter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()