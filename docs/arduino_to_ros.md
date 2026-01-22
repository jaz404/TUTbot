# Arduino to ROS

## Creating the firmware package
Selecting cmake as the build type so that we can use both python and cpp.
```bash
ros2 pkg create --build-type ament_cmake bumperbot_firmware
```
### Create [simple_serial_receiver.ino](../src/firmware/firmware/simple_serial_receiver/simple_serial_receiver.ino)
This code will turn on the builtin LED on the wemos d1 mini when it receives a 1 from the serial port and turn it off when it receives a 0. This is just to test the serial communication between the arduino and the ros2 node.

### Create node [simple_serial_tramsitter.py](../src/bumperbot_firmware/scripts/simple_serial_transmitter.py)
This is a simple publisher which subscribes to a ros2 topic and sends the message to the arduino via serial communication.

#### setup serial in the transmitter
pyserial requires two parameters (serial port and baudrate) to establish a connection. We can set them using ros parameters. 
```python  
# Declare parameters
self.declare_parameter('serial_port', '/dev/ttyUSB0')
self.declare_parameter('baudrate', 115200)

# Get parameters
self.serial_port_ = self.get_parameter('serial_port').value
self.baudrate_ = self.get_parameter('baudrate').value

# Serial requires port and baudrate so using parameters here
self.arduino_ = serial.Serial(self.serial_port_, self.baudrate_, timeout=0.1) 
```

#### subscribe to the serial_transmitter topic
messages published to this topic will be sent to the arduino via serial.
```python
self.sub_ = self.create_subscription(String, "serial_transmitter", self.subCallback, 10)
```

#### callback function
This function is called when a message is received on the `serial_transmitter` topic. It writes the message to the arduino via serial.
```python
def subCallback(self, msg):
    self.arduino_.write(msg.data.encode("utf-8"))
```
#### update the cmakelists.txt
to indicate we are using python add the following lines: 
```cmake
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/simple_serial_transmitter.py
  DESTINATION lib/${PROJECT_NAME}
)
```
#### update package.xml
```xml
<buildtool_depend>ament_cmake_python</buildtool_depend>
<depend>rclpy</depend>
<exec_depend>python3-serial</exec_depend>  <!-- since we are using pyserial -->
```

### Create [simple_serial_transmitter.ino](../src/firmware/firmware/simple_serial_transmitter/simple_serial_transmitter.ino)
Now we can test it other way around. Lets make the arduino publish a message to the ros2 node.
simple_serial_transmitter code sends an increasing number every second over Serial.

### Create node [simple_serial_receiver.py](../src/bumperbot_firmware/scripts/simple_serial_receiver.py)
#### Make the node publish to `serial_receiver` topic
```python
self.pub_ = self.create_publisher(String, "serial_receiver", 10)
```
#### Set the serial parameters in a similar fashion as we did above

#### Create a timer to read from the serial port
```python
self.timer_ = self.create_timer(self.frequency_, self.timerCallback) # 0.01 freq = 100 hz
```

#### Create the timer callback function
```python
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
```
#### Update cmake to include `simple_serial_receiver.py`
```cmake
install(PROGRAMS
  ${PROJECT_NAME}/simple_serial_transmitter.py
  ${PROJECT_NAME}/simple_serial_receiver.py
  DESTINATION lib/${PROJECT_NAME}
)
```
