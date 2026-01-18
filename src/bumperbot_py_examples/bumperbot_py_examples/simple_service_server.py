import rclpy
from rclpy.node import Node
'''
It is good practice to define new message types for topics and services in a separate package. The bumperbot_msgs package is created for this purpose. 

Inside the bumperbot_msgs package, we define a service called AddTwoInts. This service takes two integers as input and returns their sum as output. 
This is defined under the srv/ directory with the extension .srv. 

# Request
int64 a 
int64 b 
---
# Response
int64 sum

After making all the required changes in the package.xml and CMakeLists.txt files, we can build the package and then should be able to access the msgs.

To run this service server, we can use the following command:
ros2 run bumperbot_py_examples simple_service_server
You should see:
[INFO] [1768731333.735918079] [simple_service_server]: Service server 'add_two_ints' started

ros2 service list
[output in terminal]
/add_two_ints

ros2 service type /add_two_ints
[output in terminal]
bumperbot_msgs/srv/AddTwoInts

Now to make a call: (use tab completion)
ros2 service call /add_two_ints bumperbot_msgs/srv/AddTwoInts "a: 4
b: 9"

[output in terminal]
waiting for service to become available...
requester: making request: bumperbot_msgs.srv.AddTwoInts_Request(a=4, b=9)
bumperbot_msgs.srv.AddTwoInts_Response(sum=13)

Server should print:
[INFO] [1768731644.378989105] [simple_service_server]: New request recieved a: 4, b: 9
[INFO] [1768731644.379303833] [simple_service_server]: Returning response: 13

Next: check the simple_service_client.py file for the client implementation
'''
from bumperbot_msgs.srv import AddTwoInts

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__('simple_service_server')
        
        # Create a service server
        # here we are setting the name of the service to "add_two_ints" and the callback function to "service_callback"
        # the "add_two_ints" will be referred by the client to call the service
        self.service_ = self.create_service(AddTwoInts, "add_two_ints", self.service_callback) # create_service is from the Node class

        self.get_logger().info("Service server 'add_two_ints' started")

    def service_callback(self, request, response):
        self.get_logger().info("New request recieved a: %d, b: %d" % (request.a, request.b))
        response.sum = request.a + request.b
        self.get_logger().info("Returning response: %d" % response.sum)
        return response

def main():
    rclpy.init()
    node = SimpleServiceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


