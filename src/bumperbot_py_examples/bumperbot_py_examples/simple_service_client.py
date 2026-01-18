import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import AddTwoInts
import sys

'''
for implementation notes, refer to simple_service_server.py

Usage:
1. run the service server first
ros2 run bumperbot_py_examples simple_service_server 
[INFO] [1768732572.552146869] [simple_service_server]: Service server 'add_two_ints' started
[INFO] [1768732575.553181957] [simple_service_server]: New request recieved a: 3, b: 9
[INFO] [1768732575.553713240] [simple_service_server]: Returning response: 12



2. run the service client
ros2 run bumperbot_py_examples simple_service_client 3 9
[INFO] [1768732575.568025553] [simple_service_client]: Service response 12

If the client is ran without the server running, it will wait for the service and check every 1 second if the service is available
[output in terminal]
ros2 run bumperbot_py_examples simple_service_client 3 9
[INFO] [1768732691.191542263] [simple_service_client]: service not available, waiting again...
[INFO] [1768732692.193532324] [simple_service_client]: service not available, waiting again...
[INFO] [1768732693.195234419] [simple_service_client]: service not available, waiting again...
[INFO] [1768732694.197495951] [simple_service_client]: service not available, waiting again...

'''

class SimpleServiceClient(Node):
    def __init__(self, a, b):   # a and b will be the arguments for the service 
        super().__init__('simple_service_client')
        
        self.client_= self.create_client(AddTwoInts, "add_two_ints")
        # name of the service is "add_two_ints" where we want to make the call 

        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.request = AddTwoInts.Request()
        self.request.a = a
        self.request.b = b

        self.future_ = self.client_.call_async(self.request)
        # call_async is a non-blocking call
        # it returns a future object that we can use to check if the call is complete
        # we can use the future object to get the response and check if the call is complete
        self.future_.add_done_callback(self.response_callback)

    def response_callback(self, future):
        self.get_logger().info("Service response %d" % future.result().sum)

def main():
    rclpy.init()

    if len(sys.argv) != 3:
        print("Usage: simple_service_client <a> <b>")
        return -1
    
    node = SimpleServiceClient(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        