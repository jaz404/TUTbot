import time
import rclpy
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from std_msgs.msg import String
'''
Usage:
LIST ALL LIFECYCLE NODES:
ros2 lifecycle nodes
/simple_lifecycle_node

CURRENT STATE:
ros2 lifecycle get /simple_lifecycle_node
unconfigured [1]

AVAILABLE TRANSITIONS:
ros2 lifecycle list /simple_lifecycle_node
- configure [1]
	Start: unconfigured
	Goal: configuring
- shutdown [5]
	Start: unconfigured
	Goal: shuttingdown

SET TRANSITION:
ros2 lifecycle set /simple_lifecycle_node 1
Transitioning successful

Output on the node terminal:
[INFO] [1770014027.078593031] [simple_lifecycle_node]: Lifecycle node on_configure() called.

Now you should also see the /chatter topic in ros2 topic list

ACTIVATE NODE:
ros2 lifecycle set /simple_lifecycle_node activate
Transitioning successful

ros2 lifecycle list /simple_lifecycle_node
- cleanup [2]
	Start: inactive
	Goal: cleaningup
- activate [3]
	Start: inactive
	Goal: activating
- shutdown [6]
	Start: inactive
	Goal: shuttingdown

ros2 lifecycle set /simple_lifecycle_node 3
Transitioning successful

'''

class SimpleLifecycleNode(Node):

    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.sub_ = self.create_subscription(String, "chatter", self.msgCallback, 10)

        self.get_logger().info("Lifecycle node on_configure() called.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle node on_activate() called.")
        time.sleep(2)
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle node on_deactivate() called.")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_subscription(self.sub_)
        self.get_logger().info("Lifecycle node on_cleanup() called.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.destroy_subscription(self.sub_)

        self.get_logger().info("Lifecycle node on_shutdown() called")
        return TransitionCallbackReturn.SUCCESS
    
    def msgCallback(self, msg):
        current_state = self._state_machine.current_state
        if(current_state[1] == "active"):
            self.get_logger().info("I heard: %s" % msg.data)

def main():
    rclpy.init()

    executor = rclpy.executors.SingleThreadedExecutor()
    simple_lifecycle_node = SimpleLifecycleNode('simple_lifecycle_node')
    executor.add_node(simple_lifecycle_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        simple_lifecycle_node.destroy_node()


if __name__ == '__main__':
    main()
