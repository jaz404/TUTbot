import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
'''
Why use parameters?
Parameters allow us to configure our nodes without recompiling the code.
They are useful for tuning values like gains, thresholds, or enabling/disabling features.
They can be changed at runtime using ros2 param set command or through parameter files.

One of the examples where I think I will be using these would be the goal pose for navigation?

Usage:
can list params using ros2 param list

$ ros2 param list
/simple_parameter:
  int_param
  start_type_description_service
  string_param
  use_sim_time

--------------------------------------------------------------------------------------------------------
Get param values:
$ ros2 param get /simple_parameter int_param
Integer value is: 42

$ ros2 param get /simple_parameter string_param
String value is: hello

--------------------------------------------------------------------------------------------------------

Setting the param value when launching the simple_parameter node using --ros-args:
$ ros2 run bumperbot_py_examples simple_parameter --ros-args -p int_param:=90 -p string_param:="world"

OR during runtime:
Setting parameter values:
$ ros2 param set /simple_parameter int_param 90
Set parameter successful

$ ros2 param set /simple_parameter string_param "world"
Set parameter successful

Running Node displays:
$ ros2 run bumperbot_py_examples simple_parameter
[INFO] [1767100353.735731861] [simple_parameter]: Simple parameter node started
[INFO] [1767100366.627212180] [simple_parameter]: Integer parameter value is now: 90

'''

class SimpleParameter(Node):
    def __init__(self):
        super().__init__("simple_parameter")
        self.get_logger().info("Simple parameter node started")

        self.declare_parameter("int_param", 42)
        self.declare_parameter("string_param", "hello")

        # set callback
        self.add_on_set_parameters_callback(self.param_change_callback)

    # Callback function that gets triggered when parameters are changed
    def param_change_callback(self, params):
        result = SetParametersResult()

        for param in params:
            # We can use the parameter type to handle different parameter types differently
            # This allows us to validate and process parameters based on their type
            if param.name == "int_param" and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info(f"Integer parameter value is now: {param.value}")
                result.successful = True
            elif param.name == "string_param" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info(f"String parameter value is now: {param.value}")
                result.successful = True

        return result

def main(args=None):
    rclpy.init(args=args)
    simple_parameter_node = SimpleParameter()
    rclpy.spin(simple_parameter_node)
    simple_parameter_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()