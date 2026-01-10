from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

import os


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(get_package_share_directory('bumperbot_description'), 'urdf', 'bumperbot.urdf.xacro'), # the shared directory path was set in CMakeLists.txt
        description='Absolute path to the URDF file'
    )
    # We can use the above declared model and pass it to the robot_description using LaunchConfiguration
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',            # Show RViz output in terminal
        arguments=['-d', os.path.join(get_package_share_directory('bumperbot_description'), 'rviz', 'display.rviz')] # this is the rviz config file which we saved after setting all the required settings (steps 3.1 to 3.3)
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])