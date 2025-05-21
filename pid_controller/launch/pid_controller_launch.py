from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare PID parameter arguments with default values
    declare_kp = DeclareLaunchArgument('kp', default_value='1.0')
    declare_ki = DeclareLaunchArgument('ki', default_value='0.0')
    declare_kd = DeclareLaunchArgument('kd', default_value='0.1')

    # Use LaunchConfiguration to retrieve the values
    kp_arg = LaunchConfiguration('kp')
    ki_arg = LaunchConfiguration('ki')
    kd_arg = LaunchConfiguration('kd')

    params_file = PathJoinSubstitution([
        FindPackageShare('pid_controller'), 'config', 'parameters.yaml'
    ])

    return LaunchDescription([
        declare_kp,
        declare_ki,
        declare_kd,
        Node(
            package='pid_controller',
            executable='pid_controller_node',
            name='pid_controller',
            parameters=[{'p': kp_arg, 'i': ki_arg, 'd': kd_arg}]
        ),
        Node(
            package='pid_controller',
            executable='joint_simulator_node',
            name='joint_simulator',
            parameters=[params_file]
        ),
    ])
