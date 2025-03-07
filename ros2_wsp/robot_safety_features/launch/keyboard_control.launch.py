from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulated time"
    )

    # Safety Stop Node
    safety_stop_node = Node(
        package="robot_localization",
        executable="safety_stop.py",  # Ensure this matches the installed executable
        name="safety_stop_node",
        parameters=[
            {"danger_distance": 0.5},
            {"warning_distance": 1.0},
            {"angle_range": 10.0},
            {"scan_topic": "/scan"},
            {"safety_stop_topic": "/safety_stop"},
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
        output="screen",
        remappings=[
            ("/scan", "/your_lidar_scan_topic"),  # Remap LIDAR topic to actual topic name
            ("/safety_stop", "/safety_stop")     # Ensure safety stop is consistent
        ]
    )

    # Keyboard Teleoperation Node
    #keyboard_control_node = Node(
        #package="motor_control",
        #xecutable="keyboard_control",  # Matches the installed executable name
        #name="keyboard_control",
        #parameters=[
        #    {"use_sim_time": LaunchConfiguration("use_sim_time")}
        #],
        ##remappings=[
         #   ("/cmd_vel", "/keyboard_input/cmd_vel")  # Remap to relay input topic
        #],
        #output="screen"
    #)

    # Twist Relay Node
    twist_relay_node = Node(
        package="robot_localization",
        executable="twist_relay.py",  # Ensure this matches the installed executable
        name="twist_relay",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
        remappings=[
            ("/keyboard_input/cmd_vel", "/cmd_vel"),  # Forward teleop keyboard to final control
            ("/motor_control/cmd_vel_unstamped", "/cmd_vel"),  # Forward motor controller input
            ("/safety_stop", "/safety_stop")  # Ensure safety stop works consistently
        ],
        output="screen"
    )

    # Launch Description
    return LaunchDescription([
        use_sim_time_arg,
        safety_stop_node,
        #keyboard_control_node,
        twist_relay_node,
    ])
