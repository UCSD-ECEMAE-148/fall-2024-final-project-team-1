# ball_tracking/launch/ball_tracking_test.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

       # UNCOMMENT TO ACTIVATE CV BALL TRACKING
       # Node(
       #     package='ball_vision_package',
       #     executable='ball_tracker',
       #     name='ball_tracker_node',
       #     output='screen'
       # ),

       # UNCOMMENT TO ACTIVATE YOLO BALL TRACKING
       Node(
            package='ball_vision_package',
            executable='yolo_tracker',  # This should match the name before the = in your entry_point
            name='yolo_node',
            output='screen'
        ),

#        VESC node commented out for testing
        Node(
            package='ucsd_robocar_actuator2_pkg',
            executable='vesc_twist_node',
            name='vesc_twist_node',
            output='screen'
        )
    ])
