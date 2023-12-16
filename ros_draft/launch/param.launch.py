from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='parameter_py',
            executable='param_node',
            name='custom_minimal_param_node',

            # print output to terminal
            output='screen',
            emulate_tty=True,

            #change parameter value
            parameters=[
                {'my_parameter': 'earth'}
            ]
        )
    ])