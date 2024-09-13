from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz_2d_overlay_plugins',
            executable='string_to_overlay_text',
            output='screen',
            parameters=[
                {"string_topic": "control_state"},
                {"fg_color": "b"}, # colors can be: r,g,b,w,k,p,y (red,green,blue,white,black,pink,yellow)
            ],
        ),
        Node(
            package='rviz_2d_overlay_plugins',
            executable='string_to_overlay_text',
            output='screen',
            parameters=[
                {"string_topic": "kozepiskola"},
                {"fg_color": "g"}, # colors can be: r,g,b,w,k,p,y (red,green,blue,white,black,pink,yellow)
            ],
        ),
    ])