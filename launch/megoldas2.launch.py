import os
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    follow_the_gap = Node(
        package='megoldas_sim24',
        executable='follow_the_gap.py',
        output='screen',
        parameters=[
            {
                # 'kozepiskola_neve': "Nem definialt kozepiskola",
                # 'kozepiskola_azonosito': "A00",
                # 'angle_range': 360,
                # 'velocity': 1.00,
                # 'car_length': 0.445,
                # 'wheelbase': 0.3187,
                # 'map_frame': 'odom_combined', ## not uses yet
                # 'laser_frame': 'laser',
                # 'base_frame': 'base_link',
            }
        ]
    )

    start_rviz_2d_overlay = False

    # Check if the package exists
    try:
        package_path = get_package_prefix('rviz_2d_overlay_plugins')

        str_overlay = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
            FindPackageShare("megoldas_sim24"), '/string_rviz_overlay.launch.py'])
        )
        start_rviz_2d_overlay = True
    except:
        print("rviz_2d_overlay_plugins package not found, skipping...")
        start_rviz_2d_overlay = False


    if start_rviz_2d_overlay:
        return LaunchDescription([
            follow_the_gap,
            str_overlay
        ])
    else:
        # Log or handle the case where the file does not exist
        return LaunchDescription([
            follow_the_gap,
            LogInfo(msg="Error: rviz_2d_overlay_plugins not found, skipping"),  # Log a message instead of crashing
            LogInfo(msg="Install: sudo apt install ros-humble-rviz-2d-overlay*"),
        ])