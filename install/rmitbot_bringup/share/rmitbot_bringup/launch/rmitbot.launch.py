import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

# Launch the file
# ros2 launch rmitbot_bringup rmitbot.launch.py

def generate_launch_description():
    
    # Launch rviz
    display = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rmitbot_description"),
            "launch",
            "display.launch.py"
        ),
    )
    
    # Launch gazebo
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rmitbot_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )
    
    # Launch the controller manager
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rmitbot_controller"),
            "launch",
            "controller.launch.py"
        ),
    )
    
    # Launch the controller manager 3s after gazebo, to make sure the robot has spawned in simulation
    controller_delayed = TimerAction(
        period = 3., 
        actions=[controller]
    )
    
    # Launch the twistmux instead of keyboard node only
    twistmux = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rmitbot_navigation"),
            "launch",
            "twistmux.launch.py"
        ),
    )
    
    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rmitbot_localization"),
            "launch",
            "localization.launch.py"
        ),
    )
    
    # Launch the mapping node
    mapping = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rmitbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
    )
    
    nav2_in_new_terminal = ExecuteProcess(
    cmd=['xterm', '-hold', '-e', 'bash', '-lc',
         'source ~/rmitbot_v3/lesson5_ws/install/setup.bash; '
         'ros2 launch rmitbot_navigation nav.launch.py']
    )

    
    # # Launch the navigation 3s after slamtoolbox, to make sure that a map is available
    navigation_delayed = TimerAction(
        period = 10., 
        actions=[nav2_in_new_terminal]
    )
    
    return LaunchDescription([
        display, 
        gazebo,
        controller_delayed, 
        twistmux,
        localization, 
        mapping, 
        navigation_delayed, 
    ])