from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the TurtleBot3 Gazebo world
        Node(
            package='turtlebot3_gazebo',
            executable='turtlebot3_world.launch',
            name='turtlebot3_gazebo',
            output='screen',
            # You might need to use an additional `arguments` list if required
            arguments=['--use_sim_time:=True']
        ),
        
        # # Launch the TurtleBot3 Keyboard Teleop node
        # Node(
        #     package='turtlebot3_teleop',
        #     executable='teleop_keyboard',
        #     name='turtlebot3_teleop',
        #     output='screen',
        # ),
        
        # Launch the SLAM Toolbox node
        Node(
            package='slam_toolbox',
            executable='online_async_launch.py',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # Launch the TurtleBot3 Navigation node
        Node(
            package='turtlebot3_navigation2',
            executable='navigation2.launch.py',
            name='turtlebot3_navigation2',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # Launch the Explorer node
        Node(
            package='my_explorer',
            executable='explorer',
            name='explorer_node',
            output='screen',
        ),
    ])
