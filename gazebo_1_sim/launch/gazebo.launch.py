import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'),
                'launch', 'gazebo.launch.py'])
        ])
    )

    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('gazebo_1_sim'), 'urdf', 'link1_urdf.urdf.xacro'
        ])
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str)
        }],
        output='screen'
    )

    # Adding a delay before spawning the entity
    robot_spawn_node = TimerAction(
        period=3.0,  # Adjust time as needed
        actions=[
            Node(
                package='gazebo_ros', 
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description',
                           '-entity', 'robot'],
                output='screen'
            )
        ]
    )
    
    # Load controllers with a delay
    load_joint_state_broadcaster = TimerAction(
        period=5.0,  # Adjust as needed for service readiness
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                     'joint_state_broadcaster'],
                output='screen'
            )
        ]
    )

    load_joint_trajectory_controller = TimerAction(
        period=6.0,  # Adjust as needed for service readiness
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                     'joint_trajectory_controller'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        robot_spawn_node,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller
    ])
