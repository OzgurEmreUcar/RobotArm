import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("robot", package_name="robot_moveit_config").to_moveit_configs()

    move_group_params = moveit_config.to_dict()
    move_group_params["trajectory_execution.allowed_execution_duration_scaling"] = 0.0
    move_group_params["trajectory_execution.allowed_goal_duration_margin"] = 0.0
    move_group_params["trajectory_execution.execution_duration_monitoring"] = False 

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[move_group_params], 
    )


    rviz_config_file = os.path.join(moveit_config.package_path, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )


    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )


    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    trajectory_driver_node = Node(
        package="robot_description",
        executable="trajectory_driver_node.py",
        name="trajectory_driver_node",
        output="screen"
    )
    robot_jog_gui_node = Node(
        package='robot_description', 
        executable='robot_jog_gui.py',
        name='robot_jog_gui',
        output='screen'
    )

    return LaunchDescription([
        static_tf,
        robot_state_publisher,
        move_group_node,
        rviz_node,
        trajectory_driver_node,
        robot_jog_gui_node  
    ])