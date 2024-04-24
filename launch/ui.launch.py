from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    launch_arguments = {
        "robot_ip": "192.168.0.3",
        "use_fake_hardware": "true",
        "dof": "6",
    }
    moveit_config = (MoveItConfigsBuilder(robot_name="armstrong_urdf_v11",
                                          package_name="armstrong_moveit_config")
                                          .robot_description(mappings=launch_arguments)
                                          .trajectory_execution(file_path="config/moveit_controllers.yaml")
                                          .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
                                          .planning_pipelines(pipelines=['ompl'])
                                          .to_moveit_configs())
    movegroup_node = Node(package='moveit_ros_move_group',
                          executable='move_group',
                          output='screen',
                          parameters=[moveit_config.to_dict()]
                          )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = os.path.join(
        get_package_share_directory("armstrong_moveit_config"),
        "config",
        "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ])

    
    
    return LaunchDescription([
        movegroup_node,
        rviz_node,
        Node(
            package="frontass",
            namespace="/",
            executable="ui",
            name="ui"
        )
    ])
