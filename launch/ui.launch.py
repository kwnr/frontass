from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    launch_arguments = {
        "robot_ip": "192.168.0.3",
        "use_fake_hardware": "False",
        "dof": "6",
    }
    moveit_config = (
        MoveItConfigsBuilder(robot_name="armstrong_urdf_v11",
                             package_name="armstrong_moveit_config")
        .robot_description(mappings=launch_arguments)
        .robot_description_semantic(file_path="config/armstrong_urdf_v11.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=['ompl', 'pilz_industrial_motion_planner'])
        .to_moveit_configs()
        )
    movegroup_node = Node(package='moveit_ros_move_group',
                          executable='move_group',
                          namespace='mark4',
                          output='screen',
                          parameters=[moveit_config.to_dict()]
                          )
    rviz_config = os.path.join(
        get_package_share_directory("armstrong_moveit_config"),
        "config",
        "moveit.rviz"
        )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace='mark4',
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ])
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        namespace="mark4",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link_right"],
    )
    """robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )"""
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("armstrong_moveit_config"), '/launch/rsp.launch.py']
        )
    )


    return LaunchDescription([
        movegroup_node,
        rviz_node,
        static_tf,
        robot_state_publisher,
        Node(
            package="frontass",
            namespace="/mark4",
            executable="ui",
            name="ui"
        )
    ])
