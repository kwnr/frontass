import numpy as np
import pandas as pd

import time

import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.qos import qos_profile_default

from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from moveit_msgs.msg import (DisplayTrajectory,
                             RobotTrajectory,
                             MotionPlanRequest,
                             MotionPlanResponse,
                             RobotState)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.srv import GetInteractiveMarkers
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene, QueryPlannerInterfaces

from ass_msgs.srv import RequestPlan
from ass_msgs.action import RequestExecution

from moveit.core.kinematic_constraints import construct_link_constraint


class MoveGroupHandler(Node):
    def __init__(self):
        super().__init__('ass_movegroup_handler')
        self.srv_get_interactive_marker = self.create_client(GetInteractiveMarkers,
                                '/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/get_interactive_markers')
        self.srv_get_motion_plan = self.node.create_client(GetMotionPlan, '/plan_kinematic_path')

        self.srv_get_planning_scene = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.sub_joint_state = self.create_subscription(JointState, '/joint_states', self.cb_sub_joint_state, 10)
        self.srv_query_planner_interface = self.create_client(QueryPlannerInterfaces, '/query_planner_interface')

        self.srv_req_plan = self.create_service(RequestPlan, "request_plan", self.cb_request_plan)

        self.action_request_execution = rclpy.action.ActionServer(self, RequestExecution, "request_execution", self.cb_execution)

        self.trajectory = JointTrajectory()

    def cb_request_plan(self):
        result = self.srv_get_interactive_marker.call(GetInteractiveMarkers.Request())
        marker = result.markers[0]

        result = self.srv_get_planning_scene.call(GetPlanningScene.Request(compunents=2))

        marker_position = [marker.pose.position.x,
                           marker.pose.position.y,
                           marker.pose.position.z]
        marker_orientation = [
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w,
        ]
        pipeline_id, planner_id = self.get_planning_request_params()

        goal_constraint = construct_link_constraint('link6_right',
                                                    source_frame='base_link_right',
                                                    cartesian_position=marker_position,
                                                    cartesian_position_tolerance=0.01,
                                                    orientation=marker_orientation,
                                                    orientation_tolerance=0.01)

        motion_plan_request = self.build_motion_plan(pipeline_id,
                                                     planner_id,
                                                     self.robot_state,
                                                     goal_constraint)

        result = self.srv_get_motion_plan.call(
            GetMotionPlan.Request(motion_plan_request=motion_plan_request)
            )
        result: MotionPlanResponse = result.motion_plan_response
        print(result)
        if result.error_code.val == 1:
            self.trajectory = result.trajectory.joint_trajectory.points
            self.pub_traj.publish(
                DisplayTrajectory(trajectory=[result.trajectory])
                )
            return RequestPlan.Response(
                result_trajectory=self.trajectory
            )
        else:
            return RequestPlan.Response(
                result_trajectory=JointTrajectoryPoint()
            )

    def cb_execution(self, goal):
        self.time_at_start = time.perf_counter_ns()

