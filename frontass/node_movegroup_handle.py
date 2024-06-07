import numpy as np
import pandas as pd

import time

import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.qos import qos_profile_default
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

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

from threading import Thread

class MoveGroupHandler(Node):
    def __init__(self):
        super().__init__('ass_movegroup_handler')

        self.trajectory = JointTrajectory()
        self.robot_state = RobotState()
        self.callback_group = ReentrantCallbackGroup()
        self.srv_get_interactive_marker = self.create_client(GetInteractiveMarkers,
                                '/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/get_interactive_markers',
                                callback_group=self.callback_group)
        self.srv_get_motion_plan = self.create_client(
            GetMotionPlan,
            '/plan_kinematic_path',
            callback_group=self.callback_group)

        self.srv_get_planning_scene = self.create_client(
            GetPlanningScene,
            '/get_planning_scene',
            callback_group=self.callback_group)
        self.sub_joint_state = self.create_subscription(
            JointState,
            '/joint_states',
            self.cb_sub_joint_state,
            10)
        self.srv_query_planner_interface = self.create_client(
            QueryPlannerInterfaces,
            '/query_planner_interface',
            callback_group=self.callback_group)

        self.srv_req_plan = self.create_service(
            RequestPlan,
            "/request_plan",
            self.cb_request_plan,
            callback_group=self.callback_group)

        self.action_request_execution = rclpy.action.ActionServer(
            self,
            RequestExecution,
            "/request_execution",
            self.cb_execution,
            callback_group=self.callback_group)

    def cb_sub_joint_state(self, data: JointState):
        self.robot_state.joint_state = data

    def cb_request_plan(self, request: RequestPlan.Request, response: RequestPlan.Response):
        self.get_logger().info("PLANNING REQUEST RECEIVED")
        result_get = self.srv_get_interactive_marker.call(GetInteractiveMarkers.Request())
        marker = result_get.markers[0]

        marker_position = [marker.pose.position.x,
                           marker.pose.position.y,
                           marker.pose.position.z]
        marker_orientation = [
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w,
        ]
        pipeline_id = request.pipeline_id
        planner_id = request.planner_id

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
            self.get_logger().info("PLANNED SUCCESSFULLY")
        else:
            self.get_logger().warn("PLAN FAILED")

        response.response = result
        return response

    def cb_execution(self, goal):
        self.time_at_start = time.perf_counter_ns()

    def build_motion_plan(self, pipeline_id, planner_id, robot_state, goal_constraint, **kwargs):
        self.robot_state.joint_state.velocity = np.zeros_like(self.robot_state.joint_state.velocity, dtype=float)
        if planner_id == 'LIN':
            motion_plan_req = MotionPlanRequest(
                start_state=self.robot_state,
                goal_constraints=[goal_constraint],
                pipeline_id=pipeline_id,
                planner_id=planner_id,
                group_name='right_arm',
                num_planning_attempts=10,
                allowed_planning_time=3.,
                max_velocity_scaling_factor=0.1,
                max_acceleration_scaling_factor=0.1,
                )
        else:
            motion_plan_req = MotionPlanRequest(
                start_state=self.robot_state,
                goal_constraints=[goal_constraint],
                pipeline_id=pipeline_id,
                planner_id=planner_id,
                group_name='right_arm',
                num_planning_attempts=10,
                allowed_planning_time=3.,
                max_velocity_scaling_factor=1.0,
                max_acceleration_scaling_factor=0.75,
            )
        return motion_plan_req


def main():
    rclpy.init(domain_id=2)
    movegroup_handle = MoveGroupHandler()
    executer = MultiThreadedExecutor(4)
    executer.add_node(movegroup_handle)
    try:
        executer.spin()
    except KeyboardInterrupt:
        movegroup_handle.get_logger().info('Keyboard interrupt, shutting down.\n')
    finally:
        movegroup_handle.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()