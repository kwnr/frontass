import pandas as pd
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (DisplayTrajectory,
                             RobotTrajectory,
                             MotionPlanRequest,
                             MotionPlanResponse,
                             RobotState,
                             Constraints,
                             PositionConstraint,
                             OrientationConstraint)
from moveit_msgs.srv import (GetMotionPlan,
                             GetPlanningScene,
                             QueryPlannerInterfaces,
                             GetPositionFK)
from typing import List
from ass_msgs.msg import ARMstrongTrajectory
from scipy.spatial.transform import Rotation

def construct_link_constraints(link_name: str,
                               target_position: List[float],
                               target_oritentaion: List[float],
                               position_tol: float,
                               orientation_tol: float):
        pose = Pose()
        pose.position.x = target_position[0]
        pose.position.y = target_position[1]
        pose.position.z = target_position[2]
        pose.orientation.x = target_oritentaion[0]
        pose.orientation.y = target_oritentaion[1]
        pose.orientation.z = target_oritentaion[2]
        pose.orientation.w = target_oritentaion[3]

        position_constraint = PositionConstraint()
        position_constraint.link_name = link_name
        position_constraint.target_point_offset.x = position_tol
        position_constraint.target_point_offset.y = position_tol
        position_constraint.target_point_offset.z = position_tol
        position_constraint.constraint_region.primitive_poses = [pose]
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.link_name = link_name
        orientation_constraint.absolute_x_axis_tolerance = orientation_tol
        orientation_constraint.absolute_y_axis_tolerance = orientation_tol
        orientation_constraint.absolute_z_axis_tolerance = orientation_tol
        orientation_constraint.orientation = pose.orientation
        orientation_constraint.weight = 1.0

        goal_constraint = Constraints()
        goal_constraint.position_constraints = [position_constraint]
        goal_constraint.orientation_constraints = [orientation_constraint]

        return goal_constraint

def build_motion_plan(pipeline_id, planner_id, robot_state, goal_constraint, vel_scale, acc_scale):
    motion_plan_req = MotionPlanRequest(
        start_state=robot_state,
        goal_constraints=[goal_constraint],
        pipeline_id=pipeline_id,
        planner_id=planner_id,
        group_name='right_arm',
        num_planning_attempts=10,
        allowed_planning_time=3.,
        max_velocity_scaling_factor=vel_scale,
        max_acceleration_scaling_factor=acc_scale,
        )
    return motion_plan_req


class Point:
    position: np.ndarray
    orientation: np.ndarray


class PointToPoint:
    name: str
    start: Point
    end: Point
    trigger: int
    duration: float


p0 = Point
p0.position = np.array([450, -290, -44])
p0.orientation = np.array([95, 0, 1])

p1 = Point
p1.position = np.array([520, -290, -44])
p1.orientation = np.array([95, 0, 1])

p2 = Point
p2.position = np.array([521, -290, -44])
p2.orientation = np.array([95, 0, 1])

p3 = Point
p3.position = np.array([522, -290, -44])
p3.orientation = np.array([95, 0, 1])

p4 = Point
p4.position = np.array([523, -290, -44])
p4.orientation = np.array([95, 0, 1])

p5 = Point
p5.position = np.array([524, -290, -44])
p5.orientation = np.array([95, 0, 1])

p6 = Point()
p6.position = np.array([525, -290, -44])
p6.orientation = np.array([95, 0, 1])

p7 = Point()
p7.position = np.array([528, -290, -44])
p7.orientation = np.array([95, 0, 1])

path0 = PointToPoint()
path0.name = "initial_to_idle"
path0.start = p0
path0.end = p1
path0.duration = 1.
path0.trigger = 500

path1 = PointToPoint()
path1.name = "head_fit_attempt_1"
path1.start = p1
path1.end = p2
path1.duration = .5
path1.trigger = 520

path2 = PointToPoint()
path2.name = "head_fit_attempt_2"
path2.start = p2
path2.end = p3
path2.duration = .5
path2.trigger = 520

path3 = PointToPoint()
path3.name = "head_fit_attempt_3"
path3.start = p3
path3.end = p4
path3.duration = .5
path3.trigger = 520

path4 = PointToPoint()
path4.name = "head_fit_attempt_4"
path4.start = p4
path4.end = p5
path4.duration = .5
path4.trigger = 520

path5 = PointToPoint()
path5.name = "head_fit_attempt_5"
path5.start = p5
path5.end = p6
path5.duration = .5
path5.trigger = 520

path6 = PointToPoint()
path6.name = "head_tighten"
path6.start = p6
path6.end = p7
path6.duration = .5
path6.trigger = 500

path7 = PointToPoint()
path7.name = "impact_on"
path7.start = p7
path7.end = p7
path7.duration = 1.
path7.trigger = 999

path7 = PointToPoint()
path7.name = "impact_off"
path7.start = p7
path7.end = p7
path7.duration = 1.
path7.trigger = 500

path7 = PointToPoint()
path7.name = "return_to_idle"
path7.start = p7
path7.end = p1
path7.duration = 5.
path7.trigger = 500