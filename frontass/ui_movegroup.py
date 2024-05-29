from PySide6.QtWidgets import QDialog, QTableWidget, QTableWidgetItem, QFileDialog
from PySide6.QtCore import QTimer, Signal
from PySide6.QtGui import QColor
from ui_assets.movegroup_ui import Ui_Dialog

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from moveit_msgs.msg import (DisplayTrajectory,
                             RobotTrajectory,
                             MotionPlanRequest,
                             MotionPlanResponse,
                             PlanningScene,
                             PlanningSceneComponents,
                             RobotState)
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene
from moveit.core.kinematic_constraints import construct_link_constraint

from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from visualization_msgs.srv import GetInteractiveMarkers
import time

import pandas as pd

class UIMoveGroup(QDialog, Ui_Dialog):
    ik_traj_pos_changed = Signal(list)
    def __init__(self, *args, **kwargs):
        super().__init__(*args)
        self.setupUi(self)

        self.robot_position = np.zeros(16, dtype=float)

        self.ik_enabled = False
        self.ikEnableBtn.toggled.connect(self.set_ik_enabled)

        self.pose_df = pd.DataFrame(np.zeros((16, 0)))

        self.timer_execution = QTimer(self)
        self.timer_robot_position = QTimer(self)
        self.timer_execution.timeout.connect(self.cb_execution)
        self.saveTrajBtn.clicked.connect(self.save)
        self.planBtn.clicked.connect(self.plan)
        self.execBtn.clicked.connect(self.timer_execution.start)

        self.node: Node = kwargs.get('node')
        self.sub_traj = self.node.create_subscription(DisplayTrajectory, '/display_planned_path', self.cb_sub_traj, 10)
        self.pub_traj = self.node.create_publisher(DisplayTrajectory, '/display_planned_path', 10)

        self.srv_get_interactive_marker = self.node.create_client(GetInteractiveMarkers,
                                '/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/get_interactive_markers')
        self.srv_get_motion_plan = self.node.create_client(GetMotionPlan, '/plan_kinematic_path')

        self.srv_get_planning_scene = self.node.create_client(GetPlanningScene, '/get_planning_scene')
        self.sub_joint_state = self.node.create_subscription(JointState, '/joint_states', self.cb_sub_joint_state, 10)
        self.robot_state = RobotState()

        ### for debug
        debug = True
        if debug:
            self.ik_traj_pos_changed.connect(print)

    def set_ik_enabled(self, val):
        self.ik_enabled = val
        if val:
            self.ik_traj_pos_changed.emit([True, [np.nan]*16])
        else:
            self.ik_traj_pos_changed.emit([False, [np.nan]*16])

    def cb_sub_traj(self, data: DisplayTrajectory):
        traj: RobotTrajectory = data.trajectory[0]
        traj_start = data.trajectory_start

        pos = [point.positions for point in traj.joint_trajectory.points]
        nsec_from_start = [point.time_from_start.sec * 1e9
                           + point.time_from_start.nanosec
                           for point in traj.joint_trajectory.points]
        self.build_table(pos, nsec_from_start)
        self.execBtn.setEnabled(True)
        self.planStatusLabel.setText("PLANNED")

    def cb_sub_joint_state(self, data: JointState):
        self.robot_state.joint_state = data

    def build_table(self, positions, nanosecs):
        point = np.rad2deg(positions)
        self.pose_df = pd.DataFrame(
            index=[f'L{i}' for i in range(1,9)]+[f'R{i}' for i in range(1,9)]+["time"],
            columns=[str(i) for i, p in enumerate(point)])
        self.pose_df.iloc[8:14] = np.rad2deg(positions).T
        self.pose_df.iloc[16] = nanosecs
        return self.pose_df

    def update_table(self, pose_df):
        cx = self.iterTable.columnCount() - 1
        if self.pose_df.shape[1] > cx:
            for i in range(self.pose_df.shape[1] - cx):
                self.iterTable.insertColumn(cx + i)
        elif self.pose_df.shape[1] < cx:
            for i in range(cx - self.pose_df.shape[1]):
                self.iterTable.removeColumn(cx - i)
        for col in range(pose_df.shape[1]):
            for row, pose in enumerate(pose_df.iloc[:, col]):
                if row == 16:
                    self.iterTable.setItem(
                        row, col + 1, QTableWidgetItem(f"{pose/1e9:.2f}")
                    )
                else:
                    self.iterTable.setItem(
                        row, col + 1, QTableWidgetItem(f"{pose:.2f}")
                    )
        if self.pose_df.shape[1] != 0:
            self.tgt_pose = pose_df.iloc[:16, 0]
        self.iterTable.setHorizontalHeaderLabels(
            ["position", *pose_df.columns.astype(str)]
        )

    def cb_movegroup(self, position):
        self.robot_position = position
        for i, pos in enumerate(position):
            item = QTableWidgetItem(f"{pos:.2f}")
            self.iterTable.setItem(i, 0, item)
        self.update_table(self.pose_df)

    def save(self, ):
        pass

    def plan(self):
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

        goal_constraint = construct_link_constraint('link6_right',
                                                    source_frame='base_link_right',
                                                    cartesian_position=marker_position,
                                                    cartesian_position_tolerance=0.01,
                                                    orientation=marker_orientation,
                                                    orientation_tolerance=0.01)
        # TODO: adapt pilz lin planner for linear path request
        motion_plan_req = MotionPlanRequest(
            start_state=self.robot_state,
            goal_constraints=[goal_constraint],
            pipeline_id='ompl',
            group_name='right_arm',
            num_planning_attempts=10,
            allowed_planning_time=3.,
            max_velocity_scaling_factor=1.0,
            max_acceleration_scaling_factor=1.0,
        )

        result = self.srv_get_motion_plan.call(
            GetMotionPlan.Request(motion_plan_request=motion_plan_req)
            )
        result: MotionPlanResponse = result.motion_plan_response
        if result.error_code.val == 1:
            self.pub_traj.publish(DisplayTrajectory(
                trajectory=result.trajectory,
                trajectory_start=result.trajectory_start,
            ))
            self.planStatusLabel.setText("PLANNED")
        else:
            self.planStatusLabel.setText("PLAN FAILED")




    def cb_execution(self):
        self.planStatusLabel.setText("EXECUTING")

        if self.pose_df.shape[1] == 2:
            self.planStatusLabel.setText("EXECUTED")
            self.timer_execution.stop()

        dt = self.pose_df.iloc[16, 1] - self.pose_df.iloc[16, 0]
        t_start = time.time_ns()
        while time.time_ns() - t_start < dt:
            pass

        latest_pos = self.pose_df.pop(self.pose_df.columns[0])
        if self.loopCheckBox.isChecked():
            self.pose_df = pd.concat([self.pose_df, latest_pos], axis=1)

        self.update_table(self.pose_df)
        self.ik_traj_pos_changed.emit([self.ik_enabled, self.pose_df.iloc[:16, 0].to_numpy()])

