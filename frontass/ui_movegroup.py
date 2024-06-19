from PySide6.QtWidgets import QDialog, QTableWidgetItem
from PySide6.QtCore import QTimer, Signal
from ui_assets.movegroup_ui import Ui_Dialog

import numpy as np
import pandas as pd
import datetime
from scipy.spatial.transform import Rotation

from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose
from ass_msgs.msg import PoseIteration, TrajectoryPoint, TrajectoryEnabled, TrajectoryExecution
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
from visualization_msgs.srv import GetInteractiveMarkers

from typing import List

import time


class UIMoveGroup(QDialog, Ui_Dialog):
    ik_traj_pos_changed = Signal(list)

    def __init__(self, *args, **kwargs):
        super().__init__(*args)
        self.setupUi(self)

        self.robot_position = np.zeros(16, dtype=float)
        self.robot_state = RobotState()
        self.trajectory: List[JointTrajectoryPoint] = []

        self.ik_enabled = False
        self.ikEnableBtn.toggled.connect(self.set_ik_enabled)

        self.pose_df = pd.DataFrame(np.zeros((16, 0)))

        self.timer_execution = QTimer(self)
        self.timer_execution.setInterval(10)
        self.timer_execution.timeout.connect(self.cb_execution)
        self.timer_robot_position = QTimer(self)
        self.saveTrajBtn.clicked.connect(self.save)
        self.planBtn.clicked.connect(self.plan)
        # self.execBtn.clicked.connect(self.timer_execution.start)
        self.execBtn.clicked.connect(self.cb_execution_with_traj)

        # set when execution started.
        # if execution finished or not started, then set to None
        self.time_exec_started = None

        self.node: Node = kwargs["node"]
        self.callback_group = kwargs["callback_group"]
        self.sub_traj = self.node.create_subscription(
            DisplayTrajectory,
            'display_planned_path',
            self.cb_sub_traj,
            10,
            callback_group=self.callback_group
            )
        self.pub_traj = self.node.create_publisher(
            DisplayTrajectory,
            'display_planned_path',
            10,
            callback_group=self.callback_group
            )
        self.srv_get_interactive_marker = self.node.create_client(
            GetInteractiveMarkers,
            ('rviz_moveit_motion_planning_display/'
             + 'robot_interaction_interactive_marker_topic/get_interactive_markers'),
            callback_group=self.callback_group)
        self.srv_get_motion_plan = self.node.create_client(
            GetMotionPlan,
            'plan_kinematic_path',
            callback_group=self.callback_group
            )

        self.srv_get_planning_scene = self.node.create_client(
            GetPlanningScene,
            'get_planning_scene',
            callback_group=self.callback_group
            )
        self.sub_joint_state = self.node.create_subscription(
            JointState,
            'joint_states',
            self.cb_sub_joint_state,
            10,
            callback_group=self.callback_group
            )
        self.srv_query_planner_interface = self.node.create_client(
            QueryPlannerInterfaces,
            'query_planner_interface',
            callback_group=self.callback_group
            )
        self.srv_compute_fk = self.node.create_client(
            GetPositionFK,
            'compute_fk',
            callback_group=self.callback_group
        )

        self.pose_iter_publisher = self.node.create_publisher(
            PoseIteration,
            "pose_iter",
            qos_profile_system_default,
            callback_group=self.callback_group
        )
        self.traj_point_publisher = self.node.create_publisher(
            TrajectoryPoint,
            "traj_point",
            qos_profile_system_default,
            callback_group=self.callback_group
        )
        self.planned_trajectory_publisher = self.node.create_publisher(
            MotionPlanResponse,
            "planned_trajectory",
            qos_profile_system_default,
            callback_group=self.callback_group
        )
        self.traj_exec_publisher = self.node.create_publisher(
            TrajectoryExecution,
            "traj_exec",
            qos_profile_system_default
        )
        self.traj_enabled_publisher = self.node.create_publisher(
            TrajectoryEnabled,
            "traj_enabled",
            qos_profile_system_default
        )

        res = self.srv_query_planner_interface.call(QueryPlannerInterfaces.Request())
        self.planner_interface = res.planner_interfaces
        self.planner_interface.reverse()
        pipeline_ids = [i.pipeline_id for i in self.planner_interface]

        self.planningPipelineCombo.addItems(pipeline_ids)
        self.cb_planning_pipeline_combo_changed(0)
        self.planningPipelineCombo.currentIndexChanged.connect(
            self.cb_planning_pipeline_combo_changed
            )
        self.getCurrentPoseBtn.clicked.connect(
            self.cb_get_current_pose
        )

        # for debug
        debug = False
        if debug:
            self.ik_traj_pos_changed.connect(print)

    def set_ik_enabled(self, val):
        self.ik_enabled = val
        self.traj_enabled_publisher.publish(TrajectoryEnabled(enabled=self.ik_enabled))

    def cb_sub_traj(self, data: DisplayTrajectory):
        traj: RobotTrajectory = data.trajectory[0]  # type: ignore

        pos = [point.positions for point in traj.joint_trajectory.points]

        nsec_from_start = [point.time_from_start.sec * 1e9
                           + point.time_from_start.nanosec
                           for point in traj.joint_trajectory.points]
        self.build_table(pos, nsec_from_start)
        self.execBtn.setEnabled(True)
        self.planStatusLabel.setText("PLANNED")

    def cb_sub_joint_state(self, data: JointState):
        self.robot_state.joint_state = data

    def cb_planning_pipeline_combo_changed(self, index):
        planner_ids = self.planner_interface[index].planner_ids
        planner_ids.reverse()
        self.plannerIDCombo.clear()
        self.plannerIDCombo.addItems(planner_ids)

    def cb_get_current_pose(self):
        fk_req = GetPositionFK.Request()
        fk_req.fk_link_names = ['base_link_right',
                                'link1_right',
                                'link2_right',
                                'link3_right',
                                'link4_right',
                                'link5_right',
                                'link6_right']
        fk_req.robot_state = self.robot_state
        fk_response: GetPositionFK.Response = self.srv_compute_fk.call(fk_req)
        if fk_response.error_code.val == 1:
            pose = fk_response.pose_stamped[6].pose
            r, p, y = Rotation.from_quat([pose.orientation.x,
                                          pose.orientation.y,
                                          pose.orientation.z,
                                          pose.orientation.w]).as_euler('xyz', True)
            self.xDoubleSpinBox.setValue(pose.position.x * 1000)
            self.yDoubleSpinBox.setValue(pose.position.y * 1000)
            self.zDoubleSpinBox.setValue(pose.position.z * 1000)
            self.rollDoubleSpinBox.setValue(r)
            self.pitchDoubleSpinBox.setValue(p)
            self.yawDoubleSpinBox.setValue(y)

        marker = self.srv_get_interactive_marker.call(
            GetInteractiveMarkers.Request()).markers[0]
        r, p, y = Rotation.from_quat([marker.pose.orientation.x,
                                      marker.pose.orientation.y,
                                      marker.pose.orientation.z,
                                      marker.pose.orientation.w]).as_euler('xyz', True)
        self.targetXDoubleSpinBox.setValue(marker.pose.position.x * 1000)
        self.targetYDoubleSpinBox.setValue(marker.pose.position.y * 1000)
        self.targetZDoubleSpinBox.setValue(marker.pose.position.z * 1000)
        self.targetRollDoubleSpinBox.setValue(r)
        self.targetPitchDoubleSpinBox.setValue(p)
        self.targetYawDoubleSpinBox.setValue(y)

    def get_planning_request_params(self):
        pipeline_id = self.planningPipelineCombo.currentText()
        planner_id = self.plannerIDCombo.currentText()
        return pipeline_id, planner_id

    def build_table(self, positions, nanosecs):
        point = np.rad2deg(positions)
        self.pose_df = pd.DataFrame(
            index=[
                f'L{i}' for i in range(1, 9)]+[
                    f'R{i}' for i in range(1, 9)]+["time"],
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

    def save(self):
        filename = f"traj{datetime.datetime.now().strftime('%y-%m-%d_%H-%M-%S')}.csv"
        self.pose_df.T.to_csv(filename)
        self.node.get_logger().info(f"Trajectory saved to {filename}")

    def get_target_pose(self):
        x = self.targetXDoubleSpinBox.value() / 1000
        y = self.targetYDoubleSpinBox.value() / 1000
        z = self.targetZDoubleSpinBox.value() / 1000
        position = [x, y, z]

        roll = self.targetRollDoubleSpinBox.value()
        pitch = self.targetPitchDoubleSpinBox.value()
        yaw = self.targetYawDoubleSpinBox.value()
        orientation = Rotation.from_euler(
            'xyz', [roll, pitch, yaw], degrees=True).as_quat().tolist()

        return position, orientation

    def plan(self):
        pipeline_id, planner_id = self.get_planning_request_params()
        target_position, target_orientation = self.get_target_pose()

        goal_constraint = self.construct_link_constraints(
            'link6_right', target_position, target_orientation, 0.01, 0.01)

        motion_plan_request = self.build_motion_plan(pipeline_id,
                                                     planner_id,
                                                     self.robot_state,
                                                     goal_constraint)
        result = self.srv_get_motion_plan.call(
            GetMotionPlan.Request(motion_plan_request=motion_plan_request)
            ).motion_plan_response
        if result.error_code.val == 1:
            self.planned_trajectory_publisher.publish(result)
            self.trajectory = result.trajectory.joint_trajectory.points
            self.pub_traj.publish(
                DisplayTrajectory(trajectory=[result.trajectory])
                )
            self.planStatusLabel.setText("PLANNED")
            return self.trajectory
        else:
            self.planStatusLabel.setText("PLAN FAILED")

    def cb_execution_with_traj(self):
        self.execBtn.setDisabled(True)
        self.traj_exec_publisher.publish(TrajectoryExecution(enabled=True))

    def cb_execution(self):
        if self.time_exec_started is None:
            self.time_exec_started = time.time_ns()
            self.planStatusLabel.setText("EXECUTING")

        time_passed = time.time_ns() - self.time_exec_started
        idx = int(time_passed // 1e8)

        if idx >= self.pose_df.shape[1]-1:
            self.planStatusLabel.setText("EXECUTED")
            self.time_exec_started = None
            self.timer_execution.stop()
            return

        self.update_table(self.pose_df.iloc[:, idx:])
        point = self.trajectory[idx]
        point_next = self.trajectory[idx+1]
        pos = np.array(point.positions)
        pos_next = np.array(point_next.positions)
        time_from_start = (point.time_from_start.sec * 1e9
                           + point.time_from_start.nanosec)
        time_from_start_next = (point_next.time_from_start.sec * 1e9
                                + point_next.time_from_start.nanosec)

        interpolated = (pos +
                        (pos_next - pos)
                        * (time_passed - time_from_start)
                        / (time_from_start_next - time_from_start))

        message = TrajectoryPoint()
        message.enabled = self.ik_enabled
        message.point = point
        message.point.positions = interpolated
        message.point.time_from_start.sec = int(time_passed // 1e9)
        message.point.time_from_start.nanosec = int(time_passed % 1e9)
        self.traj_point_publisher.publish(message)

    def construct_link_constraints(self,
                                   link_name: str,
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

    def build_motion_plan(self, pipeline_id, planner_id, robot_state, goal_constraint):
        self.robot_state.joint_state.velocity = np.zeros_like(
            self.robot_state.joint_state.velocity, dtype=float).tolist()
        vel_scale = self.velScaleSpinBox.value()
        acc_scale = self.accScaleSpinBox.value()
        motion_plan_req = MotionPlanRequest(
            start_state=self.robot_state,
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
