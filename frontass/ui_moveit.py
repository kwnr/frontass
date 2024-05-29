from PySide6.QtWidgets import QDialog, QTableWidget, QTableWidgetItem
from PySide6.QtCore import QTimer, Signal
from PySide6.QtGui import QColor
from ui_assets.ik_ui import Ui_Dialog

from moveit.core.robot_state import RobotState
from moveit.core.planning_scene import PlanningScene
from moveit.core.kinematic_constraints import construct_joint_constraint, construct_link_constraint
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters, PlanRequestParameters
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python import get_package_share_directory

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from scipy.spatial.transform import Rotation

import numpy as np
import pandas as pd
import time

class UIMoveit(QDialog, Ui_Dialog):
    ik_traj_pos_changed = Signal(list)
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setupUi(self)
        path_to_moveit_config_pkg = get_package_share_directory("armstrong_moveit_config")
        moveit_config_builder = MoveItConfigsBuilder("armstrong")
        moveit_config = (
            moveit_config_builder
            .robot_description(
                file_path=path_to_moveit_config_pkg+"/config/armstrong_urdf_v11.urdf.xacro"
            )
            .robot_description_semantic(
                file_path=path_to_moveit_config_pkg+"/config/armstrong_urdf_v11.srdf"
            )
            .planning_scene_monitor(
                publish_robot_description=True,
                publish_robot_description_semantic=True
            )
            .joint_limits(
                file_path=path_to_moveit_config_pkg+"/config/joint_limits.yaml"
            )
            .pilz_cartesian_limits(
                file_path=path_to_moveit_config_pkg+"/config/pilz_cartesian_limits.yaml"
            )
            .planning_pipelines(
                pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
            )
            ).to_moveit_configs()

        moveit_config = (
            moveit_config_builder
            .moveit_cpp(
                file_path=path_to_moveit_config_pkg+"/config/moveit_cpp.yaml")
            .joint_limits(file_path=path_to_moveit_config_pkg+"/config/joint_limits.yaml"))

        self.moveit_py = MoveItPy(node_name="moveit_py", config_dict=moveit_config.to_dict())
        self.moveit_arm = self.moveit_py.get_planning_component("right_arm")
        self.arm_model = self.moveit_py.get_robot_model()
        self.planning_scene_monitor = self.moveit_py.get_planning_scene_monitor()
        self.arm_state = RobotState(self.arm_model)

        self.ik_enabled = False

        self.IKEnableBtn.toggled.connect(self.set_ik_enabled)

        self.getPoseBtn.clicked.connect(self.get_current_pose)

        self.planBtn.clicked.connect(self.plan)
        self.ExecBtn.clicked.connect(self.execute)

        self.plan_result = None
        self.start_time = 0.

        self.ik_pos_timer = QTimer(self)
        self.ik_pos_timer.setInterval(100)

        self.ik_traj_timer = QTimer(self)
        self.ik_traj_timer.setInterval(10)

        self.plan_request_params = PlanRequestParameters(self.moveit_py, "pilz_lin")

        self.multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
            self.moveit_py, ['ompl_rrtc', 'pilz_lin', 'chomp_planner']
        )

    def set_ik_enabled(self, val):
        self.ik_enabled = val
        if val:
            self.ik_traj_pos_changed.emit([True, [np.nan]*16])
        else:
            self.ik_traj_pos_changed.emit([False, [np.nan]*16])

    def get_current_pose(self):
        with self.planning_scene_monitor.read_only() as scene:
            robot_state = scene.current_state
            pose = robot_state.get_pose("goal_right")

        r, p, y = Rotation([pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w]).as_euler('xyz', True)

        self.xPosSpinBox.setValue(pose.position.x * 1000)
        self.yPosSpinBox.setValue(pose.position.y * 1000)
        self.zPosSpinBox.setValue(pose.position.z * 1000)

        self.rollOriSpinBox.setValue(r)
        self.pitchOriSpinBox.setValue(p)
        self.yawOriSpinBox.setValue(y)

    def get_tolerance(self):
        pos_tol = self.posTolSpinBox.value() / 1000
        ori_tol = np.deg2rad(self.oriTolSpinBox.value())
        return pos_tol, ori_tol

    def plan(self):
        self.moveit_arm.set_start_state_to_current_state()
        x, y, z, quat = self.get_tgt_pose()
        pos_tol, ori_tol = self.get_tolerance()

        goal = construct_link_constraint("goal_right", "base_link_right", [x, y, z], pos_tol, quat.tolist(), ori_tol)
        print(goal)
        self.moveit_arm.set_goal_state(motion_plan_constraints=[goal])
        plan_result = self.moveit_arm.plan()
        if plan_result:
            self.plan_status_label.setText("status: PLANNED")
            self.ExecBtn.setEnabled(True)
        else:
            self.plan_status_label.setText("status: PLAN FAILED")
            self.ExecBtn.setEnabled(False)

        self.plan_result = plan_result
        if plan_result:
            self.trajectory = self.plan_result.trajectory
            self.trajectory_points = self.trajectory.get_robot_trajectory_msg().joint_trajectory.points
            self.trajectory_positions = [point.positions for point in self.trajectory_points]
            self.trajectory_nsec_from_start = np.array([
                point.time_from_start.sec * 1e9 + point.time_from_start.nanosec for point in self.trajectory_points])
            self.pose_df = self.build_df(self.trajectory_positions, self.trajectory_nsec_from_start)
            self.set_table(self.pose_df)

    def execute(self):
        self.start_time = time.time_ns()
        self.plan_status_label.setText("status: EXECUTING")
        self.ik_traj_timer.start()

    def build_df(self, positions, nanosecs):
        point = np.rad2deg(positions)
        self.pose_df = pd.DataFrame(
            index=[f'L{i}' for i in range(1,9)]+[f'R{i}' for i in range(1,9)]+["time"],
            columns=[str(i) for i, p in enumerate(point)])
        self.pose_df.iloc[8:14] = np.rad2deg(positions).T
        self.pose_df.iloc[16] = nanosecs
        return self.pose_df

    def set_table(self, pose_df):
        cx = self.iter_table.columnCount() - 1
        if self.pose_df.shape[1] > cx:
            for i in range(self.pose_df.shape[1] - cx):
                self.iter_table.insertColumn(cx + i)
        elif self.pose_df.shape[1] < cx:
            for i in range(cx - self.pose_df.shape[1]):
                self.iter_table.removeColumn(cx - i)
        for col in range(pose_df.shape[1]):
            for row, pose in enumerate(pose_df.iloc[:, col]):
                if row == 16:
                    self.iter_table.setItem(
                        row, col + 1, QTableWidgetItem(f"{pose/1e9:.2f}")
                    )
                else:
                    self.iter_table.setItem(
                        row, col + 1, QTableWidgetItem(f"{pose:.2f}")
                    )
        self.tgt_pose = pose_df.iloc[:16, 0]
        self.iter_table.setHorizontalHeaderLabels(
            ["position", *pose_df.columns.astype(str)]
        )

    def get_tgt_pose(self):
        x = self.xPosSpinBox.value() / 1000
        y = self.yPosSpinBox.value() / 1000
        z = self.zPosSpinBox.value() / 1000
        roll = self.rollOriSpinBox.value()
        pitch = self.pitchOriSpinBox.value()
        yaw = self.yawOriSpinBox.value()

        quat = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_quat()
        return (x, y, z, quat)

    def update_pos_col(self, pos: np.ndarray):
        for i, p in enumerate(pos):
            item = QTableWidgetItem(f"{pos[i]:.2f}")
            self.iter_table.setItem(i, 0, item)

    def traj_timer_cb(self, joint_pos):
        print(self.pose_df.shape)
        if self.pose_df.shape[1] == 0:
            self.ik_traj_timer.stop()
            self.plan_status_label.setText("status: EXECUTED")
            self.ExecBtn.setEnabled(False)
            return

        target = self.pose_df.iloc[:, 0]
        self.progress = int(self.pose_df.columns[0]) / int(self.pose_df.columns[-1])
        item = QTableWidgetItem(f"{self.progress * 100:.2f}%")
        self.iter_table.setItem(16, 0, item)

        item = QTableWidgetItem(
            f"{time.time()-self.start_time:.2f}"
            if self.start_time != -1
            else "")
        self.iter_table.setItem(17, 0, item)

        if time.time_ns() - self.start_time > target["time"]:
            self.set_table(self.pose_df)
            self.pose_df = self.pose_df.iloc[:, 1:]
            self.ik_traj_pos_changed.emit([self.IKEnableBtn.isChecked(), target[:16].to_numpy(), None])

    def pos_timer_cb(self, joint_pos):
        self.update_pos_col(joint_pos)

