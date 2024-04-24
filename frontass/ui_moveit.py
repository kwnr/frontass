from PySide6.QtWidgets import QDialog, QTableWidget, QTableWidgetItem
from PySide6.QtCore import QTimer, Signal
from PySide6.QtGui import QColor 
from ui_assets.ik_ui import Ui_Dialog

from moveit.core.robot_state import RobotState
from moveit.core.planning_scene import PlanningScene
from ament_index_python import get_package_share_directory

from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters
from moveit_configs_utils import MoveItConfigsBuilder

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

import numpy as np
import pandas as pd
import time

class UIMoveit(QDialog, Ui_Dialog):
    ik_traj_pos_changed = Signal(list)        
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setupUi(self)
        moveit_config_builder = MoveItConfigsBuilder("armstrong")
        moveit_config_builder.moveit_cpp(
            file_path=get_package_share_directory("armstrong_moveit_config")+"/config/moveit_cpp.yaml"
            ).joint_limits(
            file_path=get_package_share_directory("armstrong_moveit_config")
            + "/config/joint_limits.yaml"
        )
        self.armstrong = MoveItPy(
            node_name="moveit_py", config_dict=moveit_config_builder.to_dict()
        )
        self.moveit_arm = self.armstrong.get_planning_component("right_arm")
        self.arm_model = self.armstrong.get_robot_model()
        self.planning_scene_monitor = self.armstrong.get_planning_scene_monitor()
        self.arm_state = RobotState(self.arm_model)
        
        self.multi_pipeline_plan_request_param = MultiPipelinePlanRequestParameters(
            self.armstrong, ['ompl', 'pilz_industrial_motion_planner'])

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

        self.xCurPosLineEdit.setText(f"{pose.position.x:.4f}")
        self.yCurPosLineEdit.setText(f"{pose.position.y:.4f}")
        self.zCurPosLineEdit.setText(f"{pose.position.z:.4f}")

        self.xCurOriLineEdit.setText(f"{pose.orientation.x:.4f}")
        self.yCurOriLineEdit.setText(f"{pose.orientation.y:.4f}")
        self.zCurOriLineEdit.setText(f"{pose.orientation.z:.4f}")
        self.wCurOriLineEdit.setText(f"{pose.orientation.w:.4f}")

    def plan(self):
        self.moveit_arm.set_start_state_to_current_state()
        goal = PoseStamped()
        goal.header.frame_id = "base_link_right"

        goal.pose.position.x = float(self.xCurPosLineEdit.text())
        goal.pose.position.y = float(self.yCurPosLineEdit.text())
        goal.pose.position.z = float(self.zCurPosLineEdit.text())

        goal.pose.orientation.x = float(self.xCurOriLineEdit.text())
        goal.pose.orientation.y = float(self.yCurOriLineEdit.text())
        goal.pose.orientation.z = float(self.zCurOriLineEdit.text())
        goal.pose.orientation.w = float(self.wCurOriLineEdit.text())

        self.moveit_arm.set_goal_state(pose_stamped_msg=goal, pose_link="goal_right")
        plan_result = self.moveit_arm.plan(multi_plan_parameters=self.multi_pipeline_plan_request_param)
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
        self.pose_df = pd.DataFrame(index=[f'L{i}' for i in range(1,9)]+[f'R{i}' for i in range(1,9)]+["time"], columns=[str(i) for i, p in enumerate(point)])
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

