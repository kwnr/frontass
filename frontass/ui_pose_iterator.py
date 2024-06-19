import pandas as pd
import numpy as np
import time

from rclpy.node import Node
from PySide6.QtCore import QTimer
from PySide6.QtGui import QColor
from PySide6.QtWidgets import QDialog, QFileDialog, QTableWidgetItem
from ui_assets.pose_iterator_diag_ui import Ui_Dialog
from ass_msgs.msg import ARMstrongTrajectory, ARMstrongTrajectoryEnabled, ARMstrongTrajectoryExecution
from trajectory_msgs.msg import JointTrajectoryPoint


class UIPoseIterator(QDialog, Ui_Dialog):
    def __init__(self, *args, **kwargs):
        super().__init__(*args)
        self.setupUi(self)
        self.file_path = ""
        self.pose_df = pd.DataFrame()
        self.joint_pos = np.zeros(16)
        self.tgt_pose = np.zeros(16)
        self.time_from_start = 0
        self.wait_time = 0
        self.trigger = 0.

        self.node: Node = kwargs["node"]
        self.posePathBtn.clicked.connect(self.open_csv_file)

        self.converge_criterion = 0.3
        self.convCritLineEdit.setText(str(self.converge_criterion))
        self.convCritLineEdit.editingFinished.connect(
            lambda: self.set_converge_criterion(self.convCritLineEdit.text())
        )

        self.sendBtn.clicked.connect(self.cb_send)
        self.execBtn.clicked.connect(self.cb_exec)

        self.arm_traj_enabled_publisher = self.node.create_publisher(ARMstrongTrajectoryEnabled,
                                                                     "armstrong_traj_enabled", 10)
        self.arm_traj_publisher = self.node.create_publisher(ARMstrongTrajectory, "armstrong_traj", 10)
        self.arm_traj_execution_publisher = self.node.create_publisher(ARMstrongTrajectoryExecution,
                                                                       "armstrong_traj_execution", 10)
        self.finished.connect(lambda: self.enabledBtn.setChecked(False))
        self.on_wait = False
        self.pos_start_time = -1
        self.progress = 0.0

        self.on_traj_exec = False
        self.is_traj_completed = False
        self.trajectory = np.zeros((16, 1))
        self.traj_idx = 0

    def open_csv_file(self):
        """
        header of csv file must be
        name, l1, l2 ,l3, ..., r7, r8, trigger,     time_from_start
        str , float (deg), ..., float, int(0~1000), float
        """
        self.file_path, _ = QFileDialog.getOpenFileName(
            self, caption="Select Pose File", filter="csv files (*.csv)")
        self.tgt_pose = self.pose_df.iloc[:16, 0]
        self.trigger = self.pose_df.iloc[16, 0]
        self.time_from_start = self.pose_df.iloc[17, 0]

        cx = self.iterTable.columnCount() - 1
        if self.pose_df.shape[1] > cx:
            for i in range(self.pose_df.shape[1] - cx):
                self.iterTable.insertColumn(cx + i)
        elif self.pose_df.shape[1] < cx:
            for i in range(cx - self.pose_df.shape[1]):
                self.iterTable.removeColumn(cx - i)

        self.iterTable.setHorizontalHeaderLabels(
            ["Now", *self.pose_df.columns.astype(str)]
        )

        for col in range(self.pose_df.shape[1]):
            for row, pose in enumerate(self.pose_df.iloc[:, col]):
                self.iterTable.setItem(
                    row, col + 1, QTableWidgetItem(f"{pose:.2f}")
                )

    def update_table(self):
        if self.pose_df.shape[1] == 0:
            return
        for i in range(18):
            for col in range(self.pose_df.shape[1]):
                for row, pose in enumerate(self.pose_df.iloc[:, col]):
                    self.iterTable.setItem(
                        row, col + 1, QTableWidgetItem(f"{pose:.2f}")
                    )
        self.tgt_pose = self.pose_df.iloc[:16, 0]
        self.trigger = self.pose_df.loc["trigger"].iloc[0]
        self.time_from_start = self.pose_df.loc["time_from_start"].iloc[0]
        self.iterTable.setHorizontalHeaderLabels(
            ["Now", *self.pose_df.columns.astype(str)]
        )

    def update_pos_col(self, pos: np.ndarray, converged: np.ndarray):
        for i, p in enumerate(pos):
            item = QTableWidgetItem(f"{pos[i]:.2f}")
            item.setBackground(
                QColor(0x00, 0xCD, 0x00) if converged[i] else QColor(0xFF, 0xFF, 0xFF)
            )
            self.iterTable.setItem(i, 0, item)
        self.progress = self.traj_idx / self.trajectory.shape[0]
        item = QTableWidgetItem(f"{self.progress * 100:.2f}%")
        self.iterTable.setItem(17, 0, item)
        item = QTableWidgetItem(
            f"{time.time()-self.pos_start_time:.2f}"
            if self.pos_start_time != -1
            else ""
        )
        self.iterTable.setItem(18, 0, item)

    def determine_converged(self, pos: np.ndarray, tgt_pose: np.ndarray) -> np.ndarray:
        if tgt_pose.size != 0:
            converged = np.less_equal(
                abs(tgt_pose - pos),
                self.converge_criterion,
                out=np.array([True] * 16, dtype=bool),
                where=~np.isnan(tgt_pose),
            )
            return converged
        else:
            return np.array([False] * 16)

    def build_message(self, data: pd.DataFrame):
        points = []
        point_name = list(data.columns)
        trigger = list(data["trigger"])
        for i, col in enumerate(data.columns):
            point = JointTrajectoryPoint()
            point.positions = data[col][:16].tolist()
            point.time_from_start = float(data["time_from_start"][i])
            points.append(point)
        msg = ARMstrongTrajectory()
        msg.point_name = point_name
        msg.points = points
        msg.trigger = trigger
        return msg

    def cb_send(self):
        msg = self.build_message(self.pose_df)
        self.arm_traj_publisher.publish(msg)
        self.statusLabel.setText("STATUS: sent and ready")

    def cb_exec(self):
        msg = ARMstrongTrajectoryExecution()
        msg.execute = True
        self.arm_traj_execution_publisher.publish(msg)
        self.statusLabel.setText("STATUS: executing...")
        self.execBtn.setDisabled(True)

    def set_converge_criterion(self, val):
        self.converge_criterion = float(val)
