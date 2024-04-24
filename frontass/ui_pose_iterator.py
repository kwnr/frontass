from PySide6.QtCore import QDir, QTimer
from PySide6.QtGui import QColor
from PySide6.QtWidgets import QDialog, QFileDialog, QTableWidgetItem
from ui_assets.pose_iterator_diag_ui import Ui_Dialog

import pandas as pd
import numpy as np
import time


class UIPoseIterator(QDialog, Ui_Dialog):
    def __init__(self, parent, publisher):
        super().__init__(parent)
        self.setupUi(self)
        self.file_path = ""
        self.pose_df = pd.DataFrame()
        self.joint_pos = np.zeros(16)
        self.tgt_pose = np.zeros(16)
        self.exec_time = 0
        self.wait_time = 0
        self.trigger = 0

        self.pose_path_btn.clicked.connect(self.open_csv_file)

        self.converge_criterion = 0.3
        self.conv_crit_line.setText(str(self.converge_criterion))
        self.conv_crit_line.editingFinished.connect(
            lambda: self.set_converge_criterion(self.conv_crit_line.text())
        )

        self.publisher = publisher
        self.enabled_btn.clicked.connect(self.publish)
        self.finished.connect(lambda: self.enabled_btn.setChecked(False))
        self.finished.connect(self.publish)
        self.on_wait = False
        self.pos_start_time = -1
        self.progress = 0.0

        self.on_traj_exec = False
        self.is_traj_completed = False
        self.trajectory = np.zeros((16, 1))
        self.traj_idx = 0

        self.pose_timer = QTimer(self)
        self.pose_timer.setSingleShot(True)
        self.pose_timer.timeout.connect(self.next_pose)

        self.next_pose_btn.clicked.connect(self.next_pose)

    def open_csv_file(self):
        self.file_path, _ = QFileDialog.getOpenFileName(
            self, caption="Select Pose File", filter="csv files (*.csv)"
        )
        if self.file_path != "":
            directory = QDir(self.file_path)
            self.pose_path_lineedit.setText(directory.absolutePath())
            self.pose_df = pd.read_csv(self.file_path, header=0, index_col="NAME")
            self.pose_df = self.pose_df.T
            self.tgt_pose = self.pose_df.iloc[:16, 0]
            self.exec_time = self.pose_df.iloc[17, 0]

            cx = self.iter_table.columnCount() - 1
            if self.pose_df.shape[1] > cx:
                for i in range(self.pose_df.shape[1] - cx):
                    self.iter_table.insertColumn(cx + i)
            elif self.pose_df.shape[1] < cx:
                for i in range(cx - self.pose_df.shape[1]):
                    self.iter_table.removeColumn(cx - i)

            self.iter_table.setHorizontalHeaderLabels(
                ["Now", *self.pose_df.columns.astype(str)]
            )

            for col in range(self.pose_df.shape[1]):
                for row, pose in enumerate(self.pose_df.iloc[:, col]):
                    self.iter_table.setItem(
                        row, col + 1, QTableWidgetItem(f"{pose:.2f}")
                    )

    def update_table(self):
        if self.pose_df.shape[1] == 0:
            return
        for i in range(18):
            for col in range(self.pose_df.shape[1]):
                for row, pose in enumerate(self.pose_df.iloc[:, col]):
                    self.iter_table.setItem(
                        row, col + 1, QTableWidgetItem(f"{pose:.2f}")
                    )
        self.tgt_pose = self.pose_df.iloc[:16, 0]
        self.trigger = self.pose_df.loc["trigger"].iloc[0]
        self.exec_time = self.pose_df.loc["exec_time"].iloc[0]
        self.wait_time = self.pose_df.loc["wait_time"].iloc[0]
        self.iter_table.setHorizontalHeaderLabels(
            ["Now", *self.pose_df.columns.astype(str)]
        )

    def update_pos_col(self, pos: np.ndarray, converged: np.ndarray):
        for i, p in enumerate(pos):
            item = QTableWidgetItem(f"{pos[i]:.2f}")
            item.setBackground(
                QColor(0x00, 0xCD, 0x00) if converged[i] else QColor(0xFF, 0xFF, 0xFF)
            )
            self.iter_table.setItem(i, 0, item)
        self.progress = self.traj_idx / self.trajectory.shape[0]
        item = QTableWidgetItem(f"{self.progress * 100:.2f}%")
        self.iter_table.setItem(17, 0, item)
        item = QTableWidgetItem(
            f"{time.time()-self.pos_start_time:.2f}"
            if self.pos_start_time != -1
            else ""
        )
        self.iter_table.setItem(18, 0, item)

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

    # TODO: modify as subprocess loop from timer callback
    def timer_loop(self, pos: np.ndarray):
        self.joint_pos = pos
        converged = self.determine_converged(self.joint_pos, self.tgt_pose)
        self.update_pos_col(pos, converged)
        if self.enabled_btn.isChecked():
            if not self.is_traj_completed and not self.on_traj_exec:
                self.on_traj_exec = True
                self.trajectory = self.make_trajectory(self.tgt_pose, self.exec_time)
                self.traj_idx = 0
                self.time_started = time.time()
                self.publisher.publish(
                    self.enabled_btn.isChecked(),
                    self.trajectory[self.traj_idx],
                    self.trigger,
                )
            elif not self.is_traj_completed:
                if self.traj_idx < len(self.trajectory):
                    self.publisher.publish(
                        self.enabled_btn.isChecked(),
                        self.trajectory[self.traj_idx],
                        self.trigger,
                    )
                    self.traj_idx = int(
                        (time.time() - self.time_started)
                        / self.exec_time
                        * len(self.trajectory)
                    )
                else:
                    self.publisher.publish(
                        self.enabled_btn.isChecked(), self.trajectory[-1], self.trigger
                    )
                    self.is_traj_completed = True
                    self.on_traj_exec = False
            elif converged.all() and not self.on_wait:
                self.on_wait = True
                t_wait = (
                    self.pose_df.loc["wait_time"][0] * 1000
                    if ~np.isnan(self.pose_df.loc["wait_time"][0])
                    else 0
                )
                self.pos_start_time = time.time()
                self.pose_timer.setInterval(t_wait)
                self.pose_timer.start()
            elif self.on_wait and not converged.all():
                self.pose_timer.stop()
                self.on_wait = False
                self.pos_start_time = -1

    def next_pose(self):
        col = self.pose_df.columns
        self.pose_df = pd.concat([self.pose_df, self.pose_df.pop(col[0])], axis=1)
        self.tgt_pose = self.pose_df.iloc[:, 0]
        self.update_table()
        self.publish()
        self.on_wait = False
        self.is_traj_completed = False
        self.progress = 0.0
        self.pos_start_time = -1

    def make_trajectory(self, tgt_pose, t):
        step = int(t * 10) + 2
        trajectory = np.linspace(self.joint_pos, tgt_pose, step)
        return trajectory

    def publish(self):
        enabled = self.enabled_btn.isChecked()
        print(self.trigger)
        self.publisher.publish(
            enabled=enabled, poses=self.tgt_pose, trigger=self.trigger
        )

    def set_converge_criterion(self, val):
        self.converge_criterion = float(val)
