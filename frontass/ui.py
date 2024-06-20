import sys

sys.path.append("/home/kwnr/ros2ws/src/frontass/frontass/ui_assets")
sys.path.append("/home/kwnr/ros2ws/src/frontass/frontass")

from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QPushButton,
    QLineEdit,
    QDialogButtonBox,
)
from PySide6.QtCore import Slot, QTimer
import rclpy.exceptions
from ui_assets.control_panel_ui import Ui_MainWindow
from ui_state_diag import UiStateDiag
from ui_fig_select_diag import UIFigSelectDiag
from ui_pump_config_diag import UIPumpConfigDiag
from ui_preset_diag import UIPresetDiag
from ui_pose_iterator import UIPoseIterator
from ui_movegroup import UIMoveGroup
from ui_dxl_control import UIDXLControl
from ui_manual_volt_control import UIManualVolt
from utils import BlitManager

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from ass_msgs.msg import Robot2UI, Hold, Preset, UIAction, PoseIteration
from sensor_msgs.msg import JointState

from threading import Thread
from typing import List
from itertools import compress
import numpy as np
import nmcli

from matplotlib.backends.backend_qtagg import FigureCanvas
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.axes import Axes
from matplotlib import color_sequences


class UI(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.node = Node("assui_kwnr")

        self.setupUi(self)

        self.executer = MultiThreadedExecutor(16)
        self.executer.add_node(self.node)
        self.th_spin = Thread(target=self.executer.spin)
        self.th_spin.start()

        self.callback_group = ReentrantCallbackGroup()
        self.mutex_callback_group = MutuallyExclusiveCallbackGroup()

        self.node.create_subscription(
            Robot2UI,
            "robot_to_ui",
            self.cb_subscribe_robot_state,
            qos_profile_system_default,
            callback_group=self.mutex_callback_group
        )
        self.hold_pub = self.node.create_publisher(
            Hold, "hold", qos_profile_system_default, callback_group=self.callback_group
        )
        self.ui_action_pub = self.node.create_publisher(
            UIAction, "ui_action", qos_profile_system_default, callback_group=self.callback_group
        )

        # ui properties
        # should be list to be a mutable
        self.robot_power: List[bool] = [False]
        self.wifi_status = "Not Connected"
        self.track_left_status = 500
        self.track_right_status = 500

        self.control_mode: str = "master"
        self.control_mode_list: List[str] = ["master", "preset", "hold"]

        self.pump_power: List[bool] = [False]
        self.pump_mode: List[bool] = [False]  # True if auto
        self.pump_act_rpm: int = 0
        self.pump_des_rpm: int = 0
        self.pump_des_cur: float = 0
        self.pump_tgt_speed = 0
        self.pump_elmo_temp = 0
        self.pump_max_rpm = 2800
        self.pump_min_rpm = 600
        self.pump_max_err = 50

        self.left_arm_power: List[bool] = [False]
        self.right_arm_power: List[bool] = [False]

        self.is_on_hold = np.array([False] * 16)
        self.is_joint_on = np.array([False] * 16)
        self.is_joint_on_prev = np.array([False] * 16)

        self.joint_pos = np.array([0] * 16)
        self.ref_pos = np.array([0] * 16)
        self.cmd_voltage: List[float] = [0] * 16

        self.err = np.zeros(16, dtype=float)
        self.err_i = np.zeros(16, dtype=float)
        self.err_d = np.zeros(16, dtype=float)
        self.err_norm = 0

        self.is_smooth_start_enabled = True
        self.is_smooth_start_engaged = False

        self.time_sec: float = 0.0

        self.freq_cmd = 0
        self.freq_pump = 0
        self.freq_read = 0

        # ui button actions
        self.power_btn.toggled.connect(
            lambda: self.value_toggle(self.power_btn, self.robot_power, 0)
        )
        self.power_btn.toggled.connect(self.cb_ui_action)

        # pump_frame
        self.pump_power_btn.toggled.connect(
            lambda: self.value_toggle(self.pump_power_btn, self.pump_power, 0)
        )
        self.pump_mode_btn.toggled.connect(
            lambda: self.value_toggle(self.pump_mode_btn, self.pump_mode, 0)
        )
        self.pump_power_btn.toggled.connect(self.cb_ui_action)
        self.pump_mode_btn.toggled.connect(self.cb_ui_action)

        self.pump_config_diag = UIPumpConfigDiag(self)
        self.pump_config_btn.clicked.connect(self.pump_config_diag.show)
        self.pump_config_btn.clicked.connect(self.pump_config_diag.activateWindow)
        self.pump_config_diag.manualTargetRPMLineEdit.setText(str(self.pump_tgt_speed))
        self.pump_config_diag.autoRangeMinLineEdit.setText(str(self.pump_min_rpm))
        self.pump_config_diag.autoRangeMaxLineEdit.setText(str(self.pump_max_rpm))
        self.pump_config_diag.autoMaxErrorLineEdit.setText(str(self.pump_max_err))

        self.pump_config_diag.accepted.connect(self.cb_pump_config_accepted)
        self.pump_config_diag.accepted.connect(self.cb_ui_action)

        # arm_power_frame
        self.preset_pos = np.zeros(16)
        self.preset_diag = UIPresetDiag(self)
        self.preset_pub = self.node.create_publisher(
            Preset, "preset", qos_profile_system_default, callback_group=self.callback_group
        )
        self.preset_mode_btn.clicked.connect(self.preset_diag.show)

        self.preset_diag.joint_spinbox_list[0].valueChanged.connect(
            self.get_preset_spinbox_val
        )
        self.preset_diag.joint_spinbox_list[1].valueChanged.connect(
            self.get_preset_spinbox_val
        )
        self.preset_diag.joint_spinbox_list[2].valueChanged.connect(
            self.get_preset_spinbox_val
        )
        self.preset_diag.joint_spinbox_list[3].valueChanged.connect(
            self.get_preset_spinbox_val
        )
        self.preset_diag.joint_spinbox_list[4].valueChanged.connect(
            self.get_preset_spinbox_val
        )
        self.preset_diag.joint_spinbox_list[5].valueChanged.connect(
            self.get_preset_spinbox_val
        )
        self.preset_diag.joint_spinbox_list[6].valueChanged.connect(
            self.get_preset_spinbox_val
        )
        self.preset_diag.joint_spinbox_list[7].valueChanged.connect(
            self.get_preset_spinbox_val
        )
        self.preset_diag.joint_spinbox_list[8].valueChanged.connect(
            self.get_preset_spinbox_val
        )
        self.preset_diag.joint_spinbox_list[9].valueChanged.connect(
            self.get_preset_spinbox_val
        )
        self.preset_diag.joint_spinbox_list[10].valueChanged.connect(
            self.get_preset_spinbox_val
        )
        self.preset_diag.joint_spinbox_list[11].valueChanged.connect(
            self.get_preset_spinbox_val
        )
        self.preset_diag.joint_spinbox_list[12].valueChanged.connect(
            self.get_preset_spinbox_val
        )
        self.preset_diag.joint_spinbox_list[13].valueChanged.connect(
            self.get_preset_spinbox_val
        )
        self.preset_diag.joint_spinbox_list[14].valueChanged.connect(
            self.get_preset_spinbox_val
        )
        self.preset_diag.joint_spinbox_list[15].valueChanged.connect(
            self.get_preset_spinbox_val
        )

        self.preset_diag.enable_preset_mode_btn.toggled.connect(self.cb_req_preset_srv)
        self.preset_diag.apply_inst_cbx.toggled.connect(self.cb_req_preset_srv)
        self.preset_diag.buttonBox.clicked.connect(
            lambda btn: (
                self.cb_req_preset_srv()
                if self.preset_diag.buttonBox.standardButton(btn)
                == QDialogButtonBox.Apply
                else print(self.preset_diag.buttonBox.standardButton(btn))
            )
        )
        self.preset_diag.buttonBox.accepted.connect(self.cb_req_preset_srv)

        # left arm power button
        self.left_arm_power_btn.toggled.connect(
            lambda: self.value_toggle(self.left_arm_power_btn, self.left_arm_power, 0)
        )
        self.left_arm_power_btn.toggled.connect(
            lambda: self.left_arm_power_btn.setText(
                "Left Arm\nON" if self.left_arm_power[0] else "Left Arm\nOFF"
            )
        )

        # right arm power button
        self.right_arm_power_btn.toggled.connect(
            lambda: self.value_toggle(self.right_arm_power_btn, self.right_arm_power, 0)
        )
        self.right_arm_power_btn.toggled.connect(
            lambda: self.right_arm_power_btn.setText(
                "Right Arm\nON" if self.right_arm_power[0] else "Right Arm\nOFF"
            )
        )

        self.l1_power_btn.toggled.connect(
            lambda: self.pub_hold_toggle(
                self.l1_power_btn, self.is_on_hold, 0, self.hold_pub
            )
        )
        self.l2_power_btn.toggled.connect(
            lambda: self.pub_hold_toggle(
                self.l2_power_btn, self.is_on_hold, 1, self.hold_pub
            )
        )
        self.l3_power_btn.toggled.connect(
            lambda: self.pub_hold_toggle(
                self.l3_power_btn, self.is_on_hold, 2, self.hold_pub
            )
        )
        self.l4_power_btn.toggled.connect(
            lambda: self.pub_hold_toggle(
                self.l4_power_btn, self.is_on_hold, 3, self.hold_pub
            )
        )
        self.l5_power_btn.toggled.connect(
            lambda: self.pub_hold_toggle(
                self.l5_power_btn, self.is_on_hold, 4, self.hold_pub
            )
        )
        self.l6_power_btn.toggled.connect(
            lambda: self.pub_hold_toggle(
                self.l6_power_btn, self.is_on_hold, 5, self.hold_pub
            )
        )
        self.l7_power_btn.toggled.connect(
            lambda: self.pub_hold_toggle(
                self.l7_power_btn, self.is_on_hold, 6, self.hold_pub
            )
        )
        self.l8_power_btn.toggled.connect(
            lambda: self.pub_hold_toggle(
                self.l8_power_btn, self.is_on_hold, 7, self.hold_pub
            )
        )

        self.r1_power_btn.toggled.connect(
            lambda: self.pub_hold_toggle(
                self.r1_power_btn, self.is_on_hold, 8, self.hold_pub
            )
        )
        self.r2_power_btn.toggled.connect(
            lambda: self.pub_hold_toggle(
                self.r2_power_btn, self.is_on_hold, 9, self.hold_pub
            )
        )
        self.r3_power_btn.toggled.connect(
            lambda: self.pub_hold_toggle(
                self.r3_power_btn, self.is_on_hold, 10, self.hold_pub
            )
        )
        self.r4_power_btn.toggled.connect(
            lambda: self.pub_hold_toggle(
                self.r4_power_btn, self.is_on_hold, 11, self.hold_pub
            )
        )
        self.r5_power_btn.toggled.connect(
            lambda: self.pub_hold_toggle(
                self.r5_power_btn, self.is_on_hold, 12, self.hold_pub
            )
        )
        self.r6_power_btn.toggled.connect(
            lambda: self.pub_hold_toggle(
                self.r6_power_btn, self.is_on_hold, 13, self.hold_pub
            )
        )
        self.r7_power_btn.toggled.connect(
            lambda: self.pub_hold_toggle(
                self.r7_power_btn, self.is_on_hold, 14, self.hold_pub
            )
        )
        self.r8_power_btn.toggled.connect(
            lambda: self.pub_hold_toggle(
                self.r8_power_btn, self.is_on_hold, 15, self.hold_pub
            )
        )

        self.left_arm_power_btn.toggled.connect(self.cb_ui_action)
        self.right_arm_power_btn.toggled.connect(self.cb_ui_action)

        self.l1_power_btn.toggled.connect(self.cb_ui_action)
        self.l2_power_btn.toggled.connect(self.cb_ui_action)
        self.l3_power_btn.toggled.connect(self.cb_ui_action)
        self.l4_power_btn.toggled.connect(self.cb_ui_action)
        self.l5_power_btn.toggled.connect(self.cb_ui_action)
        self.l6_power_btn.toggled.connect(self.cb_ui_action)
        self.l7_power_btn.toggled.connect(self.cb_ui_action)
        self.l8_power_btn.toggled.connect(self.cb_ui_action)

        self.r1_power_btn.toggled.connect(self.cb_ui_action)
        self.r2_power_btn.toggled.connect(self.cb_ui_action)
        self.r3_power_btn.toggled.connect(self.cb_ui_action)
        self.r4_power_btn.toggled.connect(self.cb_ui_action)
        self.r5_power_btn.toggled.connect(self.cb_ui_action)
        self.r6_power_btn.toggled.connect(self.cb_ui_action)
        self.r7_power_btn.toggled.connect(self.cb_ui_action)
        self.r8_power_btn.toggled.connect(self.cb_ui_action)

        self.pose_iter_timer = QTimer(self)
        self.pose_iter_timer.setInterval(10)
        self.pose_iter_publisher = self.node.create_publisher(
            PoseIteration, "pose_iter", qos_profile_system_default, callback_group=self.callback_group
        )
        self.pose_iteration_diag = UIPoseIterator(self, self.pose_iter_publisher)
        self.start_pose_iteration_btn.clicked.connect(self.pose_iteration_diag.show)

        self.start_pose_iteration_btn.clicked.connect(self.pose_iter_timer.start)
        self.pose_iter_timer.timeout.connect(
            lambda: self.pose_iteration_diag.timer_loop(self.joint_pos)
        )
        self.pose_iteration_diag.finished.connect(self.pose_iter_timer.stop)

        self.start_time = self.node.get_clock().now().seconds_nanoseconds()[0]
        # graph - arm
        self.arm_canvas = FigureCanvas(Figure(figsize=(6.4, 7)))
        self.arm_ax, self.pump_ax = self.arm_canvas.figure.subplots(
            nrows=2, ncols=1, sharex=True
        )
        self.pos_lines: List[Line2D] = self.arm_ax.plot(
            0,
            np.zeros((1, 16)),
            linestyle="solid",
            linewidth=2,
            label=[f"L{i} pos" for i in range(1, 9)]
            + [f"R{i} pos" for i in range(1, 9)],
            animated=True,
        )
        self.ref_lines: List[Line2D] = self.arm_ax.plot(
            0,
            np.zeros((1, 16)),
            linestyle="dashed",
            linewidth=2,
            label=[f"L{i} ref" for i in range(1, 9)]
            + [f"R{i} ref" for i in range(1, 9)],
            animated=True,
        )
        self.cmd_lines: List[Line2D] = self.arm_ax.plot(
            0,
            np.zeros((1, 16)),
            linestyle="dotted",
            linewidth=2,
            label=[f"L{i} cmd" for i in range(1, 9)]
            + [f"R{i} cmd" for i in range(1, 9)],
            animated=True,
        )
        self.err_lines: List[Line2D] = self.arm_ax.plot(
            0,
            np.zeros((1, 16)),
            linestyle=(0, (5, 1)),
            linewidth=2,
            label=[f"L{i} err" for i in range(1, 9)]
            + [f"R{i} err" for i in range(1, 9)],
            animated=True,
        )
        self.err_i_lines: List[Line2D] = self.arm_ax.plot(
            0,
            np.zeros((1, 16)),
            linestyle=(0, (3, 1, 1, 1)),
            linewidth=2,
            label=[f"L{i} err_i" for i in range(1, 9)]
            + [f"R{i} err_i" for i in range(1, 9)],
            animated=True,
        )
        self.err_d_lines: List[Line2D] = self.arm_ax.plot(
            0,
            np.zeros((1, 16)),
            linestyle=(0, (3, 1, 1, 1, 1, 1, 1)),
            linewidth=2,
            label=[f"L{i} err_d" for i in range(1, 9)]
            + [f"R{i} err_d" for i in range(1, 9)],
            animated=True,
        )

        color = color_sequences["tab20b"] + color_sequences["tab20c"]
        for idx in range(8):
            self.pos_lines[idx].set_color(color[idx * 4])  # Color for left arm
            self.ref_lines[idx].set_color(color[idx * 4])
            self.cmd_lines[idx].set_color(color[idx * 4])
            self.err_lines[idx].set_color(color[idx * 4])
            self.err_i_lines[idx].set_color(color[idx * 4])
            self.err_d_lines[idx].set_color(color[idx * 4])

            self.pos_lines[idx + 8].set_color(color[idx * 4 + 1])  # Color for right arm
            self.ref_lines[idx + 8].set_color(color[idx * 4 + 1])
            self.cmd_lines[idx + 8].set_color(color[idx * 4 + 1])
            self.err_lines[idx + 8].set_color(color[idx * 4 + 1])
            self.err_i_lines[idx + 8].set_color(color[idx * 4 + 1])
            self.err_d_lines[idx + 8].set_color(color[idx * 4 + 1])

        self.arm_ax.set_ylabel("Position [deg]")
        self.arm_ax.set_autoscaley_on(True)
        self.arm_ax.grid(True)

        # graph - pump
        self.pump_ax.figure.subplots_adjust(right=0.88, bottom=0.12)
        self.pump_etc_ax: Axes = self.pump_ax.twinx()

        self.pump_rpm_lines: List[Line2D] = self.pump_ax.plot(
            0, np.zeros((1, 2)), label=["actual rpm", "desired rpm"], linewidth=2
        )
        self.pump_cur_line, self.pump_elmo_temp_line = self.pump_etc_ax.plot(
            0,
            np.zeros((1, 2)),
            label=["desired current", "driver temp"],
            linewidth=2,
        )
        self.pump_rpm_lines[0].set_color(color[0])
        self.pump_rpm_lines[1].set_color(color[2])
        self.pump_cur_line.set_color(color[4])
        self.pump_elmo_temp_line.set_color(color[6])

        self.pump_ax.set(xlabel="time [sec]", ylabel="RPM [RPM]")
        self.pump_etc_ax.set_ylabel("Current [A], Temperature [°C]")

        self.pump_etc_ax.legend(
            handles=[
                self.pump_rpm_lines[0],
                self.pump_rpm_lines[1],
                self.pump_cur_line,
                self.pump_elmo_temp_line,
            ],
            loc="upper left",
        )
        self.pump_ax.set_autoscaley_on(True)
        self.pump_etc_ax.set_autoscaley_on(True)
        self.pump_ax.grid(True)

        self.bm = BlitManager(
            self.arm_canvas,
            [
                self.arm_ax,
                self.pump_ax,
                self.pump_etc_ax,
                *self.pos_lines,
                *self.ref_lines,
                *self.cmd_lines,
                *self.err_lines,
                *self.err_i_lines,
                *self.err_d_lines,
                *self.pump_rpm_lines,
                self.pump_cur_line,
                self.pump_elmo_temp_line,
            ],
        )

        fig_fps = 10
        self.arm_canvas_timer = self.arm_canvas.new_timer(1000 // fig_fps)
        self.arm_canvas_timer.add_callback(self.update_figure)
        self.arm_graph_layout.addWidget(self.arm_canvas)
        self.arm_canvas_timer.start()

        # panel infos: update per every 100ms
        self.panel_100ms_timer = QTimer(self)
        self.panel_100ms_timer.timeout.connect(self.cb_timer_100ms_panel)
        self.panel_100ms_timer.start(100)

        # panel infos: update once per 1s
        nmcli.disable_use_sudo()
        # self.client_ip_text.setText(nmcli.connection.show(self.get_wifi_info().ssid))
        self.panel_1s_timer = QTimer(self)
        self.panel_1s_timer.timeout.connect(self.cb_timer_1s_panel)
        self.panel_1s_timer.start(1000)

        # show state dialog
        self.state_diag = UiStateDiag(self)
        self.show_state_btn.clicked.connect(self.state_diag.show)
        self.table_timer = QTimer(self)
        self.table_timer.setInterval(250)
        self.table_timer.timeout.connect(
            lambda: self.state_diag.update_table(
                {
                    "left_pos": self.joint_pos[:8],
                    "right_pos": self.joint_pos[8:],
                    "left_ref": self.ref_pos[:8],
                    "right_ref": self.ref_pos[8:],
                    "left_cmd": self.cmd_voltage[:8],
                    "right_cmd": self.cmd_voltage[8:],
                    "is_on_hold": self.is_on_hold,
                }
            )
        )
        self.show_state_btn.clicked.connect(self.table_timer.start)
        self.state_diag.finished.connect(self.table_timer.stop)

        # show fig select dialog
        self.fig_sel_diag = UIFigSelectDiag(self)
        self.arm_graph_tool_btn.clicked.connect(self.fig_sel_diag.show)
        self.fig_sel_diag.checkbox_state = np.zeros((6, 16)).astype(bool)

        self.ik_diag = UIMoveGroup(
            self,
            node=self.node,
            callback_group=self.callback_group)
        self.ik_mode_btn.clicked.connect(self.ik_diag.show)
        self.joint_state_publisher = self.node.create_publisher(
            JointState,
            "joint_states",
            qos_profile_system_default,
            callback_group=self.mutex_callback_group
        )
        self.ik_diag.timer_robot_position.timeout.connect(
            lambda: self.ik_diag.cb_movegroup(self.joint_pos))
        self.ik_mode_btn.clicked.connect(self.ik_diag.timer_robot_position.start)
        self.ik_diag.finished.connect(self.ik_diag.timer_robot_position.stop)
        self.ik_diag.ik_traj_pos_changed.connect(self.publish_pose_override)

        self.dxl_control_diag = UIDXLControl(self, node=self.node)
        self.dxlControlBtn.clicked.connect(self.dxl_control_diag.open)

        self.manual_volt_diag = UIManualVolt(self, node=self.node)
        self.manualVoltControlBtn.clicked.connect(self.manual_volt_diag.show)

        self.preset_diag.enable_preset_mode_btn.clicked.connect(
            lambda: self.ik_diag.ikEnableBtn.setChecked(False)
            )
        self.preset_diag.enable_preset_mode_btn.clicked.connect(
            lambda: self.pose_iteration_diag.enabled_btn.setChecked(False)
            )
        self.pose_iteration_diag.enabled_btn.clicked.connect(
            lambda: self.preset_diag.enable_preset_mode_btn.setChecked(False)
            )
        self.pose_iteration_diag.enabled_btn.clicked.connect(
            lambda: self.ik_diag.ikEnableBtn.setChecked(False)
            )
        self.ik_diag.ikEnableBtn.clicked.connect(
            lambda: self.preset_diag.enable_preset_mode_btn.setChecked(False)
            )
        self.ik_diag.ikEnableBtn.clicked.connect(
            lambda: self.pose_iteration_diag.enabled_btn.setChecked(False)
            )

    def publish_pose_override(self, value):
        message = PoseIteration()
        message.enabled = value[0]
        message.poses = value[1]
        if len(value) < 3:
            pass
        else:
            pass
        self.pose_iter_publisher.publish(message)

    def cb_pump_config_accepted(self):
        self.pump_tgt_speed = int(self.pump_config_diag.manualTargetRPMLineEdit.text())
        self.pump_min_rpm = int(self.pump_config_diag.autoRangeMinLineEdit.text())
        self.pump_max_rpm = int(self.pump_config_diag.autoRangeMaxLineEdit.text())
        self.pump_max_err = int(self.pump_config_diag.autoMaxErrorLineEdit.text())

    def cb_ui_action(self):
        pump_pwr = self.pump_power[0] and self.robot_power[0]
        pump_mode = 1 if self.pump_mode[0] else 0
        pump_tgt_speed = self.pump_tgt_speed
        pump_max_rpm = self.pump_max_rpm
        pump_min_rpm = self.pump_min_rpm
        pump_max_err = self.pump_max_err
        joint_power = np.broadcast_to(self.robot_power[0], 16) & (
            self.left_arm_power * 8 + self.right_arm_power * 8
        )
        self.is_joint_on = joint_power & ~self.is_on_hold
        joint_to_smooth = self.is_joint_on ^ self.is_joint_on_prev & self.is_joint_on
        self.is_joint_on_prev = self.is_joint_on
        req = UIAction(
            pump_pwr=pump_pwr,
            pump_mode=pump_mode,
            pump_tgt_speed=pump_tgt_speed,
            pump_max_rpm=pump_max_rpm,
            pump_min_rpm=pump_min_rpm,
            pump_max_err=pump_max_err,
            joint_power=joint_power.tolist(),
            joint_to_smooth=joint_to_smooth.tolist(),
            ctrl_mode=self.control_mode_list.index(self.control_mode),
        )
        self.ui_action_pub.publish(req)

    def get_preset_spinbox_val(self):
        for i in range(16):
            self.preset_pos[i] = self.preset_diag.joint_spinbox_list[i].value()
        if self.preset_diag.apply_inst_cbx.isChecked():
            res = self.preset_pub.publish(
                Preset(
                    enabled=self.preset_diag.enable_preset_mode_btn.isChecked(),
                    preset_pos=self.preset_pos.tolist(),
                )
            )
            print(res)

    def cb_req_preset_srv(self):
        for i in range(16):
            self.preset_pos[i] = self.preset_diag.joint_spinbox_list[i].value()
        res = self.preset_pub.publish(
            Preset(
                enabled=self.preset_diag.enable_preset_mode_btn.isChecked(),
                preset_pos=self.preset_pos.tolist(),
            )
        )
        print(res)
        return res

    def update_figure(self):
        time = self.time_sec - self.start_time
        for i in np.argwhere(self.fig_sel_diag.checkbox_state):
            if i[0] == 0:  #
                self.update_line([self.pos_lines[i[1]]], time, self.joint_pos[i[1]])
            elif i[0] == 1:
                self.update_line([self.ref_lines[i[1]]], time, self.ref_pos[i[1]])
            elif i[0] == 2:
                self.update_line([self.cmd_lines[i[1]]], time, self.cmd_voltage[i[1]])
            elif i[0] == 3:
                self.update_line([self.err_lines[i[1]]], time, self.err[i[1]])
            elif i[0] == 4:
                self.update_line([self.err_i_lines[i[1]]], time, self.err_i[i[1]])
            elif i[0] == 5:
                self.update_line([self.err_d_lines[i[1]]], time, self.err_d[i[1]])

        self.update_line([self.pump_rpm_lines[0]], time, self.pump_act_rpm)
        self.update_line([self.pump_rpm_lines[1]], time, self.pump_des_rpm)
        self.update_line([self.pump_cur_line], time, self.pump_des_cur)
        self.update_line([self.pump_elmo_temp_line], time, self.pump_elmo_temp)

        self.arm_ax.legend(
            handles=list(compress(self.pos_lines, self.fig_sel_diag.checkbox_state[0]))
            + list(compress(self.ref_lines, self.fig_sel_diag.checkbox_state[1]))
            + list(compress(self.cmd_lines, self.fig_sel_diag.checkbox_state[2]))
            + list(compress(self.err_lines, self.fig_sel_diag.checkbox_state[3]))
            + list(compress(self.err_i_lines, self.fig_sel_diag.checkbox_state[4]))
            + list(compress(self.err_d_lines, self.fig_sel_diag.checkbox_state[5])),
            loc="upper left",
        ).set_zorder(101)

        self.arm_ax.set_xlim(time - 10, time)
        self.arm_ax.relim(visible_only=True)
        self.arm_ax.autoscale_view(True)

        self.pump_ax.set_xlim(time - 10, time)
        self.pump_ax.relim(visible_only=True)
        self.pump_ax.autoscale_view(True)

        self.pump_etc_ax.relim(visible_only=True)
        self.pump_etc_ax.autoscale_view(True)

        self.bm.update()

    @staticmethod
    def update_line(tgt: List[Line2D], time, data):
        prev_x, prev_y = tgt[0].get_data()
        if prev_x[-1] != time:
            x = np.append(prev_x[prev_x > time - 11], time)
            y = np.append(prev_y[prev_x > time - 11], data)

            tgt[0].set_data(x, y)

    @staticmethod
    def toggle_line(tgt: List[Line2D]):
        print(tgt[0])
        tgt[0].set_data([], [])

    def cb_timer_100ms_panel(self):
        self.act_rpm_text.setText(f"{self.pump_act_rpm}")
        self.des_rpm_text.setText(f"{self.pump_des_rpm}")
        self.des_cur_text.setText(f"{self.pump_des_cur:.2f}")
        self.elmo_temp_text.setText(f"{self.pump_elmo_temp:.2f}")
        self.disp_track_progress_bar(self.track_left_status, self.track_right_status)

    def cb_timer_1s_panel(self):
        """client_conn = self.get_wifi_info()
        if client_conn is not None:
            client_conn_text = client_conn.signal
        else:
            client_conn_text = "Not Connected" """
        """p = Popen(["ping", "-c1", "192.168.0.3"], stdout=PIPE, stdin=PIPE)
        res = p.communicate()
        res = res[0].decode()
        if 'time=' in res:
            self.ping_text.setText(res[0].split('time=')[1].split('\n')[0][:-3]+" ms")"""
        # self.client_conn_text.setText(f"{client_conn_text}")
        self.read_rate_text.setText(f"{self.freq_read:.2f} Hz")
        self.cmd_rate_text.setText(f"{self.freq_cmd:.2f} Hz")
        self.pump_rate_text.setText(f"{self.freq_pump:.2f} Hz")

    @staticmethod
    def button_toggle(btn_this, btn_opps):
        btn_this.setChecked(True)
        btn_opps.setChecked(False)

    @Slot()
    def value_toggle(self, btn: QPushButton, tgt: list, idx: int):
        tgt[idx] = btn.isChecked()

    @Slot()
    def pub_hold_toggle(self, btn: QPushButton, tgt: np.ndarray, idx: int, pub):
        tgt[idx] = not btn.isChecked()
        pub.publish(Hold(enabled=self.is_on_hold.tolist()))

    @Slot()
    def get_lineedit_text(self, btn: QLineEdit, store: list):
        try:
            store[0] = int(btn.text())
        except ValueError:
            store[0] = 0
        print(self.pump_tgt_speed)
        print(store)
        btn.clear()

    @Slot()
    def cb_ctrl_mode_cmb_changed(self, btn):
        text = btn.currentText()
        self.control_mode = text

    @staticmethod
    def to_sec(time):
        return time.sec + time.nanosec / 1e9

    def cb_subscribe_robot_state(self, data: Robot2UI):
        track_state = [data.track_left_state, data.track_right_state]
        self.joint_pos = data.arm_state
        self.ref_pos = data.ref_state
        self.cmd_voltage = data.cmd_voltage

        self.pump_act_rpm = data.pump_act_rpm
        self.pump_des_rpm = data.pump_des_rpm
        self.pump_des_cur = data.pump_des_cur
        self.pump_elmo_temp = data.pump_temp

        self.track_left_status, self.track_right_status = track_state
        self.err_norm = data.err_norm

        self.err = data.err
        self.err_i = data.err_i
        self.err_d = data.err_d

        self.time_sec = self.to_sec(data.header.stamp)
        self.freq_read, self.freq_cmd, self.freq_pump = data.freq

        """self.act_rpm_text.setText(self.pump_speed)
        self.disp_track_progress_bar(track_state)"""

    def disp_track_progress_bar(self, track_left_raw, track_right_raw):

        left_forward_value, left_backward_value = self.map_value_as_percent(
            track_left_raw, 0, 1000
        )
        right_forward_value, right_backward_value = self.map_value_as_percent(
            track_right_raw, 0, 1000
        )

        self.tr_left_forward_bar.setValue(left_forward_value)
        self.tr_left_backward_bar.setValue(left_backward_value)
        self.tr_right_forward_bar.setValue(right_forward_value)
        self.tr_right_backward_bar.setValue(right_backward_value)

        self.tr_left_status_text.setText(f"{left_forward_value-left_backward_value}%")
        self.tr_right_status_text.setText(
            f"{right_forward_value-right_backward_value}%"
        )

    @staticmethod
    def map_value_as_percent(raw_value: int, min_value: int, max_value: int):
        value = (raw_value - min_value) * (100 - (-100)) // (max_value - min_value) + (
            -100
        )
        if value >= 0:
            pos_val = value
            neg_val = 0
        else:
            pos_val = 0
            neg_val = -value
        return pos_val, neg_val

    @staticmethod
    def get_wifi_info():
        res = nmcli.device.wifi(rescan=False)
        in_use = [i for i in res if i.in_use]
        if len(in_use) == 0:
            current_conn = None
        else:
            current_conn = in_use[0]
        return current_conn


def main():
    rclpy.init(domain_id=1)
    app = QApplication()
    w = UI()
    w.show()
    app.exec()


if __name__ == "__main__":
    main()
