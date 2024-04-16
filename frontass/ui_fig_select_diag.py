from PySide6.QtWidgets import QDialog, QLabel
from ui_assets.arm_fig_select_diag_ui import Ui_Dialog
import numpy as np


class UIFigSelectDiag(QDialog, Ui_Dialog):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.checkbox_state = np.zeros((6, 16)).astype(bool)

        self.button_groups = [
            [self.L1_pos, self.L2_pos, self.L3_pos, self.L4_pos, self.L5_pos, self.L6_pos, self.L7_pos, self.L8_pos,
             self.R1_pos, self.R2_pos, self.R3_pos, self.R4_pos, self.R5_pos, self.R6_pos, self.R7_pos, self.R8_pos],
            [self.L1_ref, self.L2_ref, self.L3_ref, self.L4_ref, self.L5_ref, self.L6_ref, self.L7_ref, self.L8_ref,
             self.R1_ref, self.R2_ref, self.R3_ref, self.R4_ref, self.R5_ref, self.R6_ref, self.R7_ref, self.R8_ref],
            [self.L1_cmd, self.L2_cmd, self.L3_cmd, self.L4_cmd, self.L5_cmd, self.L6_cmd, self.L7_cmd, self.L8_cmd,
             self.R1_cmd, self.R2_cmd, self.R3_cmd, self.R4_cmd, self.R5_cmd, self.R6_cmd, self.R7_cmd, self.R8_cmd],
            [self.L1_err, self.L2_err, self.L3_err, self.L4_err, self.L5_err, self.L6_err, self.L7_err, self.L8_err,
             self.R1_err, self.R2_err, self.R3_err, self.R4_err, self.R5_err, self.R6_err, self.R7_err, self.R8_err],
            [self.L1_erri, self.L2_erri, self.L3_erri, self.L4_erri, self.L5_erri, self.L6_erri, self.L7_erri, self.L8_erri,
             self.R1_erri, self.R2_erri, self.R3_erri, self.R4_erri, self.R5_erri, self.R6_erri, self.R7_erri, self.R8_erri],
            [self.L1_errd, self.L2_errd, self.L3_errd, self.L4_errd, self.L5_errd, self.L6_errd, self.L7_errd, self.L8_errd,
             self.R1_errd, self.R2_errd, self.R3_errd, self.R4_errd, self.R5_errd, self.R6_errd, self.R7_errd, self.R8_errd],
        ]

        # fig sel diag checkbox action
        self.L1_pos.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L1_pos], [0, 0]))
        self.L2_pos.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L2_pos], [0, 1]))
        self.L3_pos.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L3_pos], [0, 2]))
        self.L4_pos.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L4_pos], [0, 3]))
        self.L5_pos.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L5_pos], [0, 4]))
        self.L6_pos.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L6_pos], [0, 5]))
        self.L7_pos.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L7_pos], [0, 6]))
        self.L8_pos.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L8_pos], [0, 7]))
        self.R1_pos.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R1_pos], [0, 8]))
        self.R2_pos.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R2_pos], [0, 9]))
        self.R3_pos.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R3_pos], [0, 10]))
        self.R4_pos.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R4_pos], [0, 11]))
        self.R5_pos.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R5_pos], [0, 12]))
        self.R6_pos.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R6_pos], [0, 13]))
        self.R7_pos.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R7_pos], [0, 14]))
        self.R8_pos.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R8_pos], [0, 15]))

        self.L1_ref.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L1_ref], [1, 0]))
        self.L2_ref.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L2_ref], [1, 1]))
        self.L3_ref.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L3_ref], [1, 2]))
        self.L4_ref.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L4_ref], [1, 3]))
        self.L5_ref.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L5_ref], [1, 4]))
        self.L6_ref.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L6_ref], [1, 5]))
        self.L7_ref.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L7_ref], [1, 6]))
        self.L8_ref.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L8_ref], [1, 7]))
        self.R1_ref.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R1_ref], [1, 8]))
        self.R2_ref.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R2_ref], [1, 9]))
        self.R3_ref.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R3_ref], [1, 10]))
        self.R4_ref.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R4_ref], [1, 11]))
        self.R5_ref.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R5_ref], [1, 12]))
        self.R6_ref.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R6_ref], [1, 13]))
        self.R7_ref.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R7_ref], [1, 14]))
        self.R8_ref.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R8_ref], [1, 15]))

        self.L1_cmd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L1_cmd], [2, 0]))
        self.L2_cmd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L2_cmd], [2, 1]))
        self.L3_cmd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L3_cmd], [2, 2]))
        self.L4_cmd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L4_cmd], [2, 3]))
        self.L5_cmd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L5_cmd], [2, 4]))
        self.L6_cmd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L6_cmd], [2, 5]))
        self.L7_cmd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L7_cmd], [2, 6]))
        self.L8_cmd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L8_cmd], [2, 7]))
        self.R1_cmd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R1_cmd], [2, 8]))
        self.R2_cmd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R2_cmd], [2, 9]))
        self.R3_cmd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R3_cmd], [2, 10]))
        self.R4_cmd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R4_cmd], [2, 11]))
        self.R5_cmd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R5_cmd], [2, 12]))
        self.R6_cmd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R6_cmd], [2, 13]))
        self.R7_cmd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R7_cmd], [2, 14]))
        self.R8_cmd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R8_cmd], [2, 15]))

        self.L1_err.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L1_err], [3, 0]))
        self.L2_err.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L2_err], [3, 1]))
        self.L3_err.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L3_err], [3, 2]))
        self.L4_err.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L4_err], [3, 3]))
        self.L5_err.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L5_err], [3, 4]))
        self.L6_err.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L6_err], [3, 5]))
        self.L7_err.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L7_err], [3, 6]))
        self.L8_err.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L8_err], [3, 7]))
        self.R1_err.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R1_err], [3, 8]))
        self.R2_err.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R2_err], [3, 9]))
        self.R3_err.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R3_err], [3, 10]))
        self.R4_err.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R4_err], [3, 11]))
        self.R5_err.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R5_err], [3, 12]))
        self.R6_err.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R6_err], [3, 13]))
        self.R7_err.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R7_err], [3, 14]))
        self.R8_err.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R8_err], [3, 15]))

        self.L1_erri.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L1_erri], [4, 0]))
        self.L2_erri.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L2_erri], [4, 1]))
        self.L3_erri.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L3_erri], [4, 2]))
        self.L4_erri.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L4_erri], [4, 3]))
        self.L5_erri.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L5_erri], [4, 4]))
        self.L6_erri.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L6_erri], [4, 5]))
        self.L7_erri.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L7_erri], [4, 6]))
        self.L8_erri.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L8_erri], [4, 7]))
        self.R1_erri.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R1_erri], [4, 8]))
        self.R2_erri.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R2_erri], [4, 9]))
        self.R3_erri.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R3_erri], [4, 10]))
        self.R4_erri.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R4_erri], [4, 11]))
        self.R5_erri.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R5_erri], [4, 12]))
        self.R6_erri.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R6_erri], [4, 13]))
        self.R7_erri.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R7_erri], [4, 14]))
        self.R8_erri.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R8_erri], [4, 15]))

        self.L1_errd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L1_errd], [5, 0]))
        self.L2_errd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L2_errd], [5, 1]))
        self.L3_errd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L3_errd], [5, 2]))
        self.L4_errd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L4_errd], [5, 3]))
        self.L5_errd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L5_errd], [5, 4]))
        self.L6_errd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L6_errd], [5, 5]))
        self.L7_errd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L7_errd], [5, 6]))
        self.L8_errd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.L8_errd], [5, 7]))
        self.R1_errd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R1_errd], [5, 8]))
        self.R2_errd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R2_errd], [5, 9]))
        self.R3_errd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R3_errd], [5, 10]))
        self.R4_errd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R4_errd], [5, 11]))
        self.R5_errd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R5_errd], [5, 12]))
        self.R6_errd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R6_errd], [5, 13]))
        self.R7_errd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R7_errd], [5, 14]))
        self.R8_errd.stateChanged.connect(
            lambda: self.set_fig_selected_value([self.R8_errd], [5, 15]))

    def set_fig_selected_value(self, who, idx):
        self.checkbox_state[idx[0]][idx[1]] = who[0].isChecked()
