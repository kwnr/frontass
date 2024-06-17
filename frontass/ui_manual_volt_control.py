from PySide6.QtWidgets import QDialog
from ui_assets.manual_volt_control_ui import Ui_Dialog
from ass_msgs.msg import ManualVolt
from rclpy.node import Node
import numpy as np


class UIManualVolt(QDialog, Ui_Dialog):
    def __init__(self, *args, **kwargs):
        super().__init__(*args)
        self.setupUi(self)

        self.node: Node = kwargs["node"]

        self.publisher = self.node.create_publisher(ManualVolt, "manual_volt", 10)

        self.enabled = np.zeros(16, dtype=bool)
        self.value = np.zeros(16, dtype=float)

        self.l1EnableBtn.toggled.connect(lambda data: self.enable_btn_toggle_event(0, data))
        self.l2EnableBtn.toggled.connect(lambda data: self.enable_btn_toggle_event(1, data))
        self.l3EnableBtn.toggled.connect(lambda data: self.enable_btn_toggle_event(2, data))
        self.l4EnableBtn.toggled.connect(lambda data: self.enable_btn_toggle_event(3, data))
        self.l5EnableBtn.toggled.connect(lambda data: self.enable_btn_toggle_event(4, data))
        self.l6EnableBtn.toggled.connect(lambda data: self.enable_btn_toggle_event(5, data))
        self.l7EnableBtn.toggled.connect(lambda data: self.enable_btn_toggle_event(6, data))
        self.l8EnableBtn.toggled.connect(lambda data: self.enable_btn_toggle_event(7, data))

        self.r1EnableBtn.toggled.connect(lambda data: self.enable_btn_toggle_event(0+8, data))
        self.r2EnableBtn.toggled.connect(lambda data: self.enable_btn_toggle_event(1+8, data))
        self.r3EnableBtn.toggled.connect(lambda data: self.enable_btn_toggle_event(2+8, data))
        self.r4EnableBtn.toggled.connect(lambda data: self.enable_btn_toggle_event(3+8, data))
        self.r5EnableBtn.toggled.connect(lambda data: self.enable_btn_toggle_event(4+8, data))
        self.r6EnableBtn.toggled.connect(lambda data: self.enable_btn_toggle_event(5+8, data))
        self.r7EnableBtn.toggled.connect(lambda data: self.enable_btn_toggle_event(6+8, data))
        self.r8EnableBtn.toggled.connect(lambda data: self.enable_btn_toggle_event(7+8, data))

        self.l1SpinBox.valueChanged.connect(lambda data: self.spinbox_changed_event(0, data))
        self.l2SpinBox.valueChanged.connect(lambda data: self.spinbox_changed_event(1, data))
        self.l3SpinBox.valueChanged.connect(lambda data: self.spinbox_changed_event(2, data))
        self.l4SpinBox.valueChanged.connect(lambda data: self.spinbox_changed_event(3, data))
        self.l5SpinBox.valueChanged.connect(lambda data: self.spinbox_changed_event(4, data))
        self.l6SpinBox.valueChanged.connect(lambda data: self.spinbox_changed_event(5, data))
        self.l7SpinBox.valueChanged.connect(lambda data: self.spinbox_changed_event(6, data))
        self.l8SpinBox.valueChanged.connect(lambda data: self.spinbox_changed_event(7, data))

        self.r1SpinBox.valueChanged.connect(lambda data: self.spinbox_changed_event(0+8, data))
        self.r2SpinBox.valueChanged.connect(lambda data: self.spinbox_changed_event(1+8, data))
        self.r3SpinBox.valueChanged.connect(lambda data: self.spinbox_changed_event(2+8, data))
        self.r4SpinBox.valueChanged.connect(lambda data: self.spinbox_changed_event(3+8, data))
        self.r5SpinBox.valueChanged.connect(lambda data: self.spinbox_changed_event(4+8, data))
        self.r6SpinBox.valueChanged.connect(lambda data: self.spinbox_changed_event(5+8, data))
        self.r7SpinBox.valueChanged.connect(lambda data: self.spinbox_changed_event(6+8, data))
        self.r8SpinBox.valueChanged.connect(lambda data: self.spinbox_changed_event(7+8, data))

    def enable_btn_toggle_event(self, idx, data):
        self.enabled[idx] = data
        msg = ManualVolt()
        msg.override_enabled = self.enabled.tolist()
        msg.volt_override = self.value.tolist()
        self.publisher.publish(msg)

    def spinbox_changed_event(self, idx, data):
        self.value[idx] = data
        msg = ManualVolt()
        msg.override_enabled = self.enabled.tolist()
        msg.volt_override = self.value.tolist()
        self.publisher.publish(msg)
