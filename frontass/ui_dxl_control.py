from PySide6.QtWidgets import QDialog
from ui_assets.dxl_control_ui import Ui_Dialog
from ass_msgs.msg import DXLCommand
from rclpy.node import Node


class UIDXLControl(QDialog, Ui_Dialog):
    def __init__(self, *args, **kwargs):
        super().__init__(*args)
        self.node: Node = kwargs["node"]
        self.setupUi(self)

        self.dxl_command_publisher = self.node.create_publisher(DXLCommand, "dxl_command", qos_profile=10)

        self.enableBtn.toggled.connect(self.enable_toggle_event)
        self.upBtn.pressed.connect(self.up_press_event)
        self.upBtn.released.connect(self.up_release_event)

        self.downBtn.pressed.connect(self.down_press_event)
        self.downBtn.released.connect(self.down_release_event)

        self.cwBtn.clicked.connect(self.cw_click_event)
        self.trackBtn.clicked.connect(self.track_click_event)
        self.ccwBtn.clicked.connect(self.ccw_click_event)

        self.leftSlider.sliderReleased.connect(self.left_slider_release_event)
        self.leftSlider.valueChanged.connect(self.left_slider_value_change_event)

        self.rightSlider.sliderReleased.connect(self.right_slider_release_event)
        self.rightSlider.valueChanged.connect(self.right_slider_value_change_event)

    def enable_toggle_event(self, data):
        msg = DXLCommand()
        msg.enabled = bool(data)
        self.dxl_command_publisher.publish(msg)

    def up_press_event(self):
        self.upBtn.setChecked(True)
        self.neutBtn.setChecked(False)
        self.downBtn.setChecked(False)
        is_enabled = self.enableBtn.isChecked()

        msg = DXLCommand()
        msg.enabled = is_enabled
        msg.lift_toggle = 2
        self.dxl_command_publisher.publish(msg)

    def up_release_event(self):
        self.upBtn.setChecked(False)
        self.neutBtn.setChecked(True)
        self.downBtn.setChecked(False)
        is_enabled = self.enableBtn.isChecked()

        msg = DXLCommand()
        msg.enabled = is_enabled
        msg.lift_toggle = 0
        self.dxl_command_publisher.publish(msg)

    def down_press_event(self):
        self.upBtn.setChecked(False)
        self.neutBtn.setChecked(False)
        self.downBtn.setChecked(True)
        is_enabled = self.enableBtn.isChecked()

        msg = DXLCommand()
        msg.enabled = is_enabled
        msg.lift_toggle = 1
        self.dxl_command_publisher.publish(msg)

    def down_release_event(self):
        self.upBtn.setChecked(False)
        self.neutBtn.setChecked(True)
        self.downBtn.setChecked(False)
        is_enabled = self.enableBtn.isChecked()

        msg = DXLCommand()
        msg.enabled = is_enabled
        msg.lift_toggle = 0
        self.dxl_command_publisher.publish(msg)

    def cw_click_event(self):
        self.trackBtn.setChecked(False)
        self.ccwBtn.setChecked(False)
        is_enabled = self.enableBtn.isChecked()

        msg = DXLCommand()
        msg.enabled = is_enabled
        msg.mode_toggle = 1
        self.dxl_command_publisher.publish(msg)

    def track_click_event(self):
        self.cwBtn.setChecked(False)
        self.ccwBtn.setChecked(False)
        is_enabled = self.enableBtn.isChecked()

        msg = DXLCommand()
        msg.enabled = is_enabled
        msg.mode_toggle = 0
        self.dxl_command_publisher.publish(msg)

    def ccw_click_event(self):
        self.cwBtn.setChecked(False)
        self.trackBtn.setChecked(False)
        is_enabled = self.enableBtn.isChecked()

        msg = DXLCommand()
        msg.enabled = is_enabled
        msg.mode_toggle = -1
        self.dxl_command_publisher.publish(msg)

    def left_slider_release_event(self):
        self.leftSlider.setValue(0)

    def right_slider_release_event(self):
        self.rightSlider.setValue(0)

    def left_slider_value_change_event(self, value):
        # range of value: -99 ~ 99, 0 is origin
        # remap value to 0 ~ 1000, origin 500
        lever = int(self.remap(value, -100, 100, 0, 1000))
        is_enabled = self.enableBtn.isChecked()

        msg = DXLCommand()
        msg.mode_toggle = 0 if self.trackBtn.isChecked() else (-1 if self.cwBtn.isChecked() else 1)
        msg.enabled = is_enabled
        msg.left_lever = lever
        self.dxl_command_publisher.publish(msg)

    def right_slider_value_change_event(self, value):
        lever = int(self.remap(value, -100, 100, 0, 1000))
        is_enabled = self.enableBtn.isChecked()

        msg = DXLCommand()
        msg.enabled = is_enabled
        msg.mode_toggle = 0 if self.trackBtn.isChecked() else (-1 if self.cwBtn.isChecked() else 1)
        msg.right_lever = lever
        self.dxl_command_publisher.publish(msg)

    @staticmethod
    def remap(x, x_min, x_max, out_min, out_max):
        return out_min + (x - x_min) * (out_max - out_min) / (x_max - x_min)
