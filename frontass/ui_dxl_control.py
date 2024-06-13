from PySide6.QtWidgets import QDialog
from ui_assets.dxl_control_ui import Ui_Dialog


class UIDXLControl(QDialog, Ui_Dialog):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setupUi(self)

        self.upBtn.pressed.connect(self.up_press_event)
        self.upBtn.released.connect(self.up_release_event)

        self.downBtn.pressed.connect(self.down_press_event)
        self.downBtn.released.connect(self.down_release_event)

    def up_press_event(self):
        self.upBtn.setChecked(True)
        self.neutBtn.setChecked(False)
        self.downBtn.setChecked(False)

    def up_release_event(self):
        self.upBtn.setChecked(False)
        self.neutBtn.setChecked(True)
        self.downBtn.setChecked(False)

    def down_press_event(self):
        self.upBtn.setChecked(False)
        self.neutBtn.setChecked(False)
        self.downBtn.setChecked(True)

    def down_release_event(self):
        self.upBtn.setChecked(False)
        self.neutBtn.setChecked(True)
        self.downBtn.setChecked(False)
