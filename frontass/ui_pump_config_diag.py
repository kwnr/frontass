from PySide6.QtWidgets import QDialog
from ui_assets.pump_config_diag_ui import Ui_Dialog


class UIPumpConfigDiag(QDialog, Ui_Dialog):
    def __init__(self, parent):
        super().__init__(parent)
        self.setupUi(self)
