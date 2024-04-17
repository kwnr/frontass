from PySide6.QtWidgets import QDialog
from ui_assets.ui_pump_config_diag import Ui_Dialog


class UIPumpConfigDiag(QDialog, Ui_Dialog):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
