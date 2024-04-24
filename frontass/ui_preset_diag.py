from ui_assets.preset_mode_diag_ui import Ui_Dialog
from PySide6.QtWidgets import QDialog, QFileDialog, QApplication
import pandas as pd


class UIPresetDiag(QDialog, Ui_Dialog):
    def __init__(self, parent):
        super().__init__(parent)
        self.setupUi(self)

        self._is_on_preset_mode = False
        self._is_preset_modified = False
        self._is_preset_loaded = False

        self.file_diag = QFileDialog(
            None, "", "/home/kwnr/arm_ws/src/aic_ui/preset_files", "csv files (*.csv)"
        )
        self.toolButton.clicked.connect(self.file_diag.exec)

        self.preset_file = pd.DataFrame()
        self.file_diag.fileSelected.connect(self.preset_file_path.setText)
        self.file_diag.fileSelected.connect(self.load_preset_file)
        self.preset_file_path.returnPressed.connect(
            lambda: self.load_preset_file(self.preset_file_path.text)
        )
        self.preset_list_cbx.currentTextChanged.connect(
            self.cb_on_preset_list_cbx_changed
        )
        self.joint_list = [f"L{i}" for i in range(1, 9)] + [
            f"R{i}" for i in range(1, 9)
        ]
        self.joint_spinbox_list = [
            self.l1DoubleSpinBox,
            self.l2DoubleSpinBox,
            self.l3DoubleSpinBox,
            self.l4DoubleSpinBox,
            self.l5DoubleSpinBox,
            self.l6DoubleSpinBox,
            self.l7DoubleSpinBox,
            self.l8DoubleSpinBox,
            self.r1DoubleSpinBox,
            self.r2DoubleSpinBox,
            self.r3DoubleSpinBox,
            self.r4DoubleSpinBox,
            self.r5DoubleSpinBox,
            self.r6DoubleSpinBox,
            self.r7DoubleSpinBox,
            self.r8DoubleSpinBox,
        ]

    def load_preset_file(self, path):
        # exception handling required
        self.preset_file = pd.read_csv(path, header=0, index_col="NAME")
        self.is_preset_loaded = True
        self.is_preset_modified = False
        self.preset_status_text.setText("Presets loaded")

        for name in self.preset_file.index:
            self.preset_list_cbx.addItem(name)

    def cb_on_preset_list_cbx_changed(self, name):
        self.l1DoubleSpinBox.setValue(self.preset_file.loc[name]["L1"])
        self.l2DoubleSpinBox.setValue(self.preset_file.loc[name]["L2"])
        self.l3DoubleSpinBox.setValue(self.preset_file.loc[name]["L3"])
        self.l4DoubleSpinBox.setValue(self.preset_file.loc[name]["L4"])
        self.l5DoubleSpinBox.setValue(self.preset_file.loc[name]["L5"])
        self.l6DoubleSpinBox.setValue(self.preset_file.loc[name]["L6"])
        self.l7DoubleSpinBox.setValue(self.preset_file.loc[name]["L7"])
        self.l8DoubleSpinBox.setValue(self.preset_file.loc[name]["L8"])

        self.r1DoubleSpinBox.setValue(self.preset_file.loc[name]["R1"])
        self.r2DoubleSpinBox.setValue(self.preset_file.loc[name]["R2"])
        self.r3DoubleSpinBox.setValue(self.preset_file.loc[name]["R3"])
        self.r4DoubleSpinBox.setValue(self.preset_file.loc[name]["R4"])
        self.r5DoubleSpinBox.setValue(self.preset_file.loc[name]["R5"])
        self.r6DoubleSpinBox.setValue(self.preset_file.loc[name]["R6"])
        self.r7DoubleSpinBox.setValue(self.preset_file.loc[name]["R7"])
        self.r8DoubleSpinBox.setValue(self.preset_file.loc[name]["R8"])

    @property
    def is_on_preset_mode(self):
        return self._is_on_preset_mode

    @is_on_preset_mode.setter
    def is_on_preset_mode(self, value: bool):
        self._is_on_preset_mode = value

    @property
    def is_preset_modified(self):
        return self._is_preset_modified

    @is_preset_modified.setter
    def is_preset_modified(self, value: bool):
        self._is_preset_modified = value
        original_text = self.preset_status_text.text()
        if value and "*" not in original_text:
            self.preset_status_text.setText(original_text + "*")
        elif not value and "*" in original_text:
            self.preset_status_text.setText(original_text[-2])

    @property
    def is_preset_loaded(self):
        return self._is_preset_loaded

    @is_preset_loaded.setter
    def is_preset_loaded(self, value: bool):
        self._is_preset_loaded = value
        if value:
            self.preset_status_text.setText("Loaded")
        else:
            self.preset_status_text.setText("Unloaded")


if __name__ == "__main__":
    app = QApplication()
    preset_diag = UIPresetDiag()
    preset_diag.show()
    app.exec()
