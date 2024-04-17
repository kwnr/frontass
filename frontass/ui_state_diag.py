from PySide6.QtWidgets import QDialog, QTableWidgetItem
from PySide6.QtGui import QColor
from ui_assets.arm_state_diag_ui import Ui_arm_state_diag


class UiStateDiag(QDialog, Ui_arm_state_diag):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setupUi(self)

        self.left_table.setVerticalHeaderLabels([f"L{i}" for i in range(1, 9)])
        self.left_table.setHorizontalHeaderLabels(["pos [deg]", "ref [deg]", "cmd [V]"])
        self.right_table.setVerticalHeaderLabels([f"R{i}" for i in range(1, 9)])
        self.right_table.setHorizontalHeaderLabels(
            ["pos [deg]", "ref [deg]", "cmd [V]"]
        )

    def update_table(self, data: dict):
        """
        update table with data passed
        :param data: {"left_pos", "left_ref", "left_cmd", "right_pos",
                      "right_ref", "right_cmd", "is_on_hold"}
        :return: None
        """
        for i in range(8):
            l_p = QTableWidgetItem(f"{data['left_pos'][i]:.3f}")
            l_r = QTableWidgetItem(f"{data['left_ref'][i]:.3f}")
            l_c = QTableWidgetItem(f"{data['left_cmd'][i]:.3f}")
            r_p = QTableWidgetItem(f"{data['right_pos'][i]:.3f}")
            r_r = QTableWidgetItem(f"{data['right_ref'][i]:.3f}")
            r_c = QTableWidgetItem(f"{data['right_cmd'][i]:.3f}")

            l_r.setBackground(
                QColor(0xAA, 0xAA, 0x00)
                if data["is_on_hold"][i]
                else QColor(0xFF, 0xFF, 0xFF)
            )
            r_r.setBackground(
                QColor(0xAA, 0xAA, 0x00)
                if data["is_on_hold"][i + 8]
                else QColor(0xFF, 0xFF, 0xFF)
            )

            self.left_table.setItem(i, 0, l_p)
            self.left_table.setItem(i, 1, l_r)
            self.left_table.setItem(i, 2, l_c)
            self.right_table.setItem(i, 0, r_p)
            self.right_table.setItem(i, 1, r_r)
            self.right_table.setItem(i, 2, r_c)


if __name__ == "__main__":
    from PySide6.QtWidgets import QApplication

    app = QApplication()
    diag = UiStateDiag()
    diag.show()
    app.exec()
