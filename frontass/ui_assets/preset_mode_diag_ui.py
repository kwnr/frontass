# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'preset_mode_diag.ui'
##
## Created by: Qt User Interface Compiler version 6.2.4
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QAbstractButton, QApplication, QCheckBox, QComboBox,
    QDialog, QDialogButtonBox, QDoubleSpinBox, QFormLayout,
    QLabel, QLineEdit, QPushButton, QSizePolicy,
    QToolButton, QWidget)
import buttons_rc

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        if not Dialog.objectName():
            Dialog.setObjectName(u"Dialog")
        Dialog.resize(565, 306)
        self.formLayoutWidget = QWidget(Dialog)
        self.formLayoutWidget.setObjectName(u"formLayoutWidget")
        self.formLayoutWidget.setGeometry(QRect(20, 20, 101, 261))
        self.formLayout = QFormLayout(self.formLayoutWidget)
        self.formLayout.setObjectName(u"formLayout")
        self.formLayout.setContentsMargins(0, 0, 0, 0)
        self.l1Label = QLabel(self.formLayoutWidget)
        self.l1Label.setObjectName(u"l1Label")

        self.formLayout.setWidget(0, QFormLayout.LabelRole, self.l1Label)

        self.l1DoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget)
        self.l1DoubleSpinBox.setObjectName(u"l1DoubleSpinBox")
        self.l1DoubleSpinBox.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.l1DoubleSpinBox.setMinimum(-99.989999999999995)

        self.formLayout.setWidget(0, QFormLayout.FieldRole, self.l1DoubleSpinBox)

        self.l2Label = QLabel(self.formLayoutWidget)
        self.l2Label.setObjectName(u"l2Label")

        self.formLayout.setWidget(1, QFormLayout.LabelRole, self.l2Label)

        self.l2DoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget)
        self.l2DoubleSpinBox.setObjectName(u"l2DoubleSpinBox")
        self.l2DoubleSpinBox.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.l2DoubleSpinBox.setMinimum(-99.989999999999995)

        self.formLayout.setWidget(1, QFormLayout.FieldRole, self.l2DoubleSpinBox)

        self.l3Label = QLabel(self.formLayoutWidget)
        self.l3Label.setObjectName(u"l3Label")

        self.formLayout.setWidget(2, QFormLayout.LabelRole, self.l3Label)

        self.l3DoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget)
        self.l3DoubleSpinBox.setObjectName(u"l3DoubleSpinBox")
        self.l3DoubleSpinBox.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.l3DoubleSpinBox.setMinimum(-99.989999999999995)
        self.l3DoubleSpinBox.setValue(20.000000000000000)

        self.formLayout.setWidget(2, QFormLayout.FieldRole, self.l3DoubleSpinBox)

        self.l4Label = QLabel(self.formLayoutWidget)
        self.l4Label.setObjectName(u"l4Label")

        self.formLayout.setWidget(3, QFormLayout.LabelRole, self.l4Label)

        self.l4DoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget)
        self.l4DoubleSpinBox.setObjectName(u"l4DoubleSpinBox")
        self.l4DoubleSpinBox.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.l4DoubleSpinBox.setMinimum(-99.989999999999995)

        self.formLayout.setWidget(3, QFormLayout.FieldRole, self.l4DoubleSpinBox)

        self.l5Label = QLabel(self.formLayoutWidget)
        self.l5Label.setObjectName(u"l5Label")

        self.formLayout.setWidget(4, QFormLayout.LabelRole, self.l5Label)

        self.l5DoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget)
        self.l5DoubleSpinBox.setObjectName(u"l5DoubleSpinBox")
        self.l5DoubleSpinBox.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.l5DoubleSpinBox.setMinimum(-99.989999999999995)

        self.formLayout.setWidget(4, QFormLayout.FieldRole, self.l5DoubleSpinBox)

        self.l6Label = QLabel(self.formLayoutWidget)
        self.l6Label.setObjectName(u"l6Label")

        self.formLayout.setWidget(5, QFormLayout.LabelRole, self.l6Label)

        self.l6DoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget)
        self.l6DoubleSpinBox.setObjectName(u"l6DoubleSpinBox")
        self.l6DoubleSpinBox.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.l6DoubleSpinBox.setMinimum(-99.989999999999995)

        self.formLayout.setWidget(5, QFormLayout.FieldRole, self.l6DoubleSpinBox)

        self.l7Label = QLabel(self.formLayoutWidget)
        self.l7Label.setObjectName(u"l7Label")

        self.formLayout.setWidget(6, QFormLayout.LabelRole, self.l7Label)

        self.l7DoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget)
        self.l7DoubleSpinBox.setObjectName(u"l7DoubleSpinBox")
        self.l7DoubleSpinBox.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.l7DoubleSpinBox.setMinimum(-99.989999999999995)

        self.formLayout.setWidget(6, QFormLayout.FieldRole, self.l7DoubleSpinBox)

        self.l8Label = QLabel(self.formLayoutWidget)
        self.l8Label.setObjectName(u"l8Label")

        self.formLayout.setWidget(7, QFormLayout.LabelRole, self.l8Label)

        self.l8DoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget)
        self.l8DoubleSpinBox.setObjectName(u"l8DoubleSpinBox")
        self.l8DoubleSpinBox.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.l8DoubleSpinBox.setMinimum(-99.989999999999995)

        self.formLayout.setWidget(7, QFormLayout.FieldRole, self.l8DoubleSpinBox)

        self.formLayoutWidget_2 = QWidget(Dialog)
        self.formLayoutWidget_2.setObjectName(u"formLayoutWidget_2")
        self.formLayoutWidget_2.setGeometry(QRect(140, 20, 101, 261))
        self.formLayout_2 = QFormLayout(self.formLayoutWidget_2)
        self.formLayout_2.setObjectName(u"formLayout_2")
        self.formLayout_2.setContentsMargins(0, 0, 0, 0)
        self.r1Label = QLabel(self.formLayoutWidget_2)
        self.r1Label.setObjectName(u"r1Label")

        self.formLayout_2.setWidget(0, QFormLayout.LabelRole, self.r1Label)

        self.r1DoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget_2)
        self.r1DoubleSpinBox.setObjectName(u"r1DoubleSpinBox")
        self.r1DoubleSpinBox.setAlignment(Qt.AlignRight|Qt.AlignTop|Qt.AlignTrailing)
        self.r1DoubleSpinBox.setMinimum(-99.989999999999995)

        self.formLayout_2.setWidget(0, QFormLayout.FieldRole, self.r1DoubleSpinBox)

        self.r2Label = QLabel(self.formLayoutWidget_2)
        self.r2Label.setObjectName(u"r2Label")

        self.formLayout_2.setWidget(1, QFormLayout.LabelRole, self.r2Label)

        self.r2DoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget_2)
        self.r2DoubleSpinBox.setObjectName(u"r2DoubleSpinBox")
        self.r2DoubleSpinBox.setAlignment(Qt.AlignRight|Qt.AlignTop|Qt.AlignTrailing)
        self.r2DoubleSpinBox.setMinimum(-99.989999999999995)

        self.formLayout_2.setWidget(1, QFormLayout.FieldRole, self.r2DoubleSpinBox)

        self.r3Label = QLabel(self.formLayoutWidget_2)
        self.r3Label.setObjectName(u"r3Label")

        self.formLayout_2.setWidget(2, QFormLayout.LabelRole, self.r3Label)

        self.r3DoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget_2)
        self.r3DoubleSpinBox.setObjectName(u"r3DoubleSpinBox")
        self.r3DoubleSpinBox.setAlignment(Qt.AlignRight|Qt.AlignTop|Qt.AlignTrailing)
        self.r3DoubleSpinBox.setMinimum(-99.989999999999995)
        self.r3DoubleSpinBox.setValue(20.000000000000000)

        self.formLayout_2.setWidget(2, QFormLayout.FieldRole, self.r3DoubleSpinBox)

        self.r4Label = QLabel(self.formLayoutWidget_2)
        self.r4Label.setObjectName(u"r4Label")

        self.formLayout_2.setWidget(3, QFormLayout.LabelRole, self.r4Label)

        self.r4DoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget_2)
        self.r4DoubleSpinBox.setObjectName(u"r4DoubleSpinBox")
        self.r4DoubleSpinBox.setAlignment(Qt.AlignRight|Qt.AlignTop|Qt.AlignTrailing)
        self.r4DoubleSpinBox.setMinimum(-99.989999999999995)

        self.formLayout_2.setWidget(3, QFormLayout.FieldRole, self.r4DoubleSpinBox)

        self.r5Label = QLabel(self.formLayoutWidget_2)
        self.r5Label.setObjectName(u"r5Label")

        self.formLayout_2.setWidget(4, QFormLayout.LabelRole, self.r5Label)

        self.r5DoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget_2)
        self.r5DoubleSpinBox.setObjectName(u"r5DoubleSpinBox")
        self.r5DoubleSpinBox.setAlignment(Qt.AlignRight|Qt.AlignTop|Qt.AlignTrailing)
        self.r5DoubleSpinBox.setMinimum(-99.989999999999995)

        self.formLayout_2.setWidget(4, QFormLayout.FieldRole, self.r5DoubleSpinBox)

        self.r6Label = QLabel(self.formLayoutWidget_2)
        self.r6Label.setObjectName(u"r6Label")

        self.formLayout_2.setWidget(5, QFormLayout.LabelRole, self.r6Label)

        self.r6DoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget_2)
        self.r6DoubleSpinBox.setObjectName(u"r6DoubleSpinBox")
        self.r6DoubleSpinBox.setAlignment(Qt.AlignRight|Qt.AlignTop|Qt.AlignTrailing)
        self.r6DoubleSpinBox.setMinimum(-99.989999999999995)

        self.formLayout_2.setWidget(5, QFormLayout.FieldRole, self.r6DoubleSpinBox)

        self.r7Label = QLabel(self.formLayoutWidget_2)
        self.r7Label.setObjectName(u"r7Label")

        self.formLayout_2.setWidget(6, QFormLayout.LabelRole, self.r7Label)

        self.r7DoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget_2)
        self.r7DoubleSpinBox.setObjectName(u"r7DoubleSpinBox")
        self.r7DoubleSpinBox.setAlignment(Qt.AlignRight|Qt.AlignTop|Qt.AlignTrailing)
        self.r7DoubleSpinBox.setMinimum(-99.989999999999995)

        self.formLayout_2.setWidget(6, QFormLayout.FieldRole, self.r7DoubleSpinBox)

        self.r8Label = QLabel(self.formLayoutWidget_2)
        self.r8Label.setObjectName(u"r8Label")

        self.formLayout_2.setWidget(7, QFormLayout.LabelRole, self.r8Label)

        self.r8DoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget_2)
        self.r8DoubleSpinBox.setObjectName(u"r8DoubleSpinBox")
        self.r8DoubleSpinBox.setAlignment(Qt.AlignRight|Qt.AlignTop|Qt.AlignTrailing)
        self.r8DoubleSpinBox.setMinimum(-99.989999999999995)

        self.formLayout_2.setWidget(7, QFormLayout.FieldRole, self.r8DoubleSpinBox)

        self.apply_inst_cbx = QCheckBox(Dialog)
        self.apply_inst_cbx.setObjectName(u"apply_inst_cbx")
        self.apply_inst_cbx.setGeometry(QRect(260, 170, 251, 25))
        self.buttonBox = QDialogButtonBox(Dialog)
        self.buttonBox.setObjectName(u"buttonBox")
        self.buttonBox.setGeometry(QRect(260, 240, 291, 30))
        self.buttonBox.setStandardButtons(QDialogButtonBox.Apply|QDialogButtonBox.Cancel|QDialogButtonBox.Ok)
        self.buttonBox.setCenterButtons(False)
        self.toolButton = QToolButton(Dialog)
        self.toolButton.setObjectName(u"toolButton")
        self.toolButton.setGeometry(QRect(530, 70, 26, 25))
        self.preset_file_path = QLineEdit(Dialog)
        self.preset_file_path.setObjectName(u"preset_file_path")
        self.preset_file_path.setGeometry(QRect(370, 70, 151, 25))
        self.label = QLabel(Dialog)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(260, 70, 81, 25))
        self.label_2 = QLabel(Dialog)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(260, 130, 101, 25))
        self.preset_list_cbx = QComboBox(Dialog)
        self.preset_list_cbx.setObjectName(u"preset_list_cbx")
        self.preset_list_cbx.setGeometry(QRect(370, 130, 181, 25))
        self.label_3 = QLabel(Dialog)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(260, 100, 101, 25))
        self.enable_preset_mode_btn = QPushButton(Dialog)
        self.enable_preset_mode_btn.setObjectName(u"enable_preset_mode_btn")
        self.enable_preset_mode_btn.setEnabled(True)
        self.enable_preset_mode_btn.setGeometry(QRect(510, 20, 44, 30))
        self.enable_preset_mode_btn.setMinimumSize(QSize(41, 21))
        self.enable_preset_mode_btn.setBaseSize(QSize(41, 21))
        self.enable_preset_mode_btn.setStyleSheet(u"QPushButton{\n"
"	background-color: none;\n"
"	border-radius: 5%;\n"
"}")
        icon = QIcon()
        icon.addFile(u":/toggle/off", QSize(), QIcon.Normal, QIcon.Off)
        icon.addFile(u":/toggle/on", QSize(), QIcon.Normal, QIcon.On)
        self.enable_preset_mode_btn.setIcon(icon)
        self.enable_preset_mode_btn.setIconSize(QSize(40, 40))
        self.enable_preset_mode_btn.setCheckable(True)
        self.enable_preset_mode_btn.setChecked(False)
        self.label_4 = QLabel(Dialog)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(260, 20, 221, 25))
        self.preset_status_text = QLabel(Dialog)
        self.preset_status_text.setObjectName(u"preset_status_text")
        self.preset_status_text.setGeometry(QRect(370, 100, 181, 25))

        self.retranslateUi(Dialog)
        self.buttonBox.accepted.connect(Dialog.accept)
        self.buttonBox.rejected.connect(Dialog.reject)

        QMetaObject.connectSlotsByName(Dialog)
    # setupUi

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", u"Preset", None))
        self.l1Label.setText(QCoreApplication.translate("Dialog", u"L1", None))
        self.l2Label.setText(QCoreApplication.translate("Dialog", u"L2", None))
        self.l3Label.setText(QCoreApplication.translate("Dialog", u"L3", None))
        self.l4Label.setText(QCoreApplication.translate("Dialog", u"L4", None))
        self.l5Label.setText(QCoreApplication.translate("Dialog", u"L5", None))
        self.l6Label.setText(QCoreApplication.translate("Dialog", u"L6", None))
        self.l7Label.setText(QCoreApplication.translate("Dialog", u"L7", None))
        self.l8Label.setText(QCoreApplication.translate("Dialog", u"L8", None))
        self.r1Label.setText(QCoreApplication.translate("Dialog", u"R1", None))
        self.r2Label.setText(QCoreApplication.translate("Dialog", u"R2", None))
        self.r3Label.setText(QCoreApplication.translate("Dialog", u"R3", None))
        self.r4Label.setText(QCoreApplication.translate("Dialog", u"R4", None))
        self.r5Label.setText(QCoreApplication.translate("Dialog", u"R5", None))
        self.r6Label.setText(QCoreApplication.translate("Dialog", u"R6", None))
        self.r7Label.setText(QCoreApplication.translate("Dialog", u"R7", None))
        self.r8Label.setText(QCoreApplication.translate("Dialog", u"R8", None))
        self.apply_inst_cbx.setText(QCoreApplication.translate("Dialog", u"Apply instantly", None))
        self.toolButton.setText(QCoreApplication.translate("Dialog", u"...", None))
        self.label.setText(QCoreApplication.translate("Dialog", u"Preset file", None))
        self.label_2.setText(QCoreApplication.translate("Dialog", u"Current Preset", None))
        self.label_3.setText(QCoreApplication.translate("Dialog", u"Preset Status", None))
        self.enable_preset_mode_btn.setText("")
        self.label_4.setText(QCoreApplication.translate("Dialog", u"Enable Preset Mode", None))
        self.preset_status_text.setText(QCoreApplication.translate("Dialog", u"Not Loaded", None))
    # retranslateUi

