# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'IK.ui'
##
## Created by: Qt User Interface Compiler version 6.7.0
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
from PySide6.QtWidgets import (QApplication, QDialog, QFormLayout, QHeaderView,
    QLabel, QLineEdit, QPushButton, QSizePolicy,
    QTableWidget, QTableWidgetItem, QWidget)
import buttons_rc

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        if not Dialog.objectName():
            Dialog.setObjectName(u"Dialog")
        Dialog.resize(562, 772)
        self.formLayoutWidget = QWidget(Dialog)
        self.formLayoutWidget.setObjectName(u"formLayoutWidget")
        self.formLayoutWidget.setGeometry(QRect(20, 100, 160, 104))
        self.current_position = QFormLayout(self.formLayoutWidget)
        self.current_position.setObjectName(u"current_position")
        self.current_position.setContentsMargins(0, 0, 0, 0)
        self.xLabel = QLabel(self.formLayoutWidget)
        self.xLabel.setObjectName(u"xLabel")
        self.xLabel.setMinimumSize(QSize(20, 30))

        self.current_position.setWidget(0, QFormLayout.LabelRole, self.xLabel)

        self.xCurPosLineEdit = QLineEdit(self.formLayoutWidget)
        self.xCurPosLineEdit.setObjectName(u"xCurPosLineEdit")

        self.current_position.setWidget(0, QFormLayout.FieldRole, self.xCurPosLineEdit)

        self.yLabel = QLabel(self.formLayoutWidget)
        self.yLabel.setObjectName(u"yLabel")
        self.yLabel.setMinimumSize(QSize(0, 30))

        self.current_position.setWidget(1, QFormLayout.LabelRole, self.yLabel)

        self.yCurPosLineEdit = QLineEdit(self.formLayoutWidget)
        self.yCurPosLineEdit.setObjectName(u"yCurPosLineEdit")

        self.current_position.setWidget(1, QFormLayout.FieldRole, self.yCurPosLineEdit)

        self.zLabel = QLabel(self.formLayoutWidget)
        self.zLabel.setObjectName(u"zLabel")
        self.zLabel.setMinimumSize(QSize(0, 30))

        self.current_position.setWidget(2, QFormLayout.LabelRole, self.zLabel)

        self.zCurPosLineEdit = QLineEdit(self.formLayoutWidget)
        self.zCurPosLineEdit.setObjectName(u"zCurPosLineEdit")

        self.current_position.setWidget(2, QFormLayout.FieldRole, self.zCurPosLineEdit)

        self.formLayoutWidget_3 = QWidget(Dialog)
        self.formLayoutWidget_3.setObjectName(u"formLayoutWidget_3")
        self.formLayoutWidget_3.setGeometry(QRect(200, 100, 160, 140))
        self.formLayout = QFormLayout(self.formLayoutWidget_3)
        self.formLayout.setObjectName(u"formLayout")
        self.formLayout.setContentsMargins(0, 0, 0, 0)
        self.xCurOriLabel = QLabel(self.formLayoutWidget_3)
        self.xCurOriLabel.setObjectName(u"xCurOriLabel")
        self.xCurOriLabel.setMinimumSize(QSize(20, 30))

        self.formLayout.setWidget(0, QFormLayout.LabelRole, self.xCurOriLabel)

        self.xCurOriLineEdit = QLineEdit(self.formLayoutWidget_3)
        self.xCurOriLineEdit.setObjectName(u"xCurOriLineEdit")

        self.formLayout.setWidget(0, QFormLayout.FieldRole, self.xCurOriLineEdit)

        self.yCurOriLabel = QLabel(self.formLayoutWidget_3)
        self.yCurOriLabel.setObjectName(u"yCurOriLabel")
        self.yCurOriLabel.setMinimumSize(QSize(20, 30))

        self.formLayout.setWidget(1, QFormLayout.LabelRole, self.yCurOriLabel)

        self.yCurOriLineEdit = QLineEdit(self.formLayoutWidget_3)
        self.yCurOriLineEdit.setObjectName(u"yCurOriLineEdit")

        self.formLayout.setWidget(1, QFormLayout.FieldRole, self.yCurOriLineEdit)

        self.zCurOriLabel = QLabel(self.formLayoutWidget_3)
        self.zCurOriLabel.setObjectName(u"zCurOriLabel")
        self.zCurOriLabel.setMinimumSize(QSize(20, 30))

        self.formLayout.setWidget(2, QFormLayout.LabelRole, self.zCurOriLabel)

        self.zCurOriLineEdit = QLineEdit(self.formLayoutWidget_3)
        self.zCurOriLineEdit.setObjectName(u"zCurOriLineEdit")

        self.formLayout.setWidget(2, QFormLayout.FieldRole, self.zCurOriLineEdit)

        self.wCurOriLabel = QLabel(self.formLayoutWidget_3)
        self.wCurOriLabel.setObjectName(u"wCurOriLabel")
        self.wCurOriLabel.setMinimumSize(QSize(20, 30))

        self.formLayout.setWidget(3, QFormLayout.LabelRole, self.wCurOriLabel)

        self.wCurOriLineEdit = QLineEdit(self.formLayoutWidget_3)
        self.wCurOriLineEdit.setObjectName(u"wCurOriLineEdit")

        self.formLayout.setWidget(3, QFormLayout.FieldRole, self.wCurOriLineEdit)

        self.label = QLabel(Dialog)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(20, 80, 67, 17))
        self.label_2 = QLabel(Dialog)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(200, 80, 91, 17))
        self.l1_power_btn = QPushButton(Dialog)
        self.l1_power_btn.setObjectName(u"l1_power_btn")
        self.l1_power_btn.setEnabled(True)
        self.l1_power_btn.setGeometry(QRect(120, 20, 41, 41))
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.l1_power_btn.sizePolicy().hasHeightForWidth())
        self.l1_power_btn.setSizePolicy(sizePolicy)
        self.l1_power_btn.setMinimumSize(QSize(41, 21))
        self.l1_power_btn.setMaximumSize(QSize(16777215, 16777215))
        self.l1_power_btn.setBaseSize(QSize(41, 21))
        self.l1_power_btn.setStyleSheet(u"QPushButton{\n"
"	background-color: none;\n"
"	border-radius: 3%;\n"
"}")
        icon = QIcon()
        icon.addFile(u":/toggle/off", QSize(), QIcon.Normal, QIcon.Off)
        icon.addFile(u":/toggle/on", QSize(), QIcon.Normal, QIcon.On)
        self.l1_power_btn.setIcon(icon)
        self.l1_power_btn.setIconSize(QSize(41, 40))
        self.l1_power_btn.setCheckable(True)
        self.l1_power_btn.setChecked(True)
        self.l1_power_btn.setFlat(True)
        self.label_3 = QLabel(Dialog)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(20, 20, 91, 41))
        self.pushButton = QPushButton(Dialog)
        self.pushButton.setObjectName(u"pushButton")
        self.pushButton.setGeometry(QRect(390, 100, 150, 50))
        self.pushButton_2 = QPushButton(Dialog)
        self.pushButton_2.setObjectName(u"pushButton_2")
        self.pushButton_2.setEnabled(False)
        self.pushButton_2.setGeometry(QRect(390, 160, 150, 50))
        self.tableWidget = QTableWidget(Dialog)
        self.tableWidget.setObjectName(u"tableWidget")
        self.tableWidget.setGeometry(QRect(10, 280, 521, 481))
        self.label_4 = QLabel(Dialog)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(390, 220, 151, 17))

        self.retranslateUi(Dialog)

        QMetaObject.connectSlotsByName(Dialog)
    # setupUi

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", u"IK mode", None))
        self.xLabel.setText(QCoreApplication.translate("Dialog", u"x", None))
        self.yLabel.setText(QCoreApplication.translate("Dialog", u"y", None))
        self.zLabel.setText(QCoreApplication.translate("Dialog", u"z", None))
        self.xCurOriLabel.setText(QCoreApplication.translate("Dialog", u"x", None))
        self.yCurOriLabel.setText(QCoreApplication.translate("Dialog", u"y", None))
        self.zCurOriLabel.setText(QCoreApplication.translate("Dialog", u"z", None))
        self.wCurOriLabel.setText(QCoreApplication.translate("Dialog", u"w", None))
        self.label.setText(QCoreApplication.translate("Dialog", u"Position", None))
        self.label_2.setText(QCoreApplication.translate("Dialog", u"Orientation", None))
        self.l1_power_btn.setText("")
        self.label_3.setText(QCoreApplication.translate("Dialog", u"IK mode\n"
"Enabled", None))
        self.pushButton.setText(QCoreApplication.translate("Dialog", u"PLAN", None))
        self.pushButton_2.setText(QCoreApplication.translate("Dialog", u"EXECUTE", None))
        self.label_4.setText(QCoreApplication.translate("Dialog", u"status: NOT PLANNED", None))
    # retranslateUi

