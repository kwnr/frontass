# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'dxl_control.ui'
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
from PySide6.QtWidgets import (QApplication, QDialog, QLabel, QPushButton,
    QSizePolicy, QSlider, QWidget)

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        if not Dialog.objectName():
            Dialog.setObjectName(u"Dialog")
        Dialog.resize(418, 414)
        Dialog.setStyleSheet(u"QSlider{\n"
"	min-height: 32px;\n"
"	width: 32px;	\n"
"}\n"
"QSlider::groove:vertical {\n"
"	background-color: white;\n"
"}\n"
"QSlider::handle:vertical {\n"
"	height: 16px;\n"
"	background-color: black;\n"
"}\n"
"QSlider::add-page:vertical {\n"
"	background-color: white;\n"
"}\n"
"QSlider::sub-page:vertical {\n"
"	background-color: white;\n"
"}")
        self.verticalSlider = QSlider(Dialog)
        self.verticalSlider.setObjectName(u"verticalSlider")
        self.verticalSlider.setGeometry(QRect(20, 149, 171, 211))
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.verticalSlider.sizePolicy().hasHeightForWidth())
        self.verticalSlider.setSizePolicy(sizePolicy)
        self.verticalSlider.setStyleSheet(u"QSlider{\n"
"	min-height: 32px;\n"
"	width: 32px;	\n"
"}\n"
"QSlider::groove:vertical {\n"
"	background-color: white;\n"
"}\n"
"QSlider::handle:vertical {\n"
"	height: 16px;\n"
"	background-color: black;\n"
"}\n"
"QSlider::add-page:vertical {\n"
"	background-color: white;\n"
"}\n"
"QSlider::sub-page:vertical {\n"
"	background-color: white;\n"
"}")
        self.verticalSlider.setMinimum(-99)
        self.verticalSlider.setOrientation(Qt.Orientation.Vertical)
        self.verticalSlider.setInvertedControls(True)
        self.pushButton = QPushButton(Dialog)
        self.pushButton.setObjectName(u"pushButton")
        self.pushButton.setGeometry(QRect(20, 60, 51, 81))
        self.pushButton_2 = QPushButton(Dialog)
        self.pushButton_2.setObjectName(u"pushButton_2")
        self.pushButton_2.setGeometry(QRect(80, 60, 51, 81))
        self.pushButton_3 = QPushButton(Dialog)
        self.pushButton_3.setObjectName(u"pushButton_3")
        self.pushButton_3.setGeometry(QRect(140, 60, 51, 81))
        self.pushButton_4 = QPushButton(Dialog)
        self.pushButton_4.setObjectName(u"pushButton_4")
        self.pushButton_4.setGeometry(QRect(280, 60, 51, 81))
        self.pushButton_5 = QPushButton(Dialog)
        self.pushButton_5.setObjectName(u"pushButton_5")
        self.pushButton_5.setGeometry(QRect(340, 60, 51, 81))
        self.pushButton_6 = QPushButton(Dialog)
        self.pushButton_6.setObjectName(u"pushButton_6")
        self.pushButton_6.setGeometry(QRect(220, 60, 51, 81))
        self.verticalSlider_2 = QSlider(Dialog)
        self.verticalSlider_2.setObjectName(u"verticalSlider_2")
        self.verticalSlider_2.setGeometry(QRect(220, 149, 171, 211))
        sizePolicy.setHeightForWidth(self.verticalSlider_2.sizePolicy().hasHeightForWidth())
        self.verticalSlider_2.setSizePolicy(sizePolicy)
        self.verticalSlider_2.setStyleSheet(u"QSlider{\n"
"	min-height: 32px;\n"
"	width: 32px;	\n"
"}\n"
"QSlider::groove:vertical {\n"
"	background-color: white;\n"
"}\n"
"QSlider::handle:vertical {\n"
"	height: 16px;\n"
"	background-color: black;\n"
"}\n"
"QSlider::add-page:vertical {\n"
"	background-color: white;\n"
"}\n"
"QSlider::sub-page:vertical {\n"
"	background-color: white;\n"
"}")
        self.verticalSlider_2.setMinimum(-99)
        self.verticalSlider_2.setOrientation(Qt.Orientation.Vertical)
        self.verticalSlider_2.setInvertedControls(True)
        self.label = QLabel(Dialog)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(20, 40, 51, 17))
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_3 = QLabel(Dialog)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(80, 40, 51, 17))
        self.label_3.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_4 = QLabel(Dialog)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(140, 40, 51, 17))
        self.label_4.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_2 = QLabel(Dialog)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(220, 40, 51, 17))
        self.label_2.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_5 = QLabel(Dialog)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setGeometry(QRect(280, 40, 51, 17))
        self.label_5.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_6 = QLabel(Dialog)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setGeometry(QRect(340, 40, 51, 17))
        self.label_6.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_7 = QLabel(Dialog)
        self.label_7.setObjectName(u"label_7")
        self.label_7.setGeometry(QRect(20, 360, 171, 31))
        self.label_7.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_8 = QLabel(Dialog)
        self.label_8.setObjectName(u"label_8")
        self.label_8.setGeometry(QRect(220, 360, 171, 31))
        self.label_8.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.retranslateUi(Dialog)

        QMetaObject.connectSlotsByName(Dialog)
    # setupUi

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", u"Dialog", None))
        self.pushButton.setText(QCoreApplication.translate("Dialog", u"I", None))
        self.pushButton_2.setText(QCoreApplication.translate("Dialog", u"O", None))
        self.pushButton_3.setText(QCoreApplication.translate("Dialog", u"II", None))
        self.pushButton_4.setText(QCoreApplication.translate("Dialog", u"O", None))
        self.pushButton_5.setText(QCoreApplication.translate("Dialog", u"II", None))
        self.pushButton_6.setText(QCoreApplication.translate("Dialog", u"I", None))
        self.label.setText(QCoreApplication.translate("Dialog", u"UP", None))
        self.label_3.setText(QCoreApplication.translate("Dialog", u"NEUT", None))
        self.label_4.setText(QCoreApplication.translate("Dialog", u"DOWN", None))
        self.label_2.setText(QCoreApplication.translate("Dialog", u"CW", None))
        self.label_5.setText(QCoreApplication.translate("Dialog", u"TRACK", None))
        self.label_6.setText(QCoreApplication.translate("Dialog", u"CCW", None))
        self.label_7.setText(QCoreApplication.translate("Dialog", u"Track LEFT", None))
        self.label_8.setText(QCoreApplication.translate("Dialog", u"Track RIGHT\n"
"TRIGGER", None))
    # retranslateUi

