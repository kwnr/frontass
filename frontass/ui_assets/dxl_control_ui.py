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
        self.leftSlider = QSlider(Dialog)
        self.leftSlider.setObjectName(u"leftSlider")
        self.leftSlider.setGeometry(QRect(20, 149, 171, 211))
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.leftSlider.sizePolicy().hasHeightForWidth())
        self.leftSlider.setSizePolicy(sizePolicy)
        self.leftSlider.setStyleSheet(u"QSlider{\n"
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
        self.leftSlider.setMinimum(-99)
        self.leftSlider.setOrientation(Qt.Orientation.Vertical)
        self.leftSlider.setInvertedControls(True)
        self.upBtn = QPushButton(Dialog)
        self.upBtn.setObjectName(u"upBtn")
        self.upBtn.setGeometry(QRect(20, 60, 51, 81))
        self.neutBtn = QPushButton(Dialog)
        self.neutBtn.setObjectName(u"neutBtn")
        self.neutBtn.setGeometry(QRect(80, 60, 51, 81))
        self.neutBtn.setCheckable(True)
        self.neutBtn.setChecked(True)
        self.downBtn = QPushButton(Dialog)
        self.downBtn.setObjectName(u"downBtn")
        self.downBtn.setGeometry(QRect(140, 60, 51, 81))
        self.trackBtn = QPushButton(Dialog)
        self.trackBtn.setObjectName(u"trackBtn")
        self.trackBtn.setGeometry(QRect(280, 60, 51, 81))
        self.trackBtn.setCheckable(True)
        self.trackBtn.setChecked(True)
        self.ccwBtn = QPushButton(Dialog)
        self.ccwBtn.setObjectName(u"ccwBtn")
        self.ccwBtn.setGeometry(QRect(340, 60, 51, 81))
        self.ccwBtn.setCheckable(True)
        self.cwBtn = QPushButton(Dialog)
        self.cwBtn.setObjectName(u"cwBtn")
        self.cwBtn.setGeometry(QRect(220, 60, 51, 81))
        self.cwBtn.setCheckable(True)
        self.rightSlider = QSlider(Dialog)
        self.rightSlider.setObjectName(u"rightSlider")
        self.rightSlider.setGeometry(QRect(220, 149, 171, 211))
        sizePolicy.setHeightForWidth(self.rightSlider.sizePolicy().hasHeightForWidth())
        self.rightSlider.setSizePolicy(sizePolicy)
        self.rightSlider.setStyleSheet(u"QSlider{\n"
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
        self.rightSlider.setMinimum(-99)
        self.rightSlider.setOrientation(Qt.Orientation.Vertical)
        self.rightSlider.setInvertedControls(True)
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
        self.upBtn.setText(QCoreApplication.translate("Dialog", u"I", None))
        self.neutBtn.setText(QCoreApplication.translate("Dialog", u"O", None))
        self.downBtn.setText(QCoreApplication.translate("Dialog", u"II", None))
        self.trackBtn.setText(QCoreApplication.translate("Dialog", u"O", None))
        self.ccwBtn.setText(QCoreApplication.translate("Dialog", u"II", None))
        self.cwBtn.setText(QCoreApplication.translate("Dialog", u"I", None))
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

