# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'arm_state_diag.ui'
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
from PySide6.QtWidgets import (QAbstractItemView, QAbstractScrollArea, QApplication, QDialog,
    QHeaderView, QLabel, QSizePolicy, QTableWidget,
    QTableWidgetItem, QWidget)

class Ui_arm_state_diag(object):
    def setupUi(self, arm_state_diag):
        if not arm_state_diag.objectName():
            arm_state_diag.setObjectName(u"arm_state_diag")
        arm_state_diag.resize(925, 425)
        self.label = QLabel(arm_state_diag)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(10, 10, 111, 21))
        font = QFont()
        font.setPointSize(14)
        self.label.setFont(font)
        self.label_3 = QLabel(arm_state_diag)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(470, 10, 111, 21))
        self.label_3.setFont(font)
        self.left_table = QTableWidget(arm_state_diag)
        if (self.left_table.columnCount() < 3):
            self.left_table.setColumnCount(3)
        if (self.left_table.rowCount() < 8):
            self.left_table.setRowCount(8)
        self.left_table.setObjectName(u"left_table")
        self.left_table.setGeometry(QRect(10, 40, 441, 361))
        self.left_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.left_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.left_table.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)
        self.left_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.left_table.setAlternatingRowColors(False)
        self.left_table.setSelectionMode(QAbstractItemView.NoSelection)
        self.left_table.setRowCount(8)
        self.left_table.setColumnCount(3)
        self.left_table.horizontalHeader().setDefaultSectionSize(135)
        self.left_table.horizontalHeader().setStretchLastSection(True)
        self.left_table.verticalHeader().setDefaultSectionSize(40)
        self.left_table.verticalHeader().setStretchLastSection(True)
        self.right_table = QTableWidget(arm_state_diag)
        if (self.right_table.columnCount() < 3):
            self.right_table.setColumnCount(3)
        if (self.right_table.rowCount() < 8):
            self.right_table.setRowCount(8)
        self.right_table.setObjectName(u"right_table")
        self.right_table.setGeometry(QRect(470, 40, 441, 361))
        self.right_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.right_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.right_table.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)
        self.right_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.right_table.setSelectionMode(QAbstractItemView.NoSelection)
        self.right_table.setRowCount(8)
        self.right_table.setColumnCount(3)
        self.right_table.horizontalHeader().setDefaultSectionSize(135)
        self.right_table.horizontalHeader().setStretchLastSection(True)
        self.right_table.verticalHeader().setDefaultSectionSize(40)
        self.right_table.verticalHeader().setStretchLastSection(True)

        self.retranslateUi(arm_state_diag)

        QMetaObject.connectSlotsByName(arm_state_diag)
    # setupUi

    def retranslateUi(self, arm_state_diag):
        arm_state_diag.setWindowTitle(QCoreApplication.translate("arm_state_diag", u"Armstrong State", None))
        self.label.setText(QCoreApplication.translate("arm_state_diag", u"Left Arm", None))
        self.label_3.setText(QCoreApplication.translate("arm_state_diag", u"Right Arm", None))
    # retranslateUi

