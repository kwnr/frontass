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
        Dialog.resize(568, 830)
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
        self.IKEnableBtn = QPushButton(Dialog)
        self.IKEnableBtn.setObjectName(u"IKEnableBtn")
        self.IKEnableBtn.setEnabled(True)
        self.IKEnableBtn.setGeometry(QRect(120, 20, 41, 41))
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.IKEnableBtn.sizePolicy().hasHeightForWidth())
        self.IKEnableBtn.setSizePolicy(sizePolicy)
        self.IKEnableBtn.setMinimumSize(QSize(41, 21))
        self.IKEnableBtn.setMaximumSize(QSize(16777215, 16777215))
        self.IKEnableBtn.setBaseSize(QSize(41, 21))
        self.IKEnableBtn.setStyleSheet(u"QPushButton{\n"
"	background-color: none;\n"
"	border-radius: 3%;\n"
"}")
        icon = QIcon()
        icon.addFile(u":/toggle/off", QSize(), QIcon.Normal, QIcon.Off)
        icon.addFile(u":/toggle/on", QSize(), QIcon.Normal, QIcon.On)
        self.IKEnableBtn.setIcon(icon)
        self.IKEnableBtn.setIconSize(QSize(41, 40))
        self.IKEnableBtn.setCheckable(True)
        self.IKEnableBtn.setChecked(False)
        self.IKEnableBtn.setFlat(True)
        self.label_3 = QLabel(Dialog)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(20, 20, 91, 41))
        self.planBtn = QPushButton(Dialog)
        self.planBtn.setObjectName(u"planBtn")
        self.planBtn.setGeometry(QRect(390, 100, 161, 50))
        self.ExecBtn = QPushButton(Dialog)
        self.ExecBtn.setObjectName(u"ExecBtn")
        self.ExecBtn.setEnabled(False)
        self.ExecBtn.setGeometry(QRect(390, 160, 161, 50))
        self.iter_table = QTableWidget(Dialog)
        if (self.iter_table.columnCount() < 1):
            self.iter_table.setColumnCount(1)
        __qtablewidgetitem = QTableWidgetItem()
        self.iter_table.setHorizontalHeaderItem(0, __qtablewidgetitem)
        if (self.iter_table.rowCount() < 17):
            self.iter_table.setRowCount(17)
        __qtablewidgetitem1 = QTableWidgetItem()
        self.iter_table.setVerticalHeaderItem(0, __qtablewidgetitem1)
        __qtablewidgetitem2 = QTableWidgetItem()
        self.iter_table.setVerticalHeaderItem(1, __qtablewidgetitem2)
        __qtablewidgetitem3 = QTableWidgetItem()
        self.iter_table.setVerticalHeaderItem(2, __qtablewidgetitem3)
        __qtablewidgetitem4 = QTableWidgetItem()
        self.iter_table.setVerticalHeaderItem(3, __qtablewidgetitem4)
        __qtablewidgetitem5 = QTableWidgetItem()
        self.iter_table.setVerticalHeaderItem(4, __qtablewidgetitem5)
        __qtablewidgetitem6 = QTableWidgetItem()
        self.iter_table.setVerticalHeaderItem(5, __qtablewidgetitem6)
        __qtablewidgetitem7 = QTableWidgetItem()
        self.iter_table.setVerticalHeaderItem(6, __qtablewidgetitem7)
        __qtablewidgetitem8 = QTableWidgetItem()
        self.iter_table.setVerticalHeaderItem(7, __qtablewidgetitem8)
        __qtablewidgetitem9 = QTableWidgetItem()
        self.iter_table.setVerticalHeaderItem(8, __qtablewidgetitem9)
        __qtablewidgetitem10 = QTableWidgetItem()
        self.iter_table.setVerticalHeaderItem(9, __qtablewidgetitem10)
        __qtablewidgetitem11 = QTableWidgetItem()
        self.iter_table.setVerticalHeaderItem(10, __qtablewidgetitem11)
        __qtablewidgetitem12 = QTableWidgetItem()
        self.iter_table.setVerticalHeaderItem(11, __qtablewidgetitem12)
        __qtablewidgetitem13 = QTableWidgetItem()
        self.iter_table.setVerticalHeaderItem(12, __qtablewidgetitem13)
        __qtablewidgetitem14 = QTableWidgetItem()
        self.iter_table.setVerticalHeaderItem(13, __qtablewidgetitem14)
        __qtablewidgetitem15 = QTableWidgetItem()
        self.iter_table.setVerticalHeaderItem(14, __qtablewidgetitem15)
        __qtablewidgetitem16 = QTableWidgetItem()
        self.iter_table.setVerticalHeaderItem(15, __qtablewidgetitem16)
        __qtablewidgetitem17 = QTableWidgetItem()
        self.iter_table.setVerticalHeaderItem(16, __qtablewidgetitem17)
        self.iter_table.setObjectName(u"iter_table")
        self.iter_table.setGeometry(QRect(20, 260, 531, 541))
        self.iter_table.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.iter_table.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.iter_table.verticalHeader().setStretchLastSection(True)
        self.plan_status_label = QLabel(Dialog)
        self.plan_status_label.setObjectName(u"plan_status_label")
        self.plan_status_label.setGeometry(QRect(390, 220, 171, 17))
        self.getPoseBtn = QPushButton(Dialog)
        self.getPoseBtn.setObjectName(u"getPoseBtn")
        self.getPoseBtn.setGeometry(QRect(390, 40, 161, 31))

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
        self.IKEnableBtn.setText("")
        self.label_3.setText(QCoreApplication.translate("Dialog", u"IK mode\n"
"Enabled", None))
        self.planBtn.setText(QCoreApplication.translate("Dialog", u"PLAN", None))
        self.ExecBtn.setText(QCoreApplication.translate("Dialog", u"EXECUTE", None))
        ___qtablewidgetitem = self.iter_table.horizontalHeaderItem(0)
        ___qtablewidgetitem.setText(QCoreApplication.translate("Dialog", u"Position", None));
        ___qtablewidgetitem1 = self.iter_table.verticalHeaderItem(0)
        ___qtablewidgetitem1.setText(QCoreApplication.translate("Dialog", u"L1", None));
        ___qtablewidgetitem2 = self.iter_table.verticalHeaderItem(1)
        ___qtablewidgetitem2.setText(QCoreApplication.translate("Dialog", u"L2", None));
        ___qtablewidgetitem3 = self.iter_table.verticalHeaderItem(2)
        ___qtablewidgetitem3.setText(QCoreApplication.translate("Dialog", u"L3", None));
        ___qtablewidgetitem4 = self.iter_table.verticalHeaderItem(3)
        ___qtablewidgetitem4.setText(QCoreApplication.translate("Dialog", u"L4", None));
        ___qtablewidgetitem5 = self.iter_table.verticalHeaderItem(4)
        ___qtablewidgetitem5.setText(QCoreApplication.translate("Dialog", u"L5", None));
        ___qtablewidgetitem6 = self.iter_table.verticalHeaderItem(5)
        ___qtablewidgetitem6.setText(QCoreApplication.translate("Dialog", u"L6", None));
        ___qtablewidgetitem7 = self.iter_table.verticalHeaderItem(6)
        ___qtablewidgetitem7.setText(QCoreApplication.translate("Dialog", u"L7", None));
        ___qtablewidgetitem8 = self.iter_table.verticalHeaderItem(7)
        ___qtablewidgetitem8.setText(QCoreApplication.translate("Dialog", u"L8", None));
        ___qtablewidgetitem9 = self.iter_table.verticalHeaderItem(8)
        ___qtablewidgetitem9.setText(QCoreApplication.translate("Dialog", u"R1", None));
        ___qtablewidgetitem10 = self.iter_table.verticalHeaderItem(9)
        ___qtablewidgetitem10.setText(QCoreApplication.translate("Dialog", u"R2", None));
        ___qtablewidgetitem11 = self.iter_table.verticalHeaderItem(10)
        ___qtablewidgetitem11.setText(QCoreApplication.translate("Dialog", u"R3", None));
        ___qtablewidgetitem12 = self.iter_table.verticalHeaderItem(11)
        ___qtablewidgetitem12.setText(QCoreApplication.translate("Dialog", u"R4", None));
        ___qtablewidgetitem13 = self.iter_table.verticalHeaderItem(12)
        ___qtablewidgetitem13.setText(QCoreApplication.translate("Dialog", u"R5", None));
        ___qtablewidgetitem14 = self.iter_table.verticalHeaderItem(13)
        ___qtablewidgetitem14.setText(QCoreApplication.translate("Dialog", u"R6", None));
        ___qtablewidgetitem15 = self.iter_table.verticalHeaderItem(14)
        ___qtablewidgetitem15.setText(QCoreApplication.translate("Dialog", u"R7", None));
        ___qtablewidgetitem16 = self.iter_table.verticalHeaderItem(15)
        ___qtablewidgetitem16.setText(QCoreApplication.translate("Dialog", u"R8", None));
        ___qtablewidgetitem17 = self.iter_table.verticalHeaderItem(16)
        ___qtablewidgetitem17.setText(QCoreApplication.translate("Dialog", u"time", None));
        self.plan_status_label.setText(QCoreApplication.translate("Dialog", u"status: NOT PLANNED", None))
        self.getPoseBtn.setText(QCoreApplication.translate("Dialog", u"Get Current Pose", None))
    # retranslateUi

