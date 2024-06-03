# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'movegroup.ui'
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
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QDialog,
    QHeaderView, QLabel, QPushButton, QSizePolicy,
    QTableWidget, QTableWidgetItem, QWidget)
import buttons_rc

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        if not Dialog.objectName():
            Dialog.setObjectName(u"Dialog")
        Dialog.resize(754, 599)
        self.ikEnableBtn = QPushButton(Dialog)
        self.ikEnableBtn.setObjectName(u"ikEnableBtn")
        self.ikEnableBtn.setEnabled(True)
        self.ikEnableBtn.setGeometry(QRect(680, 30, 41, 41))
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.ikEnableBtn.sizePolicy().hasHeightForWidth())
        self.ikEnableBtn.setSizePolicy(sizePolicy)
        self.ikEnableBtn.setMinimumSize(QSize(41, 21))
        self.ikEnableBtn.setMaximumSize(QSize(16777215, 16777215))
        self.ikEnableBtn.setBaseSize(QSize(41, 21))
        self.ikEnableBtn.setStyleSheet(u"QPushButton{\n"
"	background-color: none;\n"
"	border-radius: 3%;\n"
"}")
        icon = QIcon()
        icon.addFile(u":/toggle/off", QSize(), QIcon.Normal, QIcon.Off)
        icon.addFile(u":/toggle/on", QSize(), QIcon.Normal, QIcon.On)
        self.ikEnableBtn.setIcon(icon)
        self.ikEnableBtn.setIconSize(QSize(41, 40))
        self.ikEnableBtn.setCheckable(True)
        self.ikEnableBtn.setChecked(False)
        self.ikEnableBtn.setFlat(True)
        self.ikLabel = QLabel(Dialog)
        self.ikLabel.setObjectName(u"ikLabel")
        self.ikLabel.setGeometry(QRect(570, 30, 91, 41))
        self.execBtn = QPushButton(Dialog)
        self.execBtn.setObjectName(u"execBtn")
        self.execBtn.setEnabled(False)
        self.execBtn.setGeometry(QRect(570, 140, 161, 50))
        self.iterTable = QTableWidget(Dialog)
        if (self.iterTable.columnCount() < 1):
            self.iterTable.setColumnCount(1)
        __qtablewidgetitem = QTableWidgetItem()
        self.iterTable.setHorizontalHeaderItem(0, __qtablewidgetitem)
        if (self.iterTable.rowCount() < 17):
            self.iterTable.setRowCount(17)
        __qtablewidgetitem1 = QTableWidgetItem()
        self.iterTable.setVerticalHeaderItem(0, __qtablewidgetitem1)
        __qtablewidgetitem2 = QTableWidgetItem()
        self.iterTable.setVerticalHeaderItem(1, __qtablewidgetitem2)
        __qtablewidgetitem3 = QTableWidgetItem()
        self.iterTable.setVerticalHeaderItem(2, __qtablewidgetitem3)
        __qtablewidgetitem4 = QTableWidgetItem()
        self.iterTable.setVerticalHeaderItem(3, __qtablewidgetitem4)
        __qtablewidgetitem5 = QTableWidgetItem()
        self.iterTable.setVerticalHeaderItem(4, __qtablewidgetitem5)
        __qtablewidgetitem6 = QTableWidgetItem()
        self.iterTable.setVerticalHeaderItem(5, __qtablewidgetitem6)
        __qtablewidgetitem7 = QTableWidgetItem()
        self.iterTable.setVerticalHeaderItem(6, __qtablewidgetitem7)
        __qtablewidgetitem8 = QTableWidgetItem()
        self.iterTable.setVerticalHeaderItem(7, __qtablewidgetitem8)
        __qtablewidgetitem9 = QTableWidgetItem()
        self.iterTable.setVerticalHeaderItem(8, __qtablewidgetitem9)
        __qtablewidgetitem10 = QTableWidgetItem()
        self.iterTable.setVerticalHeaderItem(9, __qtablewidgetitem10)
        __qtablewidgetitem11 = QTableWidgetItem()
        self.iterTable.setVerticalHeaderItem(10, __qtablewidgetitem11)
        __qtablewidgetitem12 = QTableWidgetItem()
        self.iterTable.setVerticalHeaderItem(11, __qtablewidgetitem12)
        __qtablewidgetitem13 = QTableWidgetItem()
        self.iterTable.setVerticalHeaderItem(12, __qtablewidgetitem13)
        __qtablewidgetitem14 = QTableWidgetItem()
        self.iterTable.setVerticalHeaderItem(13, __qtablewidgetitem14)
        __qtablewidgetitem15 = QTableWidgetItem()
        self.iterTable.setVerticalHeaderItem(14, __qtablewidgetitem15)
        __qtablewidgetitem16 = QTableWidgetItem()
        self.iterTable.setVerticalHeaderItem(15, __qtablewidgetitem16)
        __qtablewidgetitem17 = QTableWidgetItem()
        self.iterTable.setVerticalHeaderItem(16, __qtablewidgetitem17)
        self.iterTable.setObjectName(u"iterTable")
        self.iterTable.setGeometry(QRect(20, 30, 531, 541))
        self.iterTable.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.iterTable.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.iterTable.verticalHeader().setStretchLastSection(True)
        self.planStatusLabel = QLabel(Dialog)
        self.planStatusLabel.setObjectName(u"planStatusLabel")
        self.planStatusLabel.setGeometry(QRect(570, 200, 171, 17))
        self.loopCheckBox = QCheckBox(Dialog)
        self.loopCheckBox.setObjectName(u"loopCheckBox")
        self.loopCheckBox.setEnabled(False)
        self.loopCheckBox.setGeometry(QRect(570, 240, 151, 23))
        self.saveTrajBtn = QPushButton(Dialog)
        self.saveTrajBtn.setObjectName(u"saveTrajBtn")
        self.saveTrajBtn.setEnabled(True)
        self.saveTrajBtn.setGeometry(QRect(570, 520, 161, 50))
        self.planningPipelineCombo = QComboBox(Dialog)
        self.planningPipelineCombo.setObjectName(u"planningPipelineCombo")
        self.planningPipelineCombo.setGeometry(QRect(570, 290, 161, 25))
        self.plannerIDCombo = QComboBox(Dialog)
        self.plannerIDCombo.setObjectName(u"plannerIDCombo")
        self.plannerIDCombo.setGeometry(QRect(570, 350, 161, 25))
        self.planStatusLabel_2 = QLabel(Dialog)
        self.planStatusLabel_2.setObjectName(u"planStatusLabel_2")
        self.planStatusLabel_2.setGeometry(QRect(570, 270, 171, 17))
        self.planStatusLabel_3 = QLabel(Dialog)
        self.planStatusLabel_3.setObjectName(u"planStatusLabel_3")
        self.planStatusLabel_3.setGeometry(QRect(570, 330, 171, 17))
        self.planBtn = QPushButton(Dialog)
        self.planBtn.setObjectName(u"planBtn")
        self.planBtn.setEnabled(True)
        self.planBtn.setGeometry(QRect(570, 80, 161, 50))

        self.retranslateUi(Dialog)

        QMetaObject.connectSlotsByName(Dialog)
    # setupUi

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", u"movegroup", None))
        self.ikEnableBtn.setText("")
        self.ikLabel.setText(QCoreApplication.translate("Dialog", u"IK mode\n"
"Enabled", None))
        self.execBtn.setText(QCoreApplication.translate("Dialog", u"EXECUTE", None))
        ___qtablewidgetitem = self.iterTable.horizontalHeaderItem(0)
        ___qtablewidgetitem.setText(QCoreApplication.translate("Dialog", u"Position", None));
        ___qtablewidgetitem1 = self.iterTable.verticalHeaderItem(0)
        ___qtablewidgetitem1.setText(QCoreApplication.translate("Dialog", u"L1", None));
        ___qtablewidgetitem2 = self.iterTable.verticalHeaderItem(1)
        ___qtablewidgetitem2.setText(QCoreApplication.translate("Dialog", u"L2", None));
        ___qtablewidgetitem3 = self.iterTable.verticalHeaderItem(2)
        ___qtablewidgetitem3.setText(QCoreApplication.translate("Dialog", u"L3", None));
        ___qtablewidgetitem4 = self.iterTable.verticalHeaderItem(3)
        ___qtablewidgetitem4.setText(QCoreApplication.translate("Dialog", u"L4", None));
        ___qtablewidgetitem5 = self.iterTable.verticalHeaderItem(4)
        ___qtablewidgetitem5.setText(QCoreApplication.translate("Dialog", u"L5", None));
        ___qtablewidgetitem6 = self.iterTable.verticalHeaderItem(5)
        ___qtablewidgetitem6.setText(QCoreApplication.translate("Dialog", u"L6", None));
        ___qtablewidgetitem7 = self.iterTable.verticalHeaderItem(6)
        ___qtablewidgetitem7.setText(QCoreApplication.translate("Dialog", u"L7", None));
        ___qtablewidgetitem8 = self.iterTable.verticalHeaderItem(7)
        ___qtablewidgetitem8.setText(QCoreApplication.translate("Dialog", u"L8", None));
        ___qtablewidgetitem9 = self.iterTable.verticalHeaderItem(8)
        ___qtablewidgetitem9.setText(QCoreApplication.translate("Dialog", u"R1", None));
        ___qtablewidgetitem10 = self.iterTable.verticalHeaderItem(9)
        ___qtablewidgetitem10.setText(QCoreApplication.translate("Dialog", u"R2", None));
        ___qtablewidgetitem11 = self.iterTable.verticalHeaderItem(10)
        ___qtablewidgetitem11.setText(QCoreApplication.translate("Dialog", u"R3", None));
        ___qtablewidgetitem12 = self.iterTable.verticalHeaderItem(11)
        ___qtablewidgetitem12.setText(QCoreApplication.translate("Dialog", u"R4", None));
        ___qtablewidgetitem13 = self.iterTable.verticalHeaderItem(12)
        ___qtablewidgetitem13.setText(QCoreApplication.translate("Dialog", u"R5", None));
        ___qtablewidgetitem14 = self.iterTable.verticalHeaderItem(13)
        ___qtablewidgetitem14.setText(QCoreApplication.translate("Dialog", u"R6", None));
        ___qtablewidgetitem15 = self.iterTable.verticalHeaderItem(14)
        ___qtablewidgetitem15.setText(QCoreApplication.translate("Dialog", u"R7", None));
        ___qtablewidgetitem16 = self.iterTable.verticalHeaderItem(15)
        ___qtablewidgetitem16.setText(QCoreApplication.translate("Dialog", u"R8", None));
        ___qtablewidgetitem17 = self.iterTable.verticalHeaderItem(16)
        ___qtablewidgetitem17.setText(QCoreApplication.translate("Dialog", u"time", None));
        self.planStatusLabel.setText(QCoreApplication.translate("Dialog", u"status: NOT PLANNED", None))
        self.loopCheckBox.setText(QCoreApplication.translate("Dialog", u"Loop Trajectory", None))
        self.saveTrajBtn.setText(QCoreApplication.translate("Dialog", u"Save Trajectory", None))
        self.planStatusLabel_2.setText(QCoreApplication.translate("Dialog", u"Planning Pipeline", None))
        self.planStatusLabel_3.setText(QCoreApplication.translate("Dialog", u"Planner ID", None))
        self.planBtn.setText(QCoreApplication.translate("Dialog", u"PLAN", None))
    # retranslateUi

