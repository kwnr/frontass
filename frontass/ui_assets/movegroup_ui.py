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
from PySide6.QtWidgets import (QApplication, QComboBox, QDialog, QDoubleSpinBox,
    QFormLayout, QHeaderView, QLabel, QPushButton,
    QSizePolicy, QTableWidget, QTableWidgetItem, QWidget)
import buttons_rc

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        if not Dialog.objectName():
            Dialog.setObjectName(u"Dialog")
        Dialog.resize(754, 897)
        self.ikEnableBtn = QPushButton(Dialog)
        self.ikEnableBtn.setObjectName(u"ikEnableBtn")
        self.ikEnableBtn.setEnabled(True)
        self.ikEnableBtn.setGeometry(QRect(150, 20, 41, 41))
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
        self.ikLabel.setGeometry(QRect(20, 20, 131, 41))
        self.execBtn = QPushButton(Dialog)
        self.execBtn.setObjectName(u"execBtn")
        self.execBtn.setEnabled(False)
        self.execBtn.setGeometry(QRect(570, 170, 161, 51))
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
        self.iterTable.setGeometry(QRect(20, 330, 711, 541))
        self.iterTable.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.iterTable.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.iterTable.verticalHeader().setStretchLastSection(True)
        self.planStatusLabel = QLabel(Dialog)
        self.planStatusLabel.setObjectName(u"planStatusLabel")
        self.planStatusLabel.setGeometry(QRect(570, 230, 171, 17))
        self.saveTrajBtn = QPushButton(Dialog)
        self.saveTrajBtn.setObjectName(u"saveTrajBtn")
        self.saveTrajBtn.setEnabled(True)
        self.saveTrajBtn.setGeometry(QRect(570, 260, 161, 50))
        self.planningPipelineCombo = QComboBox(Dialog)
        self.planningPipelineCombo.setObjectName(u"planningPipelineCombo")
        self.planningPipelineCombo.setGeometry(QRect(380, 230, 161, 25))
        self.plannerIDCombo = QComboBox(Dialog)
        self.plannerIDCombo.setObjectName(u"plannerIDCombo")
        self.plannerIDCombo.setGeometry(QRect(380, 290, 161, 25))
        self.planStatusLabel_2 = QLabel(Dialog)
        self.planStatusLabel_2.setObjectName(u"planStatusLabel_2")
        self.planStatusLabel_2.setGeometry(QRect(380, 210, 171, 17))
        self.planStatusLabel_3 = QLabel(Dialog)
        self.planStatusLabel_3.setObjectName(u"planStatusLabel_3")
        self.planStatusLabel_3.setGeometry(QRect(380, 270, 171, 17))
        self.planBtn = QPushButton(Dialog)
        self.planBtn.setObjectName(u"planBtn")
        self.planBtn.setEnabled(True)
        self.planBtn.setGeometry(QRect(570, 100, 161, 51))
        self.formLayoutWidget_2 = QWidget(Dialog)
        self.formLayoutWidget_2.setObjectName(u"formLayoutWidget_2")
        self.formLayoutWidget_2.setGeometry(QRect(200, 100, 160, 231))
        self.targetFormLayout = QFormLayout(self.formLayoutWidget_2)
        self.targetFormLayout.setObjectName(u"targetFormLayout")
        self.targetFormLayout.setVerticalSpacing(12)
        self.targetFormLayout.setContentsMargins(0, 0, 0, 0)
        self.targetXLabel = QLabel(self.formLayoutWidget_2)
        self.targetXLabel.setObjectName(u"targetXLabel")

        self.targetFormLayout.setWidget(0, QFormLayout.LabelRole, self.targetXLabel)

        self.targetXDoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget_2)
        self.targetXDoubleSpinBox.setObjectName(u"targetXDoubleSpinBox")
        self.targetXDoubleSpinBox.setMinimum(-999.000000000000000)
        self.targetXDoubleSpinBox.setMaximum(999.000000000000000)

        self.targetFormLayout.setWidget(0, QFormLayout.FieldRole, self.targetXDoubleSpinBox)

        self.targetYLabel = QLabel(self.formLayoutWidget_2)
        self.targetYLabel.setObjectName(u"targetYLabel")

        self.targetFormLayout.setWidget(1, QFormLayout.LabelRole, self.targetYLabel)

        self.targetYDoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget_2)
        self.targetYDoubleSpinBox.setObjectName(u"targetYDoubleSpinBox")
        self.targetYDoubleSpinBox.setMinimum(-999.000000000000000)
        self.targetYDoubleSpinBox.setMaximum(999.000000000000000)

        self.targetFormLayout.setWidget(1, QFormLayout.FieldRole, self.targetYDoubleSpinBox)

        self.targetZLabel = QLabel(self.formLayoutWidget_2)
        self.targetZLabel.setObjectName(u"targetZLabel")

        self.targetFormLayout.setWidget(2, QFormLayout.LabelRole, self.targetZLabel)

        self.targetZDoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget_2)
        self.targetZDoubleSpinBox.setObjectName(u"targetZDoubleSpinBox")
        self.targetZDoubleSpinBox.setMinimum(-999.000000000000000)
        self.targetZDoubleSpinBox.setMaximum(999.000000000000000)

        self.targetFormLayout.setWidget(2, QFormLayout.FieldRole, self.targetZDoubleSpinBox)

        self.targetRollLabel = QLabel(self.formLayoutWidget_2)
        self.targetRollLabel.setObjectName(u"targetRollLabel")

        self.targetFormLayout.setWidget(3, QFormLayout.LabelRole, self.targetRollLabel)

        self.targetRollDoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget_2)
        self.targetRollDoubleSpinBox.setObjectName(u"targetRollDoubleSpinBox")
        self.targetRollDoubleSpinBox.setMinimum(-999.000000000000000)
        self.targetRollDoubleSpinBox.setMaximum(999.000000000000000)

        self.targetFormLayout.setWidget(3, QFormLayout.FieldRole, self.targetRollDoubleSpinBox)

        self.targetPitchLabel = QLabel(self.formLayoutWidget_2)
        self.targetPitchLabel.setObjectName(u"targetPitchLabel")

        self.targetFormLayout.setWidget(4, QFormLayout.LabelRole, self.targetPitchLabel)

        self.targetPitchDoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget_2)
        self.targetPitchDoubleSpinBox.setObjectName(u"targetPitchDoubleSpinBox")
        self.targetPitchDoubleSpinBox.setMinimum(-999.000000000000000)
        self.targetPitchDoubleSpinBox.setMaximum(999.000000000000000)

        self.targetFormLayout.setWidget(4, QFormLayout.FieldRole, self.targetPitchDoubleSpinBox)

        self.targetYawLabel = QLabel(self.formLayoutWidget_2)
        self.targetYawLabel.setObjectName(u"targetYawLabel")

        self.targetFormLayout.setWidget(5, QFormLayout.LabelRole, self.targetYawLabel)

        self.targetYawDoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget_2)
        self.targetYawDoubleSpinBox.setObjectName(u"targetYawDoubleSpinBox")
        self.targetYawDoubleSpinBox.setMinimum(-999.000000000000000)
        self.targetYawDoubleSpinBox.setMaximum(999.000000000000000)

        self.targetFormLayout.setWidget(5, QFormLayout.FieldRole, self.targetYawDoubleSpinBox)

        self.label_2 = QLabel(Dialog)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(200, 70, 161, 21))
        self.formLayoutWidget = QWidget(Dialog)
        self.formLayoutWidget.setObjectName(u"formLayoutWidget")
        self.formLayoutWidget.setGeometry(QRect(20, 100, 160, 231))
        self.formLayout = QFormLayout(self.formLayoutWidget)
        self.formLayout.setObjectName(u"formLayout")
        self.formLayout.setVerticalSpacing(12)
        self.formLayout.setContentsMargins(0, 0, 0, 0)
        self.xLabel = QLabel(self.formLayoutWidget)
        self.xLabel.setObjectName(u"xLabel")

        self.formLayout.setWidget(0, QFormLayout.LabelRole, self.xLabel)

        self.xDoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget)
        self.xDoubleSpinBox.setObjectName(u"xDoubleSpinBox")
        self.xDoubleSpinBox.setEnabled(False)
        self.xDoubleSpinBox.setMinimum(-999.000000000000000)
        self.xDoubleSpinBox.setMaximum(999.000000000000000)

        self.formLayout.setWidget(0, QFormLayout.FieldRole, self.xDoubleSpinBox)

        self.yLabel = QLabel(self.formLayoutWidget)
        self.yLabel.setObjectName(u"yLabel")

        self.formLayout.setWidget(1, QFormLayout.LabelRole, self.yLabel)

        self.yDoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget)
        self.yDoubleSpinBox.setObjectName(u"yDoubleSpinBox")
        self.yDoubleSpinBox.setEnabled(False)
        self.yDoubleSpinBox.setMinimum(-999.000000000000000)
        self.yDoubleSpinBox.setMaximum(999.000000000000000)

        self.formLayout.setWidget(1, QFormLayout.FieldRole, self.yDoubleSpinBox)

        self.zLabel = QLabel(self.formLayoutWidget)
        self.zLabel.setObjectName(u"zLabel")

        self.formLayout.setWidget(2, QFormLayout.LabelRole, self.zLabel)

        self.zDoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget)
        self.zDoubleSpinBox.setObjectName(u"zDoubleSpinBox")
        self.zDoubleSpinBox.setEnabled(False)
        self.zDoubleSpinBox.setMinimum(-999.000000000000000)
        self.zDoubleSpinBox.setMaximum(999.000000000000000)

        self.formLayout.setWidget(2, QFormLayout.FieldRole, self.zDoubleSpinBox)

        self.rollLabel = QLabel(self.formLayoutWidget)
        self.rollLabel.setObjectName(u"rollLabel")

        self.formLayout.setWidget(3, QFormLayout.LabelRole, self.rollLabel)

        self.rollDoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget)
        self.rollDoubleSpinBox.setObjectName(u"rollDoubleSpinBox")
        self.rollDoubleSpinBox.setEnabled(False)
        self.rollDoubleSpinBox.setMinimum(-999.000000000000000)
        self.rollDoubleSpinBox.setMaximum(999.000000000000000)

        self.formLayout.setWidget(3, QFormLayout.FieldRole, self.rollDoubleSpinBox)

        self.pitchLabel = QLabel(self.formLayoutWidget)
        self.pitchLabel.setObjectName(u"pitchLabel")

        self.formLayout.setWidget(4, QFormLayout.LabelRole, self.pitchLabel)

        self.pitchDoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget)
        self.pitchDoubleSpinBox.setObjectName(u"pitchDoubleSpinBox")
        self.pitchDoubleSpinBox.setEnabled(False)
        self.pitchDoubleSpinBox.setMinimum(-999.000000000000000)
        self.pitchDoubleSpinBox.setMaximum(999.000000000000000)

        self.formLayout.setWidget(4, QFormLayout.FieldRole, self.pitchDoubleSpinBox)

        self.yawLabel = QLabel(self.formLayoutWidget)
        self.yawLabel.setObjectName(u"yawLabel")

        self.formLayout.setWidget(5, QFormLayout.LabelRole, self.yawLabel)

        self.yawDoubleSpinBox = QDoubleSpinBox(self.formLayoutWidget)
        self.yawDoubleSpinBox.setObjectName(u"yawDoubleSpinBox")
        self.yawDoubleSpinBox.setEnabled(False)
        self.yawDoubleSpinBox.setMinimum(-999.000000000000000)
        self.yawDoubleSpinBox.setMaximum(999.000000000000000)

        self.formLayout.setWidget(5, QFormLayout.FieldRole, self.yawDoubleSpinBox)

        self.label = QLabel(Dialog)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(20, 70, 161, 21))
        self.getCurrentPoseBtn = QPushButton(Dialog)
        self.getCurrentPoseBtn.setObjectName(u"getCurrentPoseBtn")
        self.getCurrentPoseBtn.setGeometry(QRect(380, 100, 161, 41))
        self.velScaleSpinBox = QDoubleSpinBox(Dialog)
        self.velScaleSpinBox.setObjectName(u"velScaleSpinBox")
        self.velScaleSpinBox.setGeometry(QRect(380, 170, 65, 26))
        self.velScaleSpinBox.setMaximum(1.000000000000000)
        self.velScaleSpinBox.setSingleStep(0.100000000000000)
        self.velScaleSpinBox.setValue(0.100000000000000)
        self.accScaleSpinBox = QDoubleSpinBox(Dialog)
        self.accScaleSpinBox.setObjectName(u"accScaleSpinBox")
        self.accScaleSpinBox.setGeometry(QRect(480, 170, 65, 26))
        self.accScaleSpinBox.setMaximum(1.000000000000000)
        self.accScaleSpinBox.setSingleStep(0.100000000000000)
        self.accScaleSpinBox.setValue(0.100000000000000)
        self.label_3 = QLabel(Dialog)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(380, 150, 71, 17))
        self.label_4 = QLabel(Dialog)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(480, 150, 71, 17))

        self.retranslateUi(Dialog)

        QMetaObject.connectSlotsByName(Dialog)
    # setupUi

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", u"movegroup", None))
        self.ikEnableBtn.setText("")
        self.ikLabel.setText(QCoreApplication.translate("Dialog", u"IK mode Enabled", None))
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
        self.saveTrajBtn.setText(QCoreApplication.translate("Dialog", u"Save Trajectory", None))
        self.planStatusLabel_2.setText(QCoreApplication.translate("Dialog", u"Planning Pipeline", None))
        self.planStatusLabel_3.setText(QCoreApplication.translate("Dialog", u"Planner ID", None))
        self.planBtn.setText(QCoreApplication.translate("Dialog", u"PLAN", None))
        self.targetXLabel.setText(QCoreApplication.translate("Dialog", u"X", None))
        self.targetYLabel.setText(QCoreApplication.translate("Dialog", u"Y", None))
        self.targetZLabel.setText(QCoreApplication.translate("Dialog", u"Z", None))
        self.targetRollLabel.setText(QCoreApplication.translate("Dialog", u"Roll", None))
        self.targetPitchLabel.setText(QCoreApplication.translate("Dialog", u"Pitch", None))
        self.targetYawLabel.setText(QCoreApplication.translate("Dialog", u"Yaw", None))
        self.label_2.setText(QCoreApplication.translate("Dialog", u"Target Pose", None))
        self.xLabel.setText(QCoreApplication.translate("Dialog", u"X", None))
        self.yLabel.setText(QCoreApplication.translate("Dialog", u"Y", None))
        self.zLabel.setText(QCoreApplication.translate("Dialog", u"Z", None))
        self.rollLabel.setText(QCoreApplication.translate("Dialog", u"Roll", None))
        self.pitchLabel.setText(QCoreApplication.translate("Dialog", u"Pitch", None))
        self.yawLabel.setText(QCoreApplication.translate("Dialog", u"Yaw", None))
        self.label.setText(QCoreApplication.translate("Dialog", u"Current Pose", None))
        self.getCurrentPoseBtn.setText(QCoreApplication.translate("Dialog", u"Get Current Pose", None))
        self.label_3.setText(QCoreApplication.translate("Dialog", u"Vel. Scale", None))
        self.label_4.setText(QCoreApplication.translate("Dialog", u"Acc. Scale", None))
    # retranslateUi

