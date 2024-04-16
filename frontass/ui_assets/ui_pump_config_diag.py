# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'pump_config_diag.ui'
##
## Created by: Qt User Interface Compiler version 6.3.0
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
from PySide6.QtWidgets import (QAbstractButton, QApplication, QDialog, QDialogButtonBox,
    QFormLayout, QLabel, QLineEdit, QSizePolicy,
    QWidget)

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        if not Dialog.objectName():
            Dialog.setObjectName(u"Dialog")
        Dialog.resize(389, 300)
        Dialog.setStyleSheet(u"QDialog{\n"
"	background-color: rgba(246,246,246,0.9);\n"
"}\n"
"QLineEdit{\n"
"	border-radius: 5%\n"
"}\n"
"QLineEdit:focus{\n"
"	border-radius: 5%;\n"
"	border-width: 3px;\n"
"	border-color: rgba(0, 122, 255, 0.5);\n"
"	border-style: solid;\n"
"}")
        self.buttonBox = QDialogButtonBox(Dialog)
        self.buttonBox.setObjectName(u"buttonBox")
        self.buttonBox.setGeometry(QRect(30, 240, 341, 32))
        self.buttonBox.setOrientation(Qt.Horizontal)
        self.buttonBox.setStandardButtons(QDialogButtonBox.Cancel|QDialogButtonBox.Ok)
        self.formLayoutWidget = QWidget(Dialog)
        self.formLayoutWidget.setObjectName(u"formLayoutWidget")
        self.formLayoutWidget.setGeometry(QRect(10, 20, 361, 212))
        self.formLayout = QFormLayout(self.formLayoutWidget)
        self.formLayout.setObjectName(u"formLayout")
        self.formLayout.setFormAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignTop)
        self.formLayout.setVerticalSpacing(30)
        self.formLayout.setContentsMargins(0, 6, 0, 0)
        self.manualTargetRPMLabel = QLabel(self.formLayoutWidget)
        self.manualTargetRPMLabel.setObjectName(u"manualTargetRPMLabel")

        self.formLayout.setWidget(0, QFormLayout.LabelRole, self.manualTargetRPMLabel)

        self.manualTargetRPMLineEdit = QLineEdit(self.formLayoutWidget)
        self.manualTargetRPMLineEdit.setObjectName(u"manualTargetRPMLineEdit")
        self.manualTargetRPMLineEdit.setMinimumSize(QSize(0, 25))

        self.formLayout.setWidget(0, QFormLayout.FieldRole, self.manualTargetRPMLineEdit)

        self.autoRangeMinLabel = QLabel(self.formLayoutWidget)
        self.autoRangeMinLabel.setObjectName(u"autoRangeMinLabel")

        self.formLayout.setWidget(1, QFormLayout.LabelRole, self.autoRangeMinLabel)

        self.autoRangeMinLineEdit = QLineEdit(self.formLayoutWidget)
        self.autoRangeMinLineEdit.setObjectName(u"autoRangeMinLineEdit")
        self.autoRangeMinLineEdit.setMinimumSize(QSize(0, 25))

        self.formLayout.setWidget(1, QFormLayout.FieldRole, self.autoRangeMinLineEdit)

        self.autoRangeMaxLabel = QLabel(self.formLayoutWidget)
        self.autoRangeMaxLabel.setObjectName(u"autoRangeMaxLabel")

        self.formLayout.setWidget(2, QFormLayout.LabelRole, self.autoRangeMaxLabel)

        self.autoRangeMaxLineEdit = QLineEdit(self.formLayoutWidget)
        self.autoRangeMaxLineEdit.setObjectName(u"autoRangeMaxLineEdit")
        self.autoRangeMaxLineEdit.setMinimumSize(QSize(0, 25))

        self.formLayout.setWidget(2, QFormLayout.FieldRole, self.autoRangeMaxLineEdit)

        self.autoMaxErrorLabel = QLabel(self.formLayoutWidget)
        self.autoMaxErrorLabel.setObjectName(u"autoMaxErrorLabel")

        self.formLayout.setWidget(3, QFormLayout.LabelRole, self.autoMaxErrorLabel)

        self.autoMaxErrorLineEdit = QLineEdit(self.formLayoutWidget)
        self.autoMaxErrorLineEdit.setObjectName(u"autoMaxErrorLineEdit")
        self.autoMaxErrorLineEdit.setMinimumSize(QSize(0, 25))

        self.formLayout.setWidget(3, QFormLayout.FieldRole, self.autoMaxErrorLineEdit)


        self.retranslateUi(Dialog)
        self.buttonBox.accepted.connect(Dialog.accept)
        self.buttonBox.rejected.connect(Dialog.reject)

        QMetaObject.connectSlotsByName(Dialog)
    # setupUi

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", u"Pump Config", None))
        self.manualTargetRPMLabel.setText(QCoreApplication.translate("Dialog", u"Manual Target RPM", None))
        self.autoRangeMinLabel.setText(QCoreApplication.translate("Dialog", u"Auto Range min", None))
        self.autoRangeMaxLabel.setText(QCoreApplication.translate("Dialog", u"Auto Range max", None))
        self.autoMaxErrorLabel.setText(QCoreApplication.translate("Dialog", u"Auto Max Error", None))
    # retranslateUi

