# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'pose_iterator_diag.ui'
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
from PySide6.QtWidgets import (QAbstractButton, QAbstractItemView, QApplication, QDialog,
    QDialogButtonBox, QHeaderView, QLabel, QLineEdit,
    QPushButton, QSizePolicy, QTableWidget, QTableWidgetItem,
    QToolButton, QWidget)
import buttons_rc

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        if not Dialog.objectName():
            Dialog.setObjectName(u"Dialog")
        Dialog.resize(748, 779)
        self.buttonBox = QDialogButtonBox(Dialog)
        self.buttonBox.setObjectName(u"buttonBox")
        self.buttonBox.setGeometry(QRect(380, 730, 341, 32))
        self.buttonBox.setOrientation(Qt.Orientation.Horizontal)
        self.buttonBox.setStandardButtons(QDialogButtonBox.StandardButton.Cancel|QDialogButtonBox.StandardButton.Ok)
        self.iterTable = QTableWidget(Dialog)
        if (self.iterTable.columnCount() < 1):
            self.iterTable.setColumnCount(1)
        __qtablewidgetitem = QTableWidgetItem()
        self.iterTable.setHorizontalHeaderItem(0, __qtablewidgetitem)
        if (self.iterTable.rowCount() < 18):
            self.iterTable.setRowCount(18)
        __qtablewidgetitem1 = QTableWidgetItem()
        __qtablewidgetitem1.setTextAlignment(Qt.AlignCenter);
        self.iterTable.setVerticalHeaderItem(0, __qtablewidgetitem1)
        __qtablewidgetitem2 = QTableWidgetItem()
        __qtablewidgetitem2.setTextAlignment(Qt.AlignCenter);
        self.iterTable.setVerticalHeaderItem(1, __qtablewidgetitem2)
        __qtablewidgetitem3 = QTableWidgetItem()
        __qtablewidgetitem3.setTextAlignment(Qt.AlignCenter);
        self.iterTable.setVerticalHeaderItem(2, __qtablewidgetitem3)
        __qtablewidgetitem4 = QTableWidgetItem()
        __qtablewidgetitem4.setTextAlignment(Qt.AlignCenter);
        self.iterTable.setVerticalHeaderItem(3, __qtablewidgetitem4)
        __qtablewidgetitem5 = QTableWidgetItem()
        __qtablewidgetitem5.setTextAlignment(Qt.AlignCenter);
        self.iterTable.setVerticalHeaderItem(4, __qtablewidgetitem5)
        __qtablewidgetitem6 = QTableWidgetItem()
        __qtablewidgetitem6.setTextAlignment(Qt.AlignCenter);
        self.iterTable.setVerticalHeaderItem(5, __qtablewidgetitem6)
        __qtablewidgetitem7 = QTableWidgetItem()
        __qtablewidgetitem7.setTextAlignment(Qt.AlignCenter);
        self.iterTable.setVerticalHeaderItem(6, __qtablewidgetitem7)
        __qtablewidgetitem8 = QTableWidgetItem()
        __qtablewidgetitem8.setTextAlignment(Qt.AlignCenter);
        self.iterTable.setVerticalHeaderItem(7, __qtablewidgetitem8)
        __qtablewidgetitem9 = QTableWidgetItem()
        __qtablewidgetitem9.setTextAlignment(Qt.AlignCenter);
        self.iterTable.setVerticalHeaderItem(8, __qtablewidgetitem9)
        __qtablewidgetitem10 = QTableWidgetItem()
        __qtablewidgetitem10.setTextAlignment(Qt.AlignCenter);
        self.iterTable.setVerticalHeaderItem(9, __qtablewidgetitem10)
        __qtablewidgetitem11 = QTableWidgetItem()
        __qtablewidgetitem11.setTextAlignment(Qt.AlignCenter);
        self.iterTable.setVerticalHeaderItem(10, __qtablewidgetitem11)
        __qtablewidgetitem12 = QTableWidgetItem()
        __qtablewidgetitem12.setTextAlignment(Qt.AlignCenter);
        self.iterTable.setVerticalHeaderItem(11, __qtablewidgetitem12)
        __qtablewidgetitem13 = QTableWidgetItem()
        __qtablewidgetitem13.setTextAlignment(Qt.AlignCenter);
        self.iterTable.setVerticalHeaderItem(12, __qtablewidgetitem13)
        __qtablewidgetitem14 = QTableWidgetItem()
        __qtablewidgetitem14.setTextAlignment(Qt.AlignCenter);
        self.iterTable.setVerticalHeaderItem(13, __qtablewidgetitem14)
        __qtablewidgetitem15 = QTableWidgetItem()
        __qtablewidgetitem15.setTextAlignment(Qt.AlignCenter);
        self.iterTable.setVerticalHeaderItem(14, __qtablewidgetitem15)
        __qtablewidgetitem16 = QTableWidgetItem()
        __qtablewidgetitem16.setTextAlignment(Qt.AlignCenter);
        self.iterTable.setVerticalHeaderItem(15, __qtablewidgetitem16)
        __qtablewidgetitem17 = QTableWidgetItem()
        __qtablewidgetitem17.setTextAlignment(Qt.AlignCenter);
        self.iterTable.setVerticalHeaderItem(16, __qtablewidgetitem17)
        font = QFont()
        font.setItalic(True)
        __qtablewidgetitem18 = QTableWidgetItem()
        __qtablewidgetitem18.setTextAlignment(Qt.AlignCenter);
        __qtablewidgetitem18.setFont(font);
        self.iterTable.setVerticalHeaderItem(17, __qtablewidgetitem18)
        self.iterTable.setObjectName(u"iterTable")
        self.iterTable.setGeometry(QRect(10, 120, 711, 591))
        self.iterTable.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.iterTable.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.iterTable.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.iterTable.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.iterTable.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self.iterTable.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectColumns)
        self.iterTable.verticalHeader().setStretchLastSection(True)
        self.label = QLabel(Dialog)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(10, 60, 67, 30))
        self.posePathLineEdit = QLineEdit(Dialog)
        self.posePathLineEdit.setObjectName(u"posePathLineEdit")
        self.posePathLineEdit.setGeometry(QRect(90, 60, 391, 30))
        self.posePathBtn = QToolButton(Dialog)
        self.posePathBtn.setObjectName(u"posePathBtn")
        self.posePathBtn.setGeometry(QRect(490, 60, 31, 30))
        self.enabledBtn = QPushButton(Dialog)
        self.enabledBtn.setObjectName(u"enabledBtn")
        self.enabledBtn.setEnabled(True)
        self.enabledBtn.setGeometry(QRect(80, 20, 41, 30))
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.enabledBtn.sizePolicy().hasHeightForWidth())
        self.enabledBtn.setSizePolicy(sizePolicy)
        self.enabledBtn.setMinimumSize(QSize(41, 21))
        self.enabledBtn.setMaximumSize(QSize(16777215, 16777215))
        self.enabledBtn.setBaseSize(QSize(41, 21))
        self.enabledBtn.setStyleSheet(u"QPushButton{\n"
"	background-color: none;\n"
"	border-radius: 5%;\n"
"}")
        icon = QIcon()
        icon.addFile(u":/toggle/off", QSize(), QIcon.Normal, QIcon.Off)
        icon.addFile(u":/toggle/on", QSize(), QIcon.Normal, QIcon.On)
        self.enabledBtn.setIcon(icon)
        self.enabledBtn.setIconSize(QSize(41, 40))
        self.enabledBtn.setCheckable(True)
        self.enabledBtn.setChecked(False)
        self.label_2 = QLabel(Dialog)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(10, 20, 61, 30))
        self.sendBtn = QPushButton(Dialog)
        self.sendBtn.setObjectName(u"sendBtn")
        self.sendBtn.setGeometry(QRect(550, 20, 171, 31))
        self.convCritLineEdit = QLineEdit(Dialog)
        self.convCritLineEdit.setObjectName(u"convCritLineEdit")
        self.convCritLineEdit.setGeometry(QRect(410, 20, 111, 31))
        self.label_4 = QLabel(Dialog)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(330, 20, 81, 30))
        self.execBtn = QPushButton(Dialog)
        self.execBtn.setObjectName(u"execBtn")
        self.execBtn.setGeometry(QRect(550, 60, 171, 31))
        self.statusLabel = QLabel(Dialog)
        self.statusLabel.setObjectName(u"statusLabel")
        self.statusLabel.setGeometry(QRect(550, 90, 171, 21))
        self.statusLabel.setTextFormat(Qt.TextFormat.MarkdownText)
        self.statusLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.retranslateUi(Dialog)
        self.buttonBox.rejected.connect(Dialog.reject)

        QMetaObject.connectSlotsByName(Dialog)
    # setupUi

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", u"Pose Iterator", None))
        ___qtablewidgetitem = self.iterTable.horizontalHeaderItem(0)
        ___qtablewidgetitem.setText(QCoreApplication.translate("Dialog", u"Now", None));
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
        ___qtablewidgetitem17.setText(QCoreApplication.translate("Dialog", u"trigger", None));
        ___qtablewidgetitem18 = self.iterTable.verticalHeaderItem(17)
        ___qtablewidgetitem18.setText(QCoreApplication.translate("Dialog", u"time\n"
"from\n"
"start", None));
        self.label.setText(QCoreApplication.translate("Dialog", u"Pose Path", None))
        self.posePathBtn.setText(QCoreApplication.translate("Dialog", u"...", None))
        self.enabledBtn.setText("")
        self.label_2.setText(QCoreApplication.translate("Dialog", u"Enable", None))
        self.sendBtn.setText(QCoreApplication.translate("Dialog", u"SEND", None))
        self.label_4.setText(QCoreApplication.translate("Dialog", u"Converge\n"
"Criterion", None))
        self.execBtn.setText(QCoreApplication.translate("Dialog", u"EXECUTE", None))
        self.statusLabel.setText(QCoreApplication.translate("Dialog", u"STATUS: Not Loaded", None))
    # retranslateUi

