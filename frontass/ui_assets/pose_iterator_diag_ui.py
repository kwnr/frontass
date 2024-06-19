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
        self.buttonBox.setOrientation(Qt.Horizontal)
        self.buttonBox.setStandardButtons(QDialogButtonBox.Cancel|QDialogButtonBox.Ok)
        self.iter_table = QTableWidget(Dialog)
        if (self.iter_table.columnCount() < 1):
            self.iter_table.setColumnCount(1)
        __qtablewidgetitem = QTableWidgetItem()
        self.iter_table.setHorizontalHeaderItem(0, __qtablewidgetitem)
        if (self.iter_table.rowCount() < 19):
            self.iter_table.setRowCount(19)
        __qtablewidgetitem1 = QTableWidgetItem()
        __qtablewidgetitem1.setTextAlignment(Qt.AlignCenter);
        self.iter_table.setVerticalHeaderItem(0, __qtablewidgetitem1)
        __qtablewidgetitem2 = QTableWidgetItem()
        __qtablewidgetitem2.setTextAlignment(Qt.AlignCenter);
        self.iter_table.setVerticalHeaderItem(1, __qtablewidgetitem2)
        __qtablewidgetitem3 = QTableWidgetItem()
        __qtablewidgetitem3.setTextAlignment(Qt.AlignCenter);
        self.iter_table.setVerticalHeaderItem(2, __qtablewidgetitem3)
        __qtablewidgetitem4 = QTableWidgetItem()
        __qtablewidgetitem4.setTextAlignment(Qt.AlignCenter);
        self.iter_table.setVerticalHeaderItem(3, __qtablewidgetitem4)
        __qtablewidgetitem5 = QTableWidgetItem()
        __qtablewidgetitem5.setTextAlignment(Qt.AlignCenter);
        self.iter_table.setVerticalHeaderItem(4, __qtablewidgetitem5)
        __qtablewidgetitem6 = QTableWidgetItem()
        __qtablewidgetitem6.setTextAlignment(Qt.AlignCenter);
        self.iter_table.setVerticalHeaderItem(5, __qtablewidgetitem6)
        __qtablewidgetitem7 = QTableWidgetItem()
        __qtablewidgetitem7.setTextAlignment(Qt.AlignCenter);
        self.iter_table.setVerticalHeaderItem(6, __qtablewidgetitem7)
        __qtablewidgetitem8 = QTableWidgetItem()
        __qtablewidgetitem8.setTextAlignment(Qt.AlignCenter);
        self.iter_table.setVerticalHeaderItem(7, __qtablewidgetitem8)
        __qtablewidgetitem9 = QTableWidgetItem()
        __qtablewidgetitem9.setTextAlignment(Qt.AlignCenter);
        self.iter_table.setVerticalHeaderItem(8, __qtablewidgetitem9)
        __qtablewidgetitem10 = QTableWidgetItem()
        __qtablewidgetitem10.setTextAlignment(Qt.AlignCenter);
        self.iter_table.setVerticalHeaderItem(9, __qtablewidgetitem10)
        __qtablewidgetitem11 = QTableWidgetItem()
        __qtablewidgetitem11.setTextAlignment(Qt.AlignCenter);
        self.iter_table.setVerticalHeaderItem(10, __qtablewidgetitem11)
        __qtablewidgetitem12 = QTableWidgetItem()
        __qtablewidgetitem12.setTextAlignment(Qt.AlignCenter);
        self.iter_table.setVerticalHeaderItem(11, __qtablewidgetitem12)
        __qtablewidgetitem13 = QTableWidgetItem()
        __qtablewidgetitem13.setTextAlignment(Qt.AlignCenter);
        self.iter_table.setVerticalHeaderItem(12, __qtablewidgetitem13)
        __qtablewidgetitem14 = QTableWidgetItem()
        __qtablewidgetitem14.setTextAlignment(Qt.AlignCenter);
        self.iter_table.setVerticalHeaderItem(13, __qtablewidgetitem14)
        __qtablewidgetitem15 = QTableWidgetItem()
        __qtablewidgetitem15.setTextAlignment(Qt.AlignCenter);
        self.iter_table.setVerticalHeaderItem(14, __qtablewidgetitem15)
        __qtablewidgetitem16 = QTableWidgetItem()
        __qtablewidgetitem16.setTextAlignment(Qt.AlignCenter);
        self.iter_table.setVerticalHeaderItem(15, __qtablewidgetitem16)
        __qtablewidgetitem17 = QTableWidgetItem()
        __qtablewidgetitem17.setTextAlignment(Qt.AlignCenter);
        self.iter_table.setVerticalHeaderItem(16, __qtablewidgetitem17)
        __qtablewidgetitem18 = QTableWidgetItem()
        __qtablewidgetitem18.setTextAlignment(Qt.AlignCenter);
        self.iter_table.setVerticalHeaderItem(17, __qtablewidgetitem18)
        __qtablewidgetitem19 = QTableWidgetItem()
        __qtablewidgetitem19.setTextAlignment(Qt.AlignCenter);
        self.iter_table.setVerticalHeaderItem(18, __qtablewidgetitem19)
        self.iter_table.setObjectName(u"iter_table")
        self.iter_table.setGeometry(QRect(10, 120, 711, 591))
        self.iter_table.setFocusPolicy(Qt.NoFocus)
        self.iter_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.iter_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.iter_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.iter_table.setSelectionMode(QAbstractItemView.NoSelection)
        self.iter_table.setSelectionBehavior(QAbstractItemView.SelectColumns)
        self.iter_table.verticalHeader().setStretchLastSection(True)
        self.label = QLabel(Dialog)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(10, 70, 67, 25))
        self.pose_path_lineedit = QLineEdit(Dialog)
        self.pose_path_lineedit.setObjectName(u"pose_path_lineedit")
        self.pose_path_lineedit.setGeometry(QRect(90, 70, 601, 25))
        self.pose_path_btn = QToolButton(Dialog)
        self.pose_path_btn.setObjectName(u"pose_path_btn")
        self.pose_path_btn.setGeometry(QRect(700, 70, 26, 25))
        self.enabled_btn = QPushButton(Dialog)
        self.enabled_btn.setObjectName(u"enabled_btn")
        self.enabled_btn.setEnabled(True)
        self.enabled_btn.setGeometry(QRect(80, 20, 41, 30))
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.enabled_btn.sizePolicy().hasHeightForWidth())
        self.enabled_btn.setSizePolicy(sizePolicy)
        self.enabled_btn.setMinimumSize(QSize(41, 21))
        self.enabled_btn.setMaximumSize(QSize(16777215, 16777215))
        self.enabled_btn.setBaseSize(QSize(41, 21))
        self.enabled_btn.setStyleSheet(u"QPushButton{\n"
"	background-color: none;\n"
"	border-radius: 5%;\n"
"}")
        icon = QIcon()
        icon.addFile(u":/toggle/off", QSize(), QIcon.Normal, QIcon.Off)
        icon.addFile(u":/toggle/on", QSize(), QIcon.Normal, QIcon.On)
        self.enabled_btn.setIcon(icon)
        self.enabled_btn.setIconSize(QSize(41, 40))
        self.enabled_btn.setCheckable(True)
        self.enabled_btn.setChecked(False)
        self.label_2 = QLabel(Dialog)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(10, 20, 61, 30))
        self.next_pose_btn = QPushButton(Dialog)
        self.next_pose_btn.setObjectName(u"next_pose_btn")
        self.next_pose_btn.setGeometry(QRect(600, 20, 121, 31))
        self.conv_crit_line = QLineEdit(Dialog)
        self.conv_crit_line.setObjectName(u"conv_crit_line")
        self.conv_crit_line.setGeometry(QRect(410, 20, 113, 31))
        self.label_4 = QLabel(Dialog)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(330, 20, 81, 30))

        self.retranslateUi(Dialog)
        self.buttonBox.rejected.connect(Dialog.reject)

        QMetaObject.connectSlotsByName(Dialog)
    # setupUi

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", u"Pose Iterator", None))
        ___qtablewidgetitem = self.iter_table.horizontalHeaderItem(0)
        ___qtablewidgetitem.setText(QCoreApplication.translate("Dialog", u"Now", None));
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
        ___qtablewidgetitem17.setText(QCoreApplication.translate("Dialog", u"Trig", None));
        ___qtablewidgetitem18 = self.iter_table.verticalHeaderItem(17)
        ___qtablewidgetitem18.setText(QCoreApplication.translate("Dialog", u"Exec", None));
        ___qtablewidgetitem19 = self.iter_table.verticalHeaderItem(18)
        ___qtablewidgetitem19.setText(QCoreApplication.translate("Dialog", u"Wait", None));
        self.label.setText(QCoreApplication.translate("Dialog", u"Pose Path", None))
        self.pose_path_btn.setText(QCoreApplication.translate("Dialog", u"...", None))
        self.enabled_btn.setText("")
        self.label_2.setText(QCoreApplication.translate("Dialog", u"Enable", None))
        self.next_pose_btn.setText(QCoreApplication.translate("Dialog", u"Next Pose", None))
        self.label_4.setText(QCoreApplication.translate("Dialog", u"Converge\n"
"Criterion", None))
    # retranslateUi

