# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'control_panel.ui'
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
from PySide6.QtWidgets import (QApplication, QFrame, QGridLayout, QLabel,
    QLayout, QMainWindow, QMenuBar, QProgressBar,
    QPushButton, QSizePolicy, QStatusBar, QVBoxLayout,
    QWidget)
import buttons_rc

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.setEnabled(True)
        MainWindow.resize(1418, 794)
        font = QFont()
        font.setBold(False)
        MainWindow.setFont(font)
        MainWindow.setAutoFillBackground(False)
        MainWindow.setStyleSheet(u"QMainWindow{\n"
"	background-color: #f9f9f9;\n"
"}\n"
"QPushButton{\n"
"	background-color: rgba(0,0,0,0.05);\n"
"	background-color: #ffffff;\n"
"	color: #000000;\n"
"	border-radius: 5%;\n"
"}\n"
"QPushButton:hover{\n"
"	background-color: rgba(0,0,0,0.1);\n"
"}\n"
"QPushButton:pressed{\n"
"	background-color: rgba(0,0,0,0.2);\n"
"}\n"
"QPushButton:checked{\n"
"	background-color: #007aff;\n"
"	background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop: 0 #268eff, stop: 1 #007aff);\n"
"	color: #ffffff\n"
"}\n"
"QPushButton:disabled{\n"
"	color: rgba(0,0,0,0.5);\n"
"	background-color: rgba(255,255,255,0.5);\n"
"}\n"
"QComboBox{\n"
"	border-radius: 5%;\n"
"}\n"
"QComboBox::drop-down{\n"
"	width: 20px;\n"
"	background-image: url(:/combobox/1x);\n"
"	background-position: center center;\n"
"}\n"
"QProgressBar{\n"
"	border-radius: 5% 5% 0 0;\n"
"}\n"
"QProgressBar::chunk{\n"
"	background-color: #007aff\n"
"}")
        MainWindow.setUnifiedTitleAndToolBarOnMac(False)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.power_btn = QPushButton(self.centralwidget)
        self.power_btn.setObjectName(u"power_btn")
        self.power_btn.setGeometry(QRect(39, 20, 131, 131))
        self.power_btn.setMinimumSize(QSize(131, 131))
        self.power_btn.setBaseSize(QSize(131, 131))
        font1 = QFont()
        font1.setPointSize(14)
        font1.setBold(False)
        self.power_btn.setFont(font1)
        self.power_btn.setStyleSheet(u"QPushButton{\n"
"	border-radius: 1%;\n"
"	background-color: none;\n"
"}")
        icon = QIcon()
        icon.addFile(u":/power/off3", QSize(), QIcon.Normal, QIcon.Off)
        icon.addFile(u":/power/on", QSize(), QIcon.Normal, QIcon.On)
        self.power_btn.setIcon(icon)
        self.power_btn.setIconSize(QSize(120, 120))
        self.power_btn.setCheckable(True)
        self.power_btn.setChecked(False)
        self.power_btn.setFlat(True)
        self.pump_frame = QFrame(self.centralwidget)
        self.pump_frame.setObjectName(u"pump_frame")
        self.pump_frame.setGeometry(QRect(220, 20, 521, 221))
        self.pump_frame.setFrameShape(QFrame.Shape.StyledPanel)
        self.pump_frame.setFrameShadow(QFrame.Shadow.Raised)
        self.pump_power_btn = QPushButton(self.pump_frame)
        self.pump_power_btn.setObjectName(u"pump_power_btn")
        self.pump_power_btn.setGeometry(QRect(21, 20, 75, 75))
        self.pump_power_btn.setStyleSheet(u"QPushButton{\n"
"	border-radius: 1px;\n"
"	background-color: none;\n"
"}")
        icon1 = QIcon()
        icon1.addFile(u":/pump/off", QSize(), QIcon.Normal, QIcon.Off)
        icon1.addFile(u":/pump/on", QSize(), QIcon.Normal, QIcon.On)
        self.pump_power_btn.setIcon(icon1)
        self.pump_power_btn.setIconSize(QSize(60, 60))
        self.pump_power_btn.setCheckable(True)
        self.pump_mode_btn = QPushButton(self.pump_frame)
        self.pump_mode_btn.setObjectName(u"pump_mode_btn")
        self.pump_mode_btn.setGeometry(QRect(111, 20, 75, 75))
        self.pump_mode_btn.setStyleSheet(u"QPushButton{\n"
"	border-radius: 1px;\n"
"	background-color: none;\n"
"}")
        icon2 = QIcon()
        icon2.addFile(u":/auto/man", QSize(), QIcon.Normal, QIcon.Off)
        icon2.addFile(u":/auto/auto", QSize(), QIcon.Normal, QIcon.On)
        self.pump_mode_btn.setIcon(icon2)
        self.pump_mode_btn.setIconSize(QSize(60, 60))
        self.pump_mode_btn.setCheckable(True)
        self.pump_mode_btn.setChecked(False)
        self.pump_mode_btn.setFlat(True)
        self.act_rpm_text = QLabel(self.pump_frame)
        self.act_rpm_text.setObjectName(u"act_rpm_text")
        self.act_rpm_text.setGeometry(QRect(40, 170, 61, 31))
        font2 = QFont()
        font2.setPointSize(11)
        font2.setBold(False)
        self.act_rpm_text.setFont(font2)
        self.act_rpm_text.setStyleSheet(u"background-color: rgb(186, 189, 182);")
        self.act_rpm_text.setFrameShape(QFrame.Shape.NoFrame)
        self.act_rpm_text.setFrameShadow(QFrame.Shadow.Plain)
        self.act_rpm_text.setScaledContents(False)
        self.act_rpm_text.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)
        self.act_rpm_text.setWordWrap(False)
        self.act_rpm_text.setMargin(10)
        self.act_rpm_text.setIndent(-1)
        self.label_3 = QLabel(self.pump_frame)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(40, 140, 101, 31))
        self.label_3.setFont(font2)
        self.label_2 = QLabel(self.pump_frame)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(100, 170, 41, 31))
        self.label_2.setAlignment(Qt.AlignmentFlag.AlignLeading|Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)
        self.label_2.setMargin(5)
        self.label_26 = QLabel(self.pump_frame)
        self.label_26.setObjectName(u"label_26")
        self.label_26.setGeometry(QRect(220, 170, 41, 31))
        self.label_26.setAlignment(Qt.AlignmentFlag.AlignLeading|Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)
        self.label_26.setMargin(5)
        self.des_rpm_text = QLabel(self.pump_frame)
        self.des_rpm_text.setObjectName(u"des_rpm_text")
        self.des_rpm_text.setGeometry(QRect(160, 170, 61, 31))
        self.des_rpm_text.setFont(font2)
        self.des_rpm_text.setStyleSheet(u"background-color: rgb(186, 189, 182);")
        self.des_rpm_text.setFrameShape(QFrame.Shape.NoFrame)
        self.des_rpm_text.setFrameShadow(QFrame.Shadow.Plain)
        self.des_rpm_text.setScaledContents(False)
        self.des_rpm_text.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)
        self.des_rpm_text.setWordWrap(False)
        self.des_rpm_text.setMargin(10)
        self.des_rpm_text.setIndent(-1)
        self.label_27 = QLabel(self.pump_frame)
        self.label_27.setObjectName(u"label_27")
        self.label_27.setGeometry(QRect(160, 140, 101, 31))
        self.label_27.setFont(font2)
        self.label_29 = QLabel(self.pump_frame)
        self.label_29.setObjectName(u"label_29")
        self.label_29.setGeometry(QRect(340, 170, 41, 31))
        self.label_29.setAlignment(Qt.AlignmentFlag.AlignLeading|Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)
        self.label_29.setMargin(5)
        self.label_31 = QLabel(self.pump_frame)
        self.label_31.setObjectName(u"label_31")
        self.label_31.setGeometry(QRect(460, 170, 41, 31))
        self.label_31.setAlignment(Qt.AlignmentFlag.AlignLeading|Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)
        self.label_31.setMargin(5)
        self.des_cur_text = QLabel(self.pump_frame)
        self.des_cur_text.setObjectName(u"des_cur_text")
        self.des_cur_text.setGeometry(QRect(280, 170, 61, 31))
        self.des_cur_text.setFont(font2)
        self.des_cur_text.setStyleSheet(u"background-color: rgb(186, 189, 182);")
        self.des_cur_text.setFrameShape(QFrame.Shape.NoFrame)
        self.des_cur_text.setFrameShadow(QFrame.Shadow.Plain)
        self.des_cur_text.setScaledContents(False)
        self.des_cur_text.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)
        self.des_cur_text.setWordWrap(False)
        self.des_cur_text.setMargin(10)
        self.des_cur_text.setIndent(-1)
        self.elmo_temp_text = QLabel(self.pump_frame)
        self.elmo_temp_text.setObjectName(u"elmo_temp_text")
        self.elmo_temp_text.setGeometry(QRect(400, 170, 61, 31))
        self.elmo_temp_text.setFont(font2)
        self.elmo_temp_text.setStyleSheet(u"background-color: rgb(186, 189, 182);")
        self.elmo_temp_text.setFrameShape(QFrame.Shape.NoFrame)
        self.elmo_temp_text.setFrameShadow(QFrame.Shadow.Plain)
        self.elmo_temp_text.setScaledContents(False)
        self.elmo_temp_text.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)
        self.elmo_temp_text.setWordWrap(False)
        self.elmo_temp_text.setMargin(10)
        self.elmo_temp_text.setIndent(-1)
        self.label_32 = QLabel(self.pump_frame)
        self.label_32.setObjectName(u"label_32")
        self.label_32.setGeometry(QRect(400, 140, 101, 31))
        self.label_32.setFont(font2)
        self.label_33 = QLabel(self.pump_frame)
        self.label_33.setObjectName(u"label_33")
        self.label_33.setGeometry(QRect(280, 140, 101, 31))
        self.label_33.setFont(font2)
        self.label_9 = QLabel(self.pump_frame)
        self.label_9.setObjectName(u"label_9")
        self.label_9.setGeometry(QRect(21, 90, 75, 17))
        self.label_9.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_10 = QLabel(self.pump_frame)
        self.label_10.setObjectName(u"label_10")
        self.label_10.setGeometry(QRect(111, 90, 75, 17))
        self.label_10.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.pump_config_btn = QPushButton(self.pump_frame)
        self.pump_config_btn.setObjectName(u"pump_config_btn")
        self.pump_config_btn.setGeometry(QRect(201, 20, 75, 75))
        self.pump_config_btn.setLayoutDirection(Qt.LayoutDirection.LeftToRight)
        self.pump_config_btn.setStyleSheet(u"QPushButton{\n"
"	background-color:none;\n"
"	border-radius: 30px;\n"
"}\n"
"QPushButton:hover{\n"
"	background-color: rgba(0,0,0,0.1);\n"
"}\n"
"QPushButton:pressed{\n"
"	background-color: rgba(0,0,0,0.2);\n"
"}")
        icon3 = QIcon()
        icon3.addFile(u":/gear/black", QSize(), QIcon.Normal, QIcon.Off)
        self.pump_config_btn.setIcon(icon3)
        self.pump_config_btn.setIconSize(QSize(60, 60))
        self.label_11 = QLabel(self.pump_frame)
        self.label_11.setObjectName(u"label_11")
        self.label_11.setGeometry(QRect(201, 90, 75, 17))
        self.label_11.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.arm_power_frame = QFrame(self.centralwidget)
        self.arm_power_frame.setObjectName(u"arm_power_frame")
        self.arm_power_frame.setGeometry(QRect(220, 260, 521, 231))
        self.arm_power_frame.setFrameShape(QFrame.Shape.StyledPanel)
        self.arm_power_frame.setFrameShadow(QFrame.Shadow.Raised)
        self.left_arm_power_btn = QPushButton(self.arm_power_frame)
        self.left_arm_power_btn.setObjectName(u"left_arm_power_btn")
        self.left_arm_power_btn.setEnabled(True)
        self.left_arm_power_btn.setGeometry(QRect(20, 20, 131, 51))
        self.left_arm_power_btn.setStyleSheet(u"")
        self.left_arm_power_btn.setCheckable(True)
        self.right_arm_power_btn = QPushButton(self.arm_power_frame)
        self.right_arm_power_btn.setObjectName(u"right_arm_power_btn")
        self.right_arm_power_btn.setGeometry(QRect(170, 20, 131, 51))
        self.right_arm_power_btn.setStyleSheet(u"")
        self.right_arm_power_btn.setCheckable(True)
        self.gridLayoutWidget_2 = QWidget(self.arm_power_frame)
        self.gridLayoutWidget_2.setObjectName(u"gridLayoutWidget_2")
        self.gridLayoutWidget_2.setGeometry(QRect(20, 80, 58, 141))
        self.gridLayout_2 = QGridLayout(self.gridLayoutWidget_2)
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.gridLayout_2.setSizeConstraint(QLayout.SizeConstraint.SetMinimumSize)
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.label_6 = QLabel(self.gridLayoutWidget_2)
        self.label_6.setObjectName(u"label_6")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_6.sizePolicy().hasHeightForWidth())
        self.label_6.setSizePolicy(sizePolicy)
        self.label_6.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.gridLayout_2.addWidget(self.label_6, 2, 0, 1, 1)

        self.label_5 = QLabel(self.gridLayoutWidget_2)
        self.label_5.setObjectName(u"label_5")
        sizePolicy.setHeightForWidth(self.label_5.sizePolicy().hasHeightForWidth())
        self.label_5.setSizePolicy(sizePolicy)
        self.label_5.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.gridLayout_2.addWidget(self.label_5, 1, 0, 1, 1)

        self.label_4 = QLabel(self.gridLayoutWidget_2)
        self.label_4.setObjectName(u"label_4")
        sizePolicy.setHeightForWidth(self.label_4.sizePolicy().hasHeightForWidth())
        self.label_4.setSizePolicy(sizePolicy)
        self.label_4.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.gridLayout_2.addWidget(self.label_4, 0, 0, 1, 1)

        self.label_7 = QLabel(self.gridLayoutWidget_2)
        self.label_7.setObjectName(u"label_7")
        sizePolicy.setHeightForWidth(self.label_7.sizePolicy().hasHeightForWidth())
        self.label_7.setSizePolicy(sizePolicy)
        self.label_7.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.gridLayout_2.addWidget(self.label_7, 3, 0, 1, 1)

        self.l1_power_btn = QPushButton(self.gridLayoutWidget_2)
        self.l1_power_btn.setObjectName(u"l1_power_btn")
        self.l1_power_btn.setEnabled(True)
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.l1_power_btn.sizePolicy().hasHeightForWidth())
        self.l1_power_btn.setSizePolicy(sizePolicy1)
        self.l1_power_btn.setMinimumSize(QSize(41, 21))
        self.l1_power_btn.setMaximumSize(QSize(16777215, 16777215))
        self.l1_power_btn.setBaseSize(QSize(41, 21))
        self.l1_power_btn.setStyleSheet(u"QPushButton{\n"
"	background-color: none;\n"
"}")
        icon4 = QIcon()
        icon4.addFile(u":/toggle/off", QSize(), QIcon.Normal, QIcon.Off)
        icon4.addFile(u":/toggle/on", QSize(), QIcon.Normal, QIcon.On)
        self.l1_power_btn.setIcon(icon4)
        self.l1_power_btn.setIconSize(QSize(41, 40))
        self.l1_power_btn.setCheckable(True)
        self.l1_power_btn.setChecked(True)

        self.gridLayout_2.addWidget(self.l1_power_btn, 0, 1, 1, 1)

        self.l3_power_btn = QPushButton(self.gridLayoutWidget_2)
        self.l3_power_btn.setObjectName(u"l3_power_btn")
        self.l3_power_btn.setEnabled(True)
        sizePolicy1.setHeightForWidth(self.l3_power_btn.sizePolicy().hasHeightForWidth())
        self.l3_power_btn.setSizePolicy(sizePolicy1)
        self.l3_power_btn.setMinimumSize(QSize(41, 21))
        self.l3_power_btn.setMaximumSize(QSize(16777215, 16777215))
        self.l3_power_btn.setBaseSize(QSize(41, 21))
        self.l3_power_btn.setStyleSheet(u"QPushButton{\n"
"	background-color: none;\n"
"}")
        self.l3_power_btn.setIcon(icon4)
        self.l3_power_btn.setIconSize(QSize(41, 40))
        self.l3_power_btn.setCheckable(True)
        self.l3_power_btn.setChecked(True)

        self.gridLayout_2.addWidget(self.l3_power_btn, 2, 1, 1, 1)

        self.l2_power_btn = QPushButton(self.gridLayoutWidget_2)
        self.l2_power_btn.setObjectName(u"l2_power_btn")
        self.l2_power_btn.setEnabled(True)
        sizePolicy1.setHeightForWidth(self.l2_power_btn.sizePolicy().hasHeightForWidth())
        self.l2_power_btn.setSizePolicy(sizePolicy1)
        self.l2_power_btn.setMinimumSize(QSize(41, 21))
        self.l2_power_btn.setMaximumSize(QSize(16777215, 16777215))
        self.l2_power_btn.setBaseSize(QSize(41, 21))
        self.l2_power_btn.setStyleSheet(u"QPushButton{\n"
"	background-color: none;\n"
"}")
        self.l2_power_btn.setIcon(icon4)
        self.l2_power_btn.setIconSize(QSize(41, 40))
        self.l2_power_btn.setCheckable(True)
        self.l2_power_btn.setChecked(True)

        self.gridLayout_2.addWidget(self.l2_power_btn, 1, 1, 1, 1)

        self.l4_power_btn = QPushButton(self.gridLayoutWidget_2)
        self.l4_power_btn.setObjectName(u"l4_power_btn")
        self.l4_power_btn.setEnabled(True)
        sizePolicy1.setHeightForWidth(self.l4_power_btn.sizePolicy().hasHeightForWidth())
        self.l4_power_btn.setSizePolicy(sizePolicy1)
        self.l4_power_btn.setMinimumSize(QSize(41, 21))
        self.l4_power_btn.setMaximumSize(QSize(16777215, 16777215))
        self.l4_power_btn.setBaseSize(QSize(41, 21))
        self.l4_power_btn.setStyleSheet(u"QPushButton{\n"
"	background-color: none;\n"
"}")
        self.l4_power_btn.setIcon(icon4)
        self.l4_power_btn.setIconSize(QSize(41, 40))
        self.l4_power_btn.setCheckable(True)
        self.l4_power_btn.setChecked(True)

        self.gridLayout_2.addWidget(self.l4_power_btn, 3, 1, 1, 1)

        self.gridLayoutWidget_3 = QWidget(self.arm_power_frame)
        self.gridLayoutWidget_3.setObjectName(u"gridLayoutWidget_3")
        self.gridLayoutWidget_3.setGeometry(QRect(90, 80, 61, 141))
        self.gridLayout_3 = QGridLayout(self.gridLayoutWidget_3)
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.gridLayout_3.setSizeConstraint(QLayout.SizeConstraint.SetMinimumSize)
        self.gridLayout_3.setContentsMargins(0, 0, 0, 0)
        self.label_8 = QLabel(self.gridLayoutWidget_3)
        self.label_8.setObjectName(u"label_8")
        self.label_8.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.gridLayout_3.addWidget(self.label_8, 1, 0, 1, 1)

        self.label_14 = QLabel(self.gridLayoutWidget_3)
        self.label_14.setObjectName(u"label_14")
        self.label_14.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.gridLayout_3.addWidget(self.label_14, 0, 0, 1, 1)

        self.l6_power_btn = QPushButton(self.gridLayoutWidget_3)
        self.l6_power_btn.setObjectName(u"l6_power_btn")
        self.l6_power_btn.setEnabled(True)
        self.l6_power_btn.setMinimumSize(QSize(41, 21))
        self.l6_power_btn.setBaseSize(QSize(41, 21))
        self.l6_power_btn.setStyleSheet(u"QPushButton{\n"
"	background-color: none;\n"
"}")
        self.l6_power_btn.setIcon(icon4)
        self.l6_power_btn.setIconSize(QSize(40, 40))
        self.l6_power_btn.setCheckable(True)
        self.l6_power_btn.setChecked(True)

        self.gridLayout_3.addWidget(self.l6_power_btn, 1, 1, 1, 1)

        self.label_15 = QLabel(self.gridLayoutWidget_3)
        self.label_15.setObjectName(u"label_15")
        self.label_15.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.gridLayout_3.addWidget(self.label_15, 3, 0, 1, 1)

        self.label_16 = QLabel(self.gridLayoutWidget_3)
        self.label_16.setObjectName(u"label_16")
        self.label_16.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.gridLayout_3.addWidget(self.label_16, 2, 0, 1, 1)

        self.l5_power_btn = QPushButton(self.gridLayoutWidget_3)
        self.l5_power_btn.setObjectName(u"l5_power_btn")
        self.l5_power_btn.setEnabled(True)
        self.l5_power_btn.setMinimumSize(QSize(41, 21))
        self.l5_power_btn.setBaseSize(QSize(41, 21))
        self.l5_power_btn.setStyleSheet(u"QPushButton{\n"
"	background-color: none;\n"
"}")
        self.l5_power_btn.setIcon(icon4)
        self.l5_power_btn.setIconSize(QSize(40, 40))
        self.l5_power_btn.setCheckable(True)
        self.l5_power_btn.setChecked(True)

        self.gridLayout_3.addWidget(self.l5_power_btn, 0, 1, 1, 1)

        self.l8_power_btn = QPushButton(self.gridLayoutWidget_3)
        self.l8_power_btn.setObjectName(u"l8_power_btn")
        self.l8_power_btn.setEnabled(True)
        self.l8_power_btn.setMinimumSize(QSize(41, 21))
        self.l8_power_btn.setBaseSize(QSize(41, 21))
        self.l8_power_btn.setStyleSheet(u"QPushButton{\n"
"	background-color: none;\n"
"}")
        self.l8_power_btn.setIcon(icon4)
        self.l8_power_btn.setIconSize(QSize(40, 40))
        self.l8_power_btn.setCheckable(True)
        self.l8_power_btn.setChecked(True)

        self.gridLayout_3.addWidget(self.l8_power_btn, 3, 1, 1, 1)

        self.l7_power_btn = QPushButton(self.gridLayoutWidget_3)
        self.l7_power_btn.setObjectName(u"l7_power_btn")
        self.l7_power_btn.setEnabled(True)
        self.l7_power_btn.setMinimumSize(QSize(41, 21))
        self.l7_power_btn.setBaseSize(QSize(41, 21))
        self.l7_power_btn.setStyleSheet(u"QPushButton{\n"
"	background-color: none;\n"
"}")
        self.l7_power_btn.setIcon(icon4)
        self.l7_power_btn.setIconSize(QSize(40, 40))
        self.l7_power_btn.setCheckable(True)
        self.l7_power_btn.setChecked(True)

        self.gridLayout_3.addWidget(self.l7_power_btn, 2, 1, 1, 1)

        self.gridLayoutWidget_4 = QWidget(self.arm_power_frame)
        self.gridLayoutWidget_4.setObjectName(u"gridLayoutWidget_4")
        self.gridLayoutWidget_4.setGeometry(QRect(240, 80, 61, 141))
        self.gridLayout_4 = QGridLayout(self.gridLayoutWidget_4)
        self.gridLayout_4.setObjectName(u"gridLayout_4")
        self.gridLayout_4.setSizeConstraint(QLayout.SizeConstraint.SetMinimumSize)
        self.gridLayout_4.setContentsMargins(0, 0, 0, 0)
        self.label_34 = QLabel(self.gridLayoutWidget_4)
        self.label_34.setObjectName(u"label_34")
        sizePolicy.setHeightForWidth(self.label_34.sizePolicy().hasHeightForWidth())
        self.label_34.setSizePolicy(sizePolicy)
        self.label_34.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.gridLayout_4.addWidget(self.label_34, 2, 0, 1, 1)

        self.label_18 = QLabel(self.gridLayoutWidget_4)
        self.label_18.setObjectName(u"label_18")
        sizePolicy.setHeightForWidth(self.label_18.sizePolicy().hasHeightForWidth())
        self.label_18.setSizePolicy(sizePolicy)
        self.label_18.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.gridLayout_4.addWidget(self.label_18, 0, 0, 1, 1)

        self.r7_power_btn = QPushButton(self.gridLayoutWidget_4)
        self.r7_power_btn.setObjectName(u"r7_power_btn")
        self.r7_power_btn.setEnabled(True)
        self.r7_power_btn.setMinimumSize(QSize(41, 21))
        self.r7_power_btn.setBaseSize(QSize(41, 21))
        self.r7_power_btn.setStyleSheet(u"QPushButton{\n"
"	background-color: none;\n"
"}")
        self.r7_power_btn.setIcon(icon4)
        self.r7_power_btn.setIconSize(QSize(40, 40))
        self.r7_power_btn.setCheckable(True)
        self.r7_power_btn.setChecked(True)

        self.gridLayout_4.addWidget(self.r7_power_btn, 2, 1, 1, 1)

        self.r5_power_btn = QPushButton(self.gridLayoutWidget_4)
        self.r5_power_btn.setObjectName(u"r5_power_btn")
        self.r5_power_btn.setEnabled(True)
        self.r5_power_btn.setMinimumSize(QSize(41, 21))
        self.r5_power_btn.setBaseSize(QSize(41, 21))
        self.r5_power_btn.setStyleSheet(u"QPushButton{\n"
"	background-color: none;\n"
"}")
        self.r5_power_btn.setIcon(icon4)
        self.r5_power_btn.setIconSize(QSize(40, 40))
        self.r5_power_btn.setCheckable(True)
        self.r5_power_btn.setChecked(True)

        self.gridLayout_4.addWidget(self.r5_power_btn, 0, 1, 1, 1)

        self.r6_power_btn = QPushButton(self.gridLayoutWidget_4)
        self.r6_power_btn.setObjectName(u"r6_power_btn")
        self.r6_power_btn.setEnabled(True)
        self.r6_power_btn.setMinimumSize(QSize(41, 21))
        self.r6_power_btn.setBaseSize(QSize(41, 21))
        self.r6_power_btn.setStyleSheet(u"QPushButton{\n"
"	background-color: none;\n"
"}")
        self.r6_power_btn.setIcon(icon4)
        self.r6_power_btn.setIconSize(QSize(40, 40))
        self.r6_power_btn.setCheckable(True)
        self.r6_power_btn.setChecked(True)

        self.gridLayout_4.addWidget(self.r6_power_btn, 1, 1, 1, 1)

        self.label_19 = QLabel(self.gridLayoutWidget_4)
        self.label_19.setObjectName(u"label_19")
        sizePolicy.setHeightForWidth(self.label_19.sizePolicy().hasHeightForWidth())
        self.label_19.setSizePolicy(sizePolicy)
        self.label_19.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.gridLayout_4.addWidget(self.label_19, 3, 0, 1, 1)

        self.r8_power_btn = QPushButton(self.gridLayoutWidget_4)
        self.r8_power_btn.setObjectName(u"r8_power_btn")
        self.r8_power_btn.setEnabled(True)
        self.r8_power_btn.setMinimumSize(QSize(41, 21))
        self.r8_power_btn.setBaseSize(QSize(41, 21))
        self.r8_power_btn.setStyleSheet(u"QPushButton{\n"
"	background-color: none;\n"
"}")
        self.r8_power_btn.setIcon(icon4)
        self.r8_power_btn.setIconSize(QSize(40, 40))
        self.r8_power_btn.setCheckable(True)
        self.r8_power_btn.setChecked(True)

        self.gridLayout_4.addWidget(self.r8_power_btn, 3, 1, 1, 1)

        self.label_17 = QLabel(self.gridLayoutWidget_4)
        self.label_17.setObjectName(u"label_17")
        sizePolicy.setHeightForWidth(self.label_17.sizePolicy().hasHeightForWidth())
        self.label_17.setSizePolicy(sizePolicy)
        self.label_17.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.gridLayout_4.addWidget(self.label_17, 1, 0, 1, 1)

        self.gridLayoutWidget_5 = QWidget(self.arm_power_frame)
        self.gridLayoutWidget_5.setObjectName(u"gridLayoutWidget_5")
        self.gridLayoutWidget_5.setGeometry(QRect(170, 80, 61, 141))
        self.gridLayout_5 = QGridLayout(self.gridLayoutWidget_5)
        self.gridLayout_5.setObjectName(u"gridLayout_5")
        self.gridLayout_5.setSizeConstraint(QLayout.SizeConstraint.SetMinimumSize)
        self.gridLayout_5.setContentsMargins(0, 0, 0, 0)
        self.label_35 = QLabel(self.gridLayoutWidget_5)
        self.label_35.setObjectName(u"label_35")
        self.label_35.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.gridLayout_5.addWidget(self.label_35, 1, 0, 1, 1)

        self.label_36 = QLabel(self.gridLayoutWidget_5)
        self.label_36.setObjectName(u"label_36")
        self.label_36.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.gridLayout_5.addWidget(self.label_36, 0, 0, 1, 1)

        self.r2_power_btn = QPushButton(self.gridLayoutWidget_5)
        self.r2_power_btn.setObjectName(u"r2_power_btn")
        self.r2_power_btn.setEnabled(True)
        self.r2_power_btn.setMinimumSize(QSize(41, 21))
        self.r2_power_btn.setBaseSize(QSize(41, 21))
        self.r2_power_btn.setStyleSheet(u"QPushButton{\n"
"	background-color: none;\n"
"}")
        self.r2_power_btn.setIcon(icon4)
        self.r2_power_btn.setIconSize(QSize(40, 40))
        self.r2_power_btn.setCheckable(True)
        self.r2_power_btn.setChecked(True)

        self.gridLayout_5.addWidget(self.r2_power_btn, 1, 1, 1, 1)

        self.label_37 = QLabel(self.gridLayoutWidget_5)
        self.label_37.setObjectName(u"label_37")
        self.label_37.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.gridLayout_5.addWidget(self.label_37, 3, 0, 1, 1)

        self.label_38 = QLabel(self.gridLayoutWidget_5)
        self.label_38.setObjectName(u"label_38")
        self.label_38.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.gridLayout_5.addWidget(self.label_38, 2, 0, 1, 1)

        self.r1_power_btn = QPushButton(self.gridLayoutWidget_5)
        self.r1_power_btn.setObjectName(u"r1_power_btn")
        self.r1_power_btn.setEnabled(True)
        self.r1_power_btn.setMinimumSize(QSize(41, 21))
        self.r1_power_btn.setBaseSize(QSize(41, 21))
        self.r1_power_btn.setStyleSheet(u"QPushButton{\n"
"	background-color: none;\n"
"}")
        self.r1_power_btn.setIcon(icon4)
        self.r1_power_btn.setIconSize(QSize(40, 40))
        self.r1_power_btn.setCheckable(True)
        self.r1_power_btn.setChecked(True)

        self.gridLayout_5.addWidget(self.r1_power_btn, 0, 1, 1, 1)

        self.r4_power_btn = QPushButton(self.gridLayoutWidget_5)
        self.r4_power_btn.setObjectName(u"r4_power_btn")
        self.r4_power_btn.setEnabled(True)
        self.r4_power_btn.setMinimumSize(QSize(41, 21))
        self.r4_power_btn.setBaseSize(QSize(41, 21))
        self.r4_power_btn.setStyleSheet(u"QPushButton{\n"
"	background-color: none;\n"
"}")
        self.r4_power_btn.setIcon(icon4)
        self.r4_power_btn.setIconSize(QSize(40, 40))
        self.r4_power_btn.setCheckable(True)
        self.r4_power_btn.setChecked(True)

        self.gridLayout_5.addWidget(self.r4_power_btn, 3, 1, 1, 1)

        self.r3_power_btn = QPushButton(self.gridLayoutWidget_5)
        self.r3_power_btn.setObjectName(u"r3_power_btn")
        self.r3_power_btn.setEnabled(True)
        self.r3_power_btn.setMinimumSize(QSize(41, 21))
        self.r3_power_btn.setBaseSize(QSize(41, 21))
        self.r3_power_btn.setStyleSheet(u"QPushButton{\n"
"	background-color: none;\n"
"}")
        self.r3_power_btn.setIcon(icon4)
        self.r3_power_btn.setIconSize(QSize(40, 40))
        self.r3_power_btn.setCheckable(True)
        self.r3_power_btn.setChecked(True)

        self.gridLayout_5.addWidget(self.r3_power_btn, 2, 1, 1, 1)

        self.show_state_btn = QPushButton(self.arm_power_frame)
        self.show_state_btn.setObjectName(u"show_state_btn")
        self.show_state_btn.setGeometry(QRect(340, 160, 161, 51))
        self.show_state_btn.setFont(font)
        self.preset_mode_btn = QPushButton(self.arm_power_frame)
        self.preset_mode_btn.setObjectName(u"preset_mode_btn")
        self.preset_mode_btn.setGeometry(QRect(340, 90, 161, 51))
        self.preset_mode_btn.setFont(font)
        self.func_btn_frame = QFrame(self.centralwidget)
        self.func_btn_frame.setObjectName(u"func_btn_frame")
        self.func_btn_frame.setGeometry(QRect(220, 510, 521, 211))
        self.func_btn_frame.setFrameShape(QFrame.Shape.StyledPanel)
        self.func_btn_frame.setFrameShadow(QFrame.Shadow.Raised)
        self.gridLayoutWidget = QWidget(self.func_btn_frame)
        self.gridLayoutWidget.setObjectName(u"gridLayoutWidget")
        self.gridLayoutWidget.setGeometry(QRect(10, 10, 211, 101))
        self.gridLayout = QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setSpacing(20)
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setContentsMargins(10, 10, 10, 10)
        self.ik_mode_btn = QPushButton(self.gridLayoutWidget)
        self.ik_mode_btn.setObjectName(u"ik_mode_btn")
        self.ik_mode_btn.setEnabled(True)
        sizePolicy2 = QSizePolicy(QSizePolicy.Policy.MinimumExpanding, QSizePolicy.Policy.Preferred)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.ik_mode_btn.sizePolicy().hasHeightForWidth())
        self.ik_mode_btn.setSizePolicy(sizePolicy2)

        self.gridLayout.addWidget(self.ik_mode_btn, 0, 1, 1, 1)

        self.start_pose_iteration_btn = QPushButton(self.gridLayoutWidget)
        self.start_pose_iteration_btn.setObjectName(u"start_pose_iteration_btn")
        self.start_pose_iteration_btn.setEnabled(True)
        sizePolicy2.setHeightForWidth(self.start_pose_iteration_btn.sizePolicy().hasHeightForWidth())
        self.start_pose_iteration_btn.setSizePolicy(sizePolicy2)

        self.gridLayout.addWidget(self.start_pose_iteration_btn, 0, 0, 1, 1)

        self.tr_left_forward_bar = QProgressBar(self.centralwidget)
        self.tr_left_forward_bar.setObjectName(u"tr_left_forward_bar")
        self.tr_left_forward_bar.setGeometry(QRect(50, 360, 51, 75))
        self.tr_left_forward_bar.setValue(24)
        self.tr_left_forward_bar.setTextVisible(False)
        self.tr_left_forward_bar.setOrientation(Qt.Orientation.Vertical)
        self.tr_right_forward_bar = QProgressBar(self.centralwidget)
        self.tr_right_forward_bar.setObjectName(u"tr_right_forward_bar")
        self.tr_right_forward_bar.setGeometry(QRect(110, 360, 51, 75))
        self.tr_right_forward_bar.setValue(24)
        self.tr_right_forward_bar.setTextVisible(False)
        self.tr_right_forward_bar.setOrientation(Qt.Orientation.Vertical)
        self.tr_right_backward_bar = QProgressBar(self.centralwidget)
        self.tr_right_backward_bar.setObjectName(u"tr_right_backward_bar")
        self.tr_right_backward_bar.setGeometry(QRect(110, 430, 51, 75))
        self.tr_right_backward_bar.setValue(24)
        self.tr_right_backward_bar.setTextVisible(False)
        self.tr_right_backward_bar.setOrientation(Qt.Orientation.Vertical)
        self.tr_right_backward_bar.setInvertedAppearance(True)
        self.tr_left_backward_bar = QProgressBar(self.centralwidget)
        self.tr_left_backward_bar.setObjectName(u"tr_left_backward_bar")
        self.tr_left_backward_bar.setGeometry(QRect(50, 430, 51, 75))
        self.tr_left_backward_bar.setMinimum(0)
        self.tr_left_backward_bar.setMaximum(100)
        self.tr_left_backward_bar.setValue(24)
        self.tr_left_backward_bar.setTextVisible(False)
        self.tr_left_backward_bar.setOrientation(Qt.Orientation.Vertical)
        self.tr_left_backward_bar.setInvertedAppearance(True)
        self.label = QLabel(self.centralwidget)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(50, 320, 51, 40))
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_21 = QLabel(self.centralwidget)
        self.label_21.setObjectName(u"label_21")
        self.label_21.setGeometry(QRect(110, 320, 51, 40))
        self.label_21.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.tr_right_status_text = QLabel(self.centralwidget)
        self.tr_right_status_text.setObjectName(u"tr_right_status_text")
        self.tr_right_status_text.setGeometry(QRect(110, 510, 51, 20))
        self.tr_right_status_text.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.tr_left_status_text = QLabel(self.centralwidget)
        self.tr_left_status_text.setObjectName(u"tr_left_status_text")
        self.tr_left_status_text.setGeometry(QRect(50, 510, 51, 20))
        self.tr_left_status_text.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_28 = QLabel(self.centralwidget)
        self.label_28.setObjectName(u"label_28")
        self.label_28.setGeometry(QRect(10, 170, 81, 21))
        self.ping_text = QLabel(self.centralwidget)
        self.ping_text.setObjectName(u"ping_text")
        self.ping_text.setGeometry(QRect(120, 170, 91, 21))
        self.verticalLayoutWidget = QWidget(self.centralwidget)
        self.verticalLayoutWidget.setObjectName(u"verticalLayoutWidget")
        self.verticalLayoutWidget.setGeometry(QRect(760, 20, 641, 701))
        self.arm_graph_layout = QVBoxLayout(self.verticalLayoutWidget)
        self.arm_graph_layout.setObjectName(u"arm_graph_layout")
        self.arm_graph_layout.setSizeConstraint(QLayout.SizeConstraint.SetFixedSize)
        self.arm_graph_layout.setContentsMargins(0, 0, 0, 0)
        self.arm_graph_tool_btn = QPushButton(self.centralwidget)
        self.arm_graph_tool_btn.setObjectName(u"arm_graph_tool_btn")
        self.arm_graph_tool_btn.setGeometry(QRect(1345, 30, 45, 30))
        sizePolicy3 = QSizePolicy(QSizePolicy.Policy.MinimumExpanding, QSizePolicy.Policy.MinimumExpanding)
        sizePolicy3.setHorizontalStretch(0)
        sizePolicy3.setVerticalStretch(0)
        sizePolicy3.setHeightForWidth(self.arm_graph_tool_btn.sizePolicy().hasHeightForWidth())
        self.arm_graph_tool_btn.setSizePolicy(sizePolicy3)
        self.label_12 = QLabel(self.centralwidget)
        self.label_12.setObjectName(u"label_12")
        self.label_12.setGeometry(QRect(10, 200, 92, 21))
        self.label_13 = QLabel(self.centralwidget)
        self.label_13.setObjectName(u"label_13")
        self.label_13.setGeometry(QRect(10, 230, 92, 21))
        self.label_39 = QLabel(self.centralwidget)
        self.label_39.setObjectName(u"label_39")
        self.label_39.setGeometry(QRect(10, 260, 92, 21))
        self.read_rate_text = QLabel(self.centralwidget)
        self.read_rate_text.setObjectName(u"read_rate_text")
        self.read_rate_text.setGeometry(QRect(120, 200, 91, 21))
        self.cmd_rate_text = QLabel(self.centralwidget)
        self.cmd_rate_text.setObjectName(u"cmd_rate_text")
        self.cmd_rate_text.setGeometry(QRect(120, 230, 91, 21))
        self.pump_rate_text = QLabel(self.centralwidget)
        self.pump_rate_text.setObjectName(u"pump_rate_text")
        self.pump_rate_text.setGeometry(QRect(120, 260, 81, 21))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1418, 22))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"Armstrong Control Panel", None))
        self.power_btn.setText("")
        self.pump_power_btn.setText("")
        self.pump_mode_btn.setText("")
        self.act_rpm_text.setText(QCoreApplication.translate("MainWindow", u"-1", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"Feedback RPM", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"RPM", None))
        self.label_26.setText(QCoreApplication.translate("MainWindow", u"RPM", None))
        self.des_rpm_text.setText(QCoreApplication.translate("MainWindow", u"-1", None))
        self.label_27.setText(QCoreApplication.translate("MainWindow", u"Desired RPM", None))
        self.label_29.setText(QCoreApplication.translate("MainWindow", u"A", None))
        self.label_31.setText(QCoreApplication.translate("MainWindow", u"\u00b0C", None))
        self.des_cur_text.setText(QCoreApplication.translate("MainWindow", u"-1", None))
        self.elmo_temp_text.setText(QCoreApplication.translate("MainWindow", u"-1", None))
        self.label_32.setText(QCoreApplication.translate("MainWindow", u"Driver Temp.", None))
        self.label_33.setText(QCoreApplication.translate("MainWindow", u"Active Cur.", None))
        self.label_9.setText(QCoreApplication.translate("MainWindow", u"pump", None))
        self.label_10.setText(QCoreApplication.translate("MainWindow", u"mode", None))
        self.pump_config_btn.setText("")
        self.label_11.setText(QCoreApplication.translate("MainWindow", u"config", None))
        self.left_arm_power_btn.setText(QCoreApplication.translate("MainWindow", u"Left Arm\n"
"OFF", None))
        self.right_arm_power_btn.setText(QCoreApplication.translate("MainWindow", u"Right Arm\n"
"OFF", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"3", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"2", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"1", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"4", None))
        self.l1_power_btn.setText("")
        self.l3_power_btn.setText("")
        self.l2_power_btn.setText("")
        self.l4_power_btn.setText("")
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"6", None))
        self.label_14.setText(QCoreApplication.translate("MainWindow", u"5", None))
        self.l6_power_btn.setText("")
        self.label_15.setText(QCoreApplication.translate("MainWindow", u"8", None))
        self.label_16.setText(QCoreApplication.translate("MainWindow", u"7", None))
        self.l5_power_btn.setText("")
        self.l8_power_btn.setText("")
        self.l7_power_btn.setText("")
        self.label_34.setText(QCoreApplication.translate("MainWindow", u"7", None))
        self.label_18.setText(QCoreApplication.translate("MainWindow", u"5", None))
        self.r7_power_btn.setText("")
        self.r5_power_btn.setText("")
        self.r6_power_btn.setText("")
        self.label_19.setText(QCoreApplication.translate("MainWindow", u"8", None))
        self.r8_power_btn.setText("")
        self.label_17.setText(QCoreApplication.translate("MainWindow", u"6", None))
        self.label_35.setText(QCoreApplication.translate("MainWindow", u"2", None))
        self.label_36.setText(QCoreApplication.translate("MainWindow", u"1", None))
        self.r2_power_btn.setText("")
        self.label_37.setText(QCoreApplication.translate("MainWindow", u"4", None))
        self.label_38.setText(QCoreApplication.translate("MainWindow", u"3", None))
        self.r1_power_btn.setText("")
        self.r4_power_btn.setText("")
        self.r3_power_btn.setText("")
        self.show_state_btn.setText(QCoreApplication.translate("MainWindow", u"Show Robot State", None))
        self.preset_mode_btn.setText(QCoreApplication.translate("MainWindow", u"Enter Preset Mode", None))
        self.ik_mode_btn.setText(QCoreApplication.translate("MainWindow", u"IK\n"
"Mode", None))
        self.start_pose_iteration_btn.setText(QCoreApplication.translate("MainWindow", u"Start\n"
"Pose\n"
"Iteration", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"Track\n"
"Left", None))
        self.label_21.setText(QCoreApplication.translate("MainWindow", u"Track\n"
"Right", None))
        self.tr_right_status_text.setText(QCoreApplication.translate("MainWindow", u"-100%", None))
        self.tr_left_status_text.setText(QCoreApplication.translate("MainWindow", u"-100%", None))
        self.label_28.setText(QCoreApplication.translate("MainWindow", u"ping:", None))
        self.ping_text.setText(QCoreApplication.translate("MainWindow", u"[PING] ms", None))
        self.arm_graph_tool_btn.setText(QCoreApplication.translate("MainWindow", u"...", None))
        self.label_12.setText(QCoreApplication.translate("MainWindow", u"read rate:", None))
        self.label_13.setText(QCoreApplication.translate("MainWindow", u"cmd rate:", None))
        self.label_39.setText(QCoreApplication.translate("MainWindow", u"pump rate:", None))
        self.read_rate_text.setText(QCoreApplication.translate("MainWindow", u"0 Hz", None))
        self.cmd_rate_text.setText(QCoreApplication.translate("MainWindow", u"0 Hz", None))
        self.pump_rate_text.setText(QCoreApplication.translate("MainWindow", u"0 Hz", None))
    # retranslateUi

