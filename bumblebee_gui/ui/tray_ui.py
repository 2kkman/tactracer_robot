# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/root/ui/tray/tray.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1100, 350)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.groupBox_2 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_2.setGeometry(QtCore.QRect(550, 20, 151, 291))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.groupBox_2.sizePolicy().hasHeightForWidth())
        self.groupBox_2.setSizePolicy(sizePolicy)
        self.groupBox_2.setObjectName("groupBox_2")
        self.verticalLayoutWidget_3 = QtWidgets.QWidget(self.groupBox_2)
        self.verticalLayoutWidget_3.setGeometry(QtCore.QRect(0, 20, 151, 271))
        self.verticalLayoutWidget_3.setObjectName("verticalLayoutWidget_3")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_3)
        self.verticalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.btn_degree23 = QtWidgets.QPushButton(self.verticalLayoutWidget_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btn_degree23.sizePolicy().hasHeightForWidth())
        self.btn_degree23.setSizePolicy(sizePolicy)
        self.btn_degree23.setObjectName("btn_degree23")
        self.verticalLayout_5.addWidget(self.btn_degree23)
        self.btn_degree30 = QtWidgets.QPushButton(self.verticalLayoutWidget_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btn_degree30.sizePolicy().hasHeightForWidth())
        self.btn_degree30.setSizePolicy(sizePolicy)
        self.btn_degree30.setObjectName("btn_degree30")
        self.verticalLayout_5.addWidget(self.btn_degree30)
        self.btn_degree45 = QtWidgets.QPushButton(self.verticalLayoutWidget_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btn_degree45.sizePolicy().hasHeightForWidth())
        self.btn_degree45.setSizePolicy(sizePolicy)
        self.btn_degree45.setObjectName("btn_degree45")
        self.verticalLayout_5.addWidget(self.btn_degree45)
        self.btn_degree90 = QtWidgets.QPushButton(self.verticalLayoutWidget_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btn_degree90.sizePolicy().hasHeightForWidth())
        self.btn_degree90.setSizePolicy(sizePolicy)
        self.btn_degree90.setObjectName("btn_degree90")
        self.verticalLayout_5.addWidget(self.btn_degree90)
        self.btn_degree120 = QtWidgets.QPushButton(self.verticalLayoutWidget_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btn_degree120.sizePolicy().hasHeightForWidth())
        self.btn_degree120.setSizePolicy(sizePolicy)
        self.btn_degree120.setObjectName("btn_degree120")
        self.verticalLayout_5.addWidget(self.btn_degree120)
        self.groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QtCore.QRect(360, 19, 151, 291))
        self.groupBox.setFlat(False)
        self.groupBox.setObjectName("groupBox")
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self.groupBox)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(0, 20, 151, 271))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_4.setSizeConstraint(QtWidgets.QLayout.SetMaximumSize)
        self.verticalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.btn_up = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btn_up.sizePolicy().hasHeightForWidth())
        self.btn_up.setSizePolicy(sizePolicy)
        self.btn_up.setObjectName("btn_up")
        self.verticalLayout_4.addWidget(self.btn_up)
        self.btn_stop = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btn_stop.sizePolicy().hasHeightForWidth())
        self.btn_stop.setSizePolicy(sizePolicy)
        self.btn_stop.setObjectName("btn_stop")
        self.verticalLayout_4.addWidget(self.btn_stop)
        self.btn_down = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btn_down.sizePolicy().hasHeightForWidth())
        self.btn_down.setSizePolicy(sizePolicy)
        self.btn_down.setObjectName("btn_down")
        self.verticalLayout_4.addWidget(self.btn_down)
        self.groupBox_3 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_3.setGeometry(QtCore.QRect(30, 20, 291, 291))
        self.groupBox_3.setObjectName("groupBox_3")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.groupBox_3)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(0, 20, 291, 271))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.label = QtWidgets.QLabel(self.verticalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.horizontalLayout_7.addWidget(self.label)
        self.label_2 = QtWidgets.QLabel(self.verticalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_2.sizePolicy().hasHeightForWidth())
        self.label_2.setSizePolicy(sizePolicy)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_7.addWidget(self.label_2)
        self.label_3 = QtWidgets.QLabel(self.verticalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_3.sizePolicy().hasHeightForWidth())
        self.label_3.setSizePolicy(sizePolicy)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_7.addWidget(self.label_3)
        self.verticalLayout_3.addLayout(self.horizontalLayout_7)
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.cb_cmd = QtWidgets.QComboBox(self.verticalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.cb_cmd.sizePolicy().hasHeightForWidth())
        self.cb_cmd.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(24)
        self.cb_cmd.setFont(font)
        self.cb_cmd.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.cb_cmd.setAutoFillBackground(True)
        self.cb_cmd.setObjectName("cb_cmd")
        self.cb_cmd.addItem("")
        self.cb_cmd.addItem("")
        self.horizontalLayout_6.addWidget(self.cb_cmd)
        self.cb_name = QtWidgets.QComboBox(self.verticalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.cb_name.sizePolicy().hasHeightForWidth())
        self.cb_name.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(24)
        self.cb_name.setFont(font)
        self.cb_name.setAutoFillBackground(True)
        self.cb_name.setObjectName("cb_name")
        self.cb_name.addItem("")
        self.horizontalLayout_6.addWidget(self.cb_name)
        self.cb_value = QtWidgets.QComboBox(self.verticalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.cb_value.sizePolicy().hasHeightForWidth())
        self.cb_value.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(24)
        self.cb_value.setFont(font)
        self.cb_value.setAutoFillBackground(True)
        self.cb_value.setObjectName("cb_value")
        self.cb_value.addItem("")
        self.cb_value.addItem("")
        self.cb_value.addItem("")
        self.cb_value.addItem("")
        self.cb_value.addItem("")
        self.horizontalLayout_6.addWidget(self.cb_value)
        self.verticalLayout_3.addLayout(self.horizontalLayout_6)
        self.btn_call = QtWidgets.QPushButton(self.verticalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btn_call.sizePolicy().hasHeightForWidth())
        self.btn_call.setSizePolicy(sizePolicy)
        self.btn_call.setObjectName("btn_call")
        self.verticalLayout_3.addWidget(self.btn_call)
        self.verticalLayout_3.setStretch(0, 1)
        self.verticalLayout_3.setStretch(1, 2)
        self.verticalLayout_3.setStretch(2, 1)
        self.groupBox_4 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_4.setGeometry(QtCore.QRect(740, 20, 291, 291))
        self.groupBox_4.setObjectName("groupBox_4")
        self.verticalLayoutWidget_4 = QtWidgets.QWidget(self.groupBox_4)
        self.verticalLayoutWidget_4.setGeometry(QtCore.QRect(0, 20, 291, 271))
        self.verticalLayoutWidget_4.setObjectName("verticalLayoutWidget_4")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_4)
        self.verticalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.textEdit = QtWidgets.QTextEdit(self.verticalLayoutWidget_4)
        self.textEdit.setObjectName("textEdit")
        self.verticalLayout_6.addWidget(self.textEdit)
        self.btn_send = QtWidgets.QPushButton(self.verticalLayoutWidget_4)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btn_send.sizePolicy().hasHeightForWidth())
        self.btn_send.setSizePolicy(sizePolicy)
        self.btn_send.setObjectName("btn_send")
        self.verticalLayout_6.addWidget(self.btn_send)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.groupBox_2.setTitle(_translate("MainWindow", "EYES"))
        self.btn_degree23.setText(_translate("MainWindow", "23°"))
        self.btn_degree30.setText(_translate("MainWindow", "30°"))
        self.btn_degree45.setText(_translate("MainWindow", "45°"))
        self.btn_degree90.setText(_translate("MainWindow", "90°"))
        self.btn_degree120.setText(_translate("MainWindow", "120°"))
        self.groupBox.setTitle(_translate("MainWindow", "DOOR"))
        self.btn_up.setText(_translate("MainWindow", "UP"))
        self.btn_stop.setText(_translate("MainWindow", "STOP"))
        self.btn_down.setText(_translate("MainWindow", "DOWN"))
        self.groupBox_3.setTitle(_translate("MainWindow", "COMMAND"))
        self.label.setText(_translate("MainWindow", "CMD"))
        self.label_2.setText(_translate("MainWindow", "NAME"))
        self.label_3.setText(_translate("MainWindow", "VALUE"))
        self.cb_cmd.setCurrentText(_translate("MainWindow", "S"))
        self.cb_cmd.setItemText(0, _translate("MainWindow", "S"))
        self.cb_cmd.setItemText(1, _translate("MainWindow", "O"))
        self.cb_name.setItemText(0, _translate("MainWindow", "4"))
        self.cb_value.setItemText(0, _translate("MainWindow", "23"))
        self.cb_value.setItemText(1, _translate("MainWindow", "30"))
        self.cb_value.setItemText(2, _translate("MainWindow", "45"))
        self.cb_value.setItemText(3, _translate("MainWindow", "90"))
        self.cb_value.setItemText(4, _translate("MainWindow", "120"))
        self.btn_call.setText(_translate("MainWindow", "CALL"))
        self.groupBox_4.setTitle(_translate("MainWindow", "TTS"))
        self.textEdit.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.btn_send.setText(_translate("MainWindow", "SEND"))
