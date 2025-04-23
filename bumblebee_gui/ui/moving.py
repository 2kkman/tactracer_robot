# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'moving.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(1280, 400)
        Form.setStyleSheet("background-color: #FFFFFF")
        self.table_label = QtWidgets.QLabel(Form)
        self.table_label.setGeometry(QtCore.QRect(1, 0, 1280, 400))
        font = QtGui.QFont()
        font.setPointSize(48)
        font.setBold(True)
        font.setWeight(75)
        self.table_label.setFont(font)
        self.table_label.setStyleSheet("QLabel {background-color: #FDDE45;border-style: outset; border-width: 2px; border-radius: 10px;border-color: #FDDE45;min-width: 5em;padding: 1px; }")
        self.table_label.setTextFormat(QtCore.Qt.AutoText)
        self.table_label.setScaledContents(True)
        self.table_label.setAlignment(QtCore.Qt.AlignCenter)
        self.table_label.setObjectName("table_label")
        self.btnTable = QtWidgets.QPushButton(Form)
        self.btnTable.setGeometry(QtCore.QRect(1150, 50, 99, 30))
        self.btnTable.setStyleSheet("QPushButton {background-color: #FDDE45;border-style: outset; border-width: 2px; border-radius: 10px;border-color: beige;min-width: 5em;padding: 5px; }")
        self.btnTable.setObjectName("btnTable")
        self.btnConfirm = QtWidgets.QPushButton(Form)
        self.btnConfirm.setGeometry(QtCore.QRect(1150, 90, 99, 30))
        self.btnConfirm.setStyleSheet("QPushButton {background-color: #FDDE45;border-style: outset; border-width: 2px; border-radius: 10px;border-color: beige;min-width: 5em;padding: 5px; }\n"
                                      "QPushButton:hover {background-color: #F9B93D;}")
        self.btnConfirm.setFlat(False)
        self.btnConfirm.setObjectName("btnConfirm")
        self.lblImage = QtWidgets.QLabel(Form)
        self.lblImage.setEnabled(True)
        self.lblImage.setGeometry(QtCore.QRect(100, 50, 161, 261))
        self.lblImage.setWhatsThis("")
        self.lblImage.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.lblImage.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.lblImage.setFrameShadow(QtWidgets.QFrame.Plain)
        self.lblImage.setText("")
        self.lblImage.setPixmap(QtGui.QPixmap(":/res/movingbee.png"))
        self.lblImage.setScaledContents(True)
        self.lblImage.setAlignment(QtCore.Qt.AlignCenter)
        self.lblImage.setObjectName("lblImage")

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.table_label.setText(_translate("Form", "Moving"))
        self.btnTable.setText(_translate("Form", "flagTable"))
        self.btnConfirm.setText(_translate("Form", "Confirm"))
import ui.main_rc


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())
