import sys
from PyQt5.QtWidgets import QApplication, QDialog, QGridLayout, QPushButton, QLineEdit
from PyQt5.QtCore import QRegExp, Qt
from PyQt5.QtGui import QRegExpValidator

class KeypadDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("테이블 번호")
        self.layout = QGridLayout()
        self.lineEdit = QLineEdit()
        self.lineEdit.setStyleSheet("""
                                    QLineEdit {
                                        background-color: beige;
                                        border-style: outset;
                                        border-width: 2px;
                                        border-radius: 20px;
                                        border-color: beige;
                                        min-width: 5em;
                                        height: 50px;
                                        padding: 6px; 
                                        font-size: 36px; 
                                        font-family: Noto Sans Mono CJK KR;
                                        } """)
        self.lineEdit.setMaxLength(3)
        self.lineEdit.setAlignment(Qt.AlignCenter)
        self.lineEdit.setFocusPolicy(Qt.NoFocus)
        regex = QRegExp("[0-9]*")
        validator = QRegExpValidator(regex)
        self.lineEdit.setValidator(validator)
        
        self.layout.addWidget(self.lineEdit, 0, 0, 1, 3)
        self.setGeometry(900, 120, 100, 100)
        
        buttons = [
            '1', '2', '3',
            '4', '5', '6',
            '7', '8', '9',
            'Clear', '0', 'Enter'
        ]

        positions = [(i, j) for i in range(1, 5) for j in range(3)]

        for position, button_label in zip(positions, buttons):
            button = QPushButton(button_label)
            button.setStyleSheet("""
                        QPushButton {
                            border-radius: 10px;
                            background-color: #FDDE45;
                            min-width: 1em;
                            padding: 5px;
                            font-size: 16px;
                            } 
                        QPushButton:hover {
                            background-color: rgb(58, 134, 255);
                            }
                        """)
            button.setFixedHeight(40)
            button.clicked.connect(lambda checked, label=button_label: self.onButtonClick(label))
            self.layout.addWidget(button, *position)

        self.setLayout(self.layout)
        
    def onButtonClick(self, label):
        if label == 'Clear':
            self.lineEdit.clear()
        elif label == 'Enter':
            text = self.lineEdit.text()
            if text.isdigit():
                self.accept()
        else:
            self.lineEdit.insert(label)