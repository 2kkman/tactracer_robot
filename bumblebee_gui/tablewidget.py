import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *


class DlgMain(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("My GUI")
        self.date = QDate.currentDate()
        
        spinfont = QFont('Arial', 115)
        labelfont = QFont('Noto Sans Mono CJK KR', 20)
        
        ### Create Widgets
        self.tabMain = QTabWidget()
        # self.tabMain.setTabPosition(QTabWidget.West)
        self.tabMain.setStyleSheet("""
                                    QTabWidget::tab-bar {
                                        alignment: left;
                                    }
                                    QTabBar::tab {
                                        width: 150px;
                                        height: 50px;
                                        font-size: 24px; 
                                        font-family: Noto Sans Mono CJK KR;
                                    }
                                    QTabBar::tab:selected, QTabBar::tab:hover{ 
                                        background-color: #FDDE45;
                                    }
                                    QTabBar::close-button {
                                        image: none;
                                    }
                                    """)

        ####  Create Widgets
        self.cmbSelector = QComboBox()
        self.cmbSelector.addItems(["General", "Species", "Location", "Surveys"])
        self.cmbSelector.currentIndexChanged.connect(self.evt_cmbSelector_changed)

        self.wdgGeneral = QWidget()
        self.wdgSpecies = QWidget()
        self.wdgLocation = QWidget()
        self.wdgSurveys = QWidget()
        
        # 숫자만 입력 가능하도록 제한
        regex = QRegExp("[0-9]*")
        validator = QRegExpValidator(regex)
        
        ######  General Widgets
        self.lblNest = QLabel("상 호 명 :")
        self.lblNest.setFont(labelfont)        
        self.lblNestID = QLabel("주식회사 택트레이서")
        self.lblNestID.setFont(labelfont)
        
        self.lblAddress = QLabel("주    소 :")
        self.lblAddress.setFont(labelfont)        
        self.lblAddressID = QLabel("경기도 안산시 한양대학로 55 창업보육센터 211호")
        self.lblAddressID.setFont(labelfont)   
             
        self.lblTable = QLabel("테 이 블 :")
        self.lblTable.setFont(labelfont)       
        self.ledTableMax = QLineEdit()
        self.ledTableMax.setReadOnly(True)
        self.ledTableMax.setMaxLength(3) 
        self.ledTableMax.setValidator(validator)
        self.ledTableMax.setAlignment(Qt.AlignCenter)
        self.ledTableMax.setText("2")
        self.ledTableMax.setFixedWidth(100)     
        
        self.ledTableMax.setStyleSheet("QLineEdit { border: none; }")
        self.ledTableMax.setFont(labelfont)
        
        self.lblTableUnit = QLabel("개")
        self.lblTableUnit.setFont(labelfont)  
        self.lblTableUnit.setFixedWidth(50)     
         
        self.dteFound = QDateTimeEdit(QDate(2016, 5, 13))
        self.dteFound.setFixedWidth(200)
        self.dteFound.setAlignment(Qt.AlignCenter)
        self.dteFound.setEnabled(False)
        
        self.dteLast = QDateTimeEdit(QDate(2020, 4, 14))
        self.dteLast.setFixedWidth(200)
        self.dteLast.setAlignment(Qt.AlignCenter)
        self.dteLast.setEnabled(False)
        
        self.btnEdit = QPushButton("수 정")
        self.btnEdit.setStyleSheet(""" 
                                    QPushButton {
                                        background-color: #FDDE45;
                                        border-style: outset;
                                        border-width: 2px;
                                        border-radius: 10px;
                                        border-color: beige;
                                        min-width: 10em;
                                        height: 50px;
                                        padding: 6px; 
                                        font-size: 16px; 
                                        font-family: Noto Sans Mono CJK KR;
                                        } """)
        self.btnEdit.clicked.connect(self.evt_btnEdit_clicked)
        
        self.btnConfirm = QPushButton("저 장")
        self.btnConfirm.setStyleSheet(""" 
                                    QPushButton {
                                        background-color: #FDDE45;
                                        border-style: outset;
                                        border-width: 2px;
                                        border-radius: 10px;
                                        border-color: beige;
                                        min-width: 10em;
                                        height: 50px;
                                        padding: 6px; 
                                        font-size: 16px; 
                                        font-family: Noto Sans Mono CJK KR;
                                        } """)
        self.btnConfirm.clicked.connect(self.evt_btnConfirm_clicked)
        self.btnConfirm.setEnabled(False)
        self.chkActive = QCheckBox()

        ########  Species widgets
        self.cmbSpecies = QComboBox()
        self.cmbSpecies.addItem("Red-tailed Hawk", 800)
        self.cmbSpecies.addItem("Swainsons Hawk", 400)
        self.cmbSpecies.addItem("Other", 1600)
        
        self.ledSpecies = QLineEdit()
        self.spbBuffer = QSpinBox()
        self.spbBuffer.setValue(800)

        ###########  Location Widget
        self.spbLatitude = QDoubleSpinBox()
        self.spbLongitude = QDoubleSpinBox()

        ###########   Survey Widget
        self.lstSurveys = QListWidget()
        self.lstSurveys.addItem("03/24/2020 - MSM - INACTIVE")
        self.lstSurveys.addItem("03/30/2020 - MSM - INACTIVE")
        self.lstSurveys.addItem("04/07/2020 - MSM - INACTIVE")
        self.lstSurveys.addItem("04/14/2020 - MSM - ACTIVE!!")
        self.btnAddSurvey = QPushButton("Add Survey")     
           
        # self.setStyleSheet("Background-color: #FFFAFA;")
        self.setGeometry(0, 0, 1200, 400)
        
        self.setupLayout()

    def setupLayout(self):
        self.lytMain = QHBoxLayout()
        self.lytLeft = QVBoxLayout()

        # self.lytMain.addLayout(self.lytLeft)
        self.lytMain.addWidget(self.tabMain)

        ######  Add Widgets to leftLayout
        self.lytLeft.addWidget(self.cmbSelector)
        self.lytLeft.addStretch()

        #####   Add Stacked Widgets to rightLayout
        self.tabMain.addTab(self.wdgGeneral, "일반")
        self.tabMain.addTab(self.wdgSpecies, "Species")
        self.tabMain.addTab(self.wdgLocation, "Location")
        self.tabMain.addTab(self.wdgSurveys, "Surveys")

        ##### Setup General Widget
        self.lytGeneralbtn = QHBoxLayout()
        self.lytGeneralbtn.addStretch()
        self.lytGeneralbtn.addWidget(self.btnEdit)
        self.lytGeneralbtn.addWidget(self.btnConfirm)
        
        self.lytGeneralempty = QVBoxLayout()
        self.lytGeneralempty.addStretch()
        self.lytGeneralempty.addWidget(QLabel(" "))
        
        self.lytGeneralDate = QHBoxLayout()
        self.lytGeneralDate.addWidget(QLabel("Date Found:"))
        self.lytGeneralDate.addWidget(self.dteFound)
        self.lytGeneralDate.addStretch(1)
        self.lytGeneralDate.addWidget(QLabel("Last Surveyed:"))
        self.lytGeneralDate.addWidget(self.dteLast)
        self.lytGeneralDate.addStretch(2)
        
        self.lytGeneralNest = QHBoxLayout()
        self.lytGeneralNest.addWidget(self.lblNest)
        self.lytGeneralNest.addWidget(self.lblNestID)        
        self.lytGeneralNest.addStretch()

        self.lytGeneralTable = QHBoxLayout()
        self.lytGeneralTable.addWidget(self.lblTable)
        self.lytGeneralTable.addWidget(self.ledTableMax)
        self.lytGeneralTable.addWidget(self.lblTableUnit)
        
        self.lytGeneralTable.addStretch(2)

        self.lytGeneralAddress = QHBoxLayout()
        self.lytGeneralAddress.addWidget(self.lblAddress)
        self.lytGeneralAddress.addWidget(self.lblAddressID)
        self.lytGeneralAddress.addStretch()        
        
        self.lytGeneral = QFormLayout()
        self.lytGeneral.addRow(self.lytGeneralNest)
        self.lytGeneral.addRow(self.lytGeneralAddress)
        self.lytGeneral.addRow(self.lytGeneralTable)
        self.lytGeneral.addRow(self.lytGeneralDate)
        self.lytGeneral.addRow(self.lytGeneralempty)
        self.lytGeneral.addRow(self.lytGeneralbtn)
        self.wdgGeneral.setLayout(self.lytGeneral)
        self.wdgGeneral.setStyleSheet("Background-color: #EFE8D9;")

        ##### Setup Species Widget
        self.lytSpecies = QFormLayout()
        self.lytSpecies.addRow("Species:", self.cmbSpecies)
        self.lytSpecies.addRow("Species:", self.ledSpecies)
        self.lytSpecies.addRow("Buffer:", self.spbBuffer)
        self.spbBuffer.setSuffix(" m")
        self.wdgSpecies.setLayout(self.lytSpecies)

        ##### Setup Location Widget
        self.lytLocation = QFormLayout()
        self.lytLocation.addRow("Latitude:", self.spbLatitude)
        self.lytLocation.addRow("Longitude:", self.spbLongitude)
        self.wdgLocation.setLayout(self.lytLocation)

        ##### Setup Surveys Widget
        self.lytSurveys = QVBoxLayout()
        self.lytSurveys.addWidget(self.lstSurveys)
        self.lytSurveys.addWidget(self.btnAddSurvey)
        self.wdgSurveys.setLayout(self.lytSurveys)
        

        self.setLayout(self.lytMain)
    def evt_cmbSelector_changed(self, idx):
        self.tabMain.setCurrentIndex(idx)
        
    def evt_btnEdit_clicked(self):
        self.msgEdit = QMessageBox()
        self.msgEdit.setText("수정모드로 변경합니다")
        self.msgEdit.setWindowTitle("수정모드")
        self.msgEdit.setStandardButtons(QMessageBox.Ok)
        self.msgEdit.setStyleSheet("""
                                    QMessageBox{
                                        background-color: #FDDE45; 
                                        font-size: 24px;
                                        width: 500px;
                                        height: 200px;} 
                                    QPushButton {
                                        font-size: 20px;
                                        width: 100px;
                                        }
                                   """) 
           
        bOK = self.msgEdit.exec_()
        if bOK == QMessageBox.Ok:
            self.ledTableMax.setPlaceholderText("최대 테이블 3자리 숫자만 입력하세요.")
            self.ledTableMax.setReadOnly(False)
            self.ledTableMax.setStyleSheet("QLineEdit { border: 1px solid black; }")
            self.btnConfirm.setEnabled(True)
            self.btnEdit.setEnabled(False)
            
            
    def evt_btnConfirm_clicked(self):
        self.msgEdit = QMessageBox()
        self.msgEdit.setText("저장하시겠습니까?")
        self.msgEdit.setWindowTitle("저 장")
        self.msgEdit.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        self.msgEdit.setStyleSheet("""
                                    QMessageBox{
                                        background-color: #FDDE45; 
                                        font-size: 24px;
                                        width: 500px;
                                        height: 200px;} 
                                    QPushButton {
                                        font-size: 20px;
                                        width: 100px;}
                                   """)    
        bOK = self.msgEdit.exec_()
        if bOK == QMessageBox.Ok:
            self.ledTableMax.setReadOnly(True)
            self.ledTableMax.setStyleSheet("QLineEdit { border: none; }")
            self.btnConfirm.setEnabled(False)
            self.btnEdit.setEnabled(True)

                
if __name__ == "__main__":
    app = QApplication(sys.argv)
    dlgMain = DlgMain()
    dlgMain.showFullScreen()
    sys.exit(app.exec_())
