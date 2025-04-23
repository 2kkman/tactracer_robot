import sys
import csv
import json
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QFileDialog, QVBoxLayout, QLabel

class FileConverter(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()

        self.label = QLabel('파일 변환기')
        layout.addWidget(self.label)

        btn_csv_to_json = QPushButton('CSV를 JSON으로 변환', self)
        btn_csv_to_json.clicked.connect(self.csv_to_json)
        layout.addWidget(btn_csv_to_json)

        btn_json_to_csv = QPushButton('JSON을 CSV로 변환', self)
        btn_json_to_csv.clicked.connect(self.json_to_csv)
        layout.addWidget(btn_json_to_csv)

        self.setLayout(layout)
        self.setGeometry(300, 300, 300, 150)
        self.setWindowTitle('파일 변환기')
        self.show()

    def csv_to_json(self):
        fname = QFileDialog.getOpenFileName(self, 'CSV 파일 열기', '', 'CSV 파일 (*.csv)')
        if fname[0]:
            with open(fname[0], 'r', newline='', encoding='utf-8') as csvfile:
                reader = csv.DictReader(csvfile)
                data = list(reader)

            save_name = QFileDialog.getSaveFileName(self, 'JSON 파일 저장', '', 'JSON 파일 (*.json)')
            if save_name[0]:
                with open(save_name[0], 'w', encoding='utf-8') as jsonfile:
                    json.dump(data, jsonfile, ensure_ascii=False, indent=4)

                self.label.setText('CSV를 JSON으로 변환 완료!')

    def json_to_csv(self):
        fname = QFileDialog.getOpenFileName(self, 'JSON 파일 열기', '', 'JSON 파일 (*.json)')
        if fname[0]:
            with open(fname[0], 'r', encoding='utf-8') as jsonfile:
                data = json.load(jsonfile)

            save_name = QFileDialog.getSaveFileName(self, 'CSV 파일 저장', '', 'CSV 파일 (*.csv)')
            if save_name[0]:
                with open(save_name[0], 'w', newline='', encoding='utf-8') as csvfile:
                    if data:
                        writer = csv.DictWriter(csvfile, fieldnames=data[0].keys())
                        writer.writeheader()
                        writer.writerows(data)

                self.label.setText('JSON을 CSV로 변환 완료!')

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = FileConverter()
    sys.exit(app.exec_())