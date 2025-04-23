from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent
from PyQt5.QtCore import QUrl

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        self.player = QMediaPlayer(self)
        self.list = ['/root/catkin_ws/src/bumblebee_gui/audio1.mp3', 'audio2.mp3', 'audio3.mp3']

        self.play_button1 = QPushButton('Play 1', self)
        self.play_button1.clicked.connect(lambda: self.play_audio(self.list[0]))
        self.play_button1.show() # Hide the button

        self.setGeometry(100, 100, 400, 300)
        # self.play_audio('audio1.mp3') # Play audio automatically on startup

    def play_audio(self, file_name):
        if self.player.state() == QMediaPlayer.PlayingState:
            self.player.stop()

        self.player.setMedia(QMediaContent(QUrl.fromLocalFile(file_name)))
        self.player.play()

if __name__ == '__main__':
    app = QApplication([])
    window = MainWindow()
    window.show()
    app.exec_()