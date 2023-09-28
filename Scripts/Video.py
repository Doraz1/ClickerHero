import threading
import time

from PyQt5.QtGui import QIcon, QFont
from PyQt5.QtCore import QDir, Qt, QUrl, QSize
from PyQt5.QtMultimedia import QMediaContent, QMediaPlayer
from PyQt5.QtMultimediaWidgets import QVideoWidget
from PyQt5.QtWidgets import (QApplication, QFileDialog, QHBoxLayout, QLabel,
        QPushButton, QSizePolicy, QSlider, QStyle, QVBoxLayout, QWidget, QStatusBar)

class VideoPlayer(QWidget):

    def __init__(self, parent=None):
        super(VideoPlayer, self).__init__(parent)
        self.mediaPlayer = QMediaPlayer(None, QMediaPlayer.VideoSurface)

        btnSize = QSize(16, 16)
        videoWidget = QVideoWidget()
        self.isActive = True
        # closeButton = QPushButton("Close Video")
        # closeButton.setFixedHeight(50)
        # closeButton.setIconSize(btnSize)


        # closeButton.setStatusTip("Close Video Screen")
        # closeButton.setToolTip("Close Video Screen")

        # pixmapi = QStyle.SP_MessageBoxCritical
        # icon = self.style().standardIcon(pixmapi)
        # closeButton.setIcon(icon)
        # closeButton.setIcon(self.style().standardIcon(QStyle.SP_DialogCloseButton))


        self.playButton = QPushButton()
        self.playButton.setEnabled(False)
        self.playButton.setFixedHeight(24)
        self.playButton.setIconSize(btnSize)
        self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        self.playButton.clicked.connect(self.play)

        self.closeButton = QPushButton()
        self.closeButton.setText("X")
        self.closeButton.setFont(QFont("Noto Sans", 15))

        close_btn_bg = (255, 0, 0, 1)
        self.closeButton.setStyleSheet(f"background-color: rgba{close_btn_bg}; "
                          "border-radius: 10;"
                          "border: 1px solid black;"
                          "text-align: center;"
                          "padding: 3px;"
                          "margin: 2px;")


        self.closeButton.clicked.connect(self.closeBtnLogic)
        self.positionSlider = QSlider(Qt.Horizontal)
        self.positionSlider.setRange(0, 0)
        self.positionSlider.sliderMoved.connect(self.setPosition)

        self.statusBar = QStatusBar()
        self.statusBar.setFont(QFont("Noto Sans", 10))

        controlLayout = QHBoxLayout()
        # controlLayout.setContentsMargins(0, 0, 0, 0)
        controlLayout.addWidget(self.playButton)
        controlLayout.addWidget(self.positionSlider)
        # controlLayout.addWidget(self.statusBar)
        controlLayout.addWidget(self.closeButton)

        layout = QVBoxLayout()
        # self.setGeometry(-20, -20, 300 ,200)
        layout.addLayout(controlLayout)
        layout.addWidget(videoWidget)

        self.setLayout(layout)

        self.mediaPlayer.setVideoOutput(videoWidget)
        self.mediaPlayer.stateChanged.connect(self.mediaStateChanged)
        self.mediaPlayer.positionChanged.connect(self.positionChanged)
        self.mediaPlayer.durationChanged.connect(self.durationChanged)
        self.mediaPlayer.error.connect(self.handleError)
        self.statusBar.showMessage("Ready")

    def load(self, fileName):
        if fileName != '':
            self.mediaPlayer.setMedia(
                    QMediaContent(QUrl.fromLocalFile(fileName)))
            self.playButton.setEnabled(True)
            self.statusBar.showMessage(fileName)

    def play(self, path: str, firstRun: bool):
        self.load(path)

        if self.mediaPlayer.state() == QMediaPlayer.PlayingState:
            self.mediaPlayer.pause()
        else:
            self.mediaPlayer.play()

        self.show()
        if not firstRun:
            self.isActive = True
            while self.isActive:
                # print(f"Sleeping since media status = {self.isActive}")
                time.sleep(0.1)
        else:
            self.mediaPlayer.stop()
            self.isActive = False

        print("Done!")
        self.close()

    def mediaStateChanged(self, state):
        self.isActive = (state == 1) and (not state == 0)
        # print(f"media state changed: {state} | isactive = {self.isActive}")

        if self.mediaPlayer.state() == QMediaPlayer.PlayingState:
            self.playButton.setIcon(
                    self.style().standardIcon(QStyle.SP_MediaPause))
        else:
            self.playButton.setIcon(
                    self.style().standardIcon(QStyle.SP_MediaPlay))

    def positionChanged(self, position):
        self.positionSlider.setValue(position)

    def durationChanged(self, duration):
        self.positionSlider.setRange(0, duration)

    def setPosition(self, position):
        self.mediaPlayer.setPosition(position)

    def handleError(self):
        self.playButton.setEnabled(False)
        self.statusBar.showMessage("Error: " + self.mediaPlayer.errorString())

    def closeBtnLogic(self):
        self.mediaPlayer.stop()
        self.isActive = False

if __name__ == '__main__':
    import sys
    app = QApplication(sys.argv)
    path = r'/home/clickerhero/Game/Assets/Videos/test_vid2.mp4'
    player = VideoPlayer()
    player.setWindowTitle("Player")
    player.resize(600, 400)

    t = threading.Thread(target=player.play, args=(path,))
    t.start()
    player.show()
    sys.exit(app.exec_())
