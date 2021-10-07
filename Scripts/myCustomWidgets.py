from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QWidget, QSizePolicy, QVBoxLayout, QPushButton, QApplication, QHBoxLayout, QInputDialog, QLineEdit
from PyQt5.QtGui import QColor, QRegion, QPainter, QBrush
from PyQt5.QtCore import Qt, QRect
import time
from Scripts.threads import ClickerBlinkThread
import numpy as np


class AutoClicker(QWidget):
    # transparentColor = QColor(50, 50, 50, 30)
    onColor = QColor(255, 0, 0, 255)
    offColor = QColor(42, 13, 0, 255)

    def __init__(self, parent):
        super().__init__(parent)
        self.color = self.offColor
        self.parent = parent

        self.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
        self.isClicked = False
        self.leds_active = False
        self.score = 0

    def sizeHint(self):
        return QtCore.QSize(60, 120)

    def mouseMoveEvent(self, e):
        # click and drag
        pass

    def mousePressEvent(self, e):
        # click
        if self.leds_active:
            self.setColor(self.offColor)
            self.leds_active = False
            self.score += 1

    def setColor(self, color):
        self.color = color
        self.update()

    def paintEvent(self, e):
        width, height = self.parent.frameGeometry().width(), self.parent.frameGeometry().height()
        region = QRegion(QRect(40, 30, int(width/2), int(height/2)), QRegion.Ellipse)
        self.setMask(region)

        bg_color = self.color
        brush = QBrush()
        brush.setStyle(Qt.SolidPattern)
        brush.setColor(QColor(bg_color))

        painter = QPainter(self)
        rect = QtCore.QRect(0, 0, painter.device().width(), painter.device().height())
        painter.fillRect(rect, brush)


class AutoClickerAnimation(QWidget):
    blink_speed = 16
    on_time = 2  # sec
    blink_time = 1  # sec

    def __init__(self, ind):
        super(AutoClickerAnimation, self).__init__()

        self.autoclicker = AutoClicker(self)
        layout = QVBoxLayout()
        layout.addWidget(self.autoclicker)
        self.setLayout(layout)

        self.ind = ind

        self.resetBit = False

        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)

        self.numBlinkLevels = 8
        self.blinkState = self.numBlinkLevels - 1
        minLevel, maxLevel = 30, 255
        step = ((maxLevel - minLevel) / (self.numBlinkLevels - 1)) - 1
        self.alphaLevels = np.arange(30, 255, int(step), dtype=np.int32)

    def activateLEDs(self):
        self.autoclicker.leds_active = True

        self.blinkState = self.numBlinkLevels - 1
        color = self.autoclicker.onColor
        color.setAlpha(self.alphaLevels[self.blinkState])
        self.autoclicker.setColor(color)
        time.sleep(self.on_time - self.blink_time)
        self.blink()

    def blink(self):
        dt = 1/self.blink_speed
        list = range(int(self.blink_time * self.blink_speed))

        for i in list:
            if self.resetBit:
                self.resetBit = False
                break
            if not self.autoclicker.leds_active:
                break

            time.sleep(dt)

            self.blinkState = (self.blinkState - 1) % self.numBlinkLevels

            color = self.autoclicker.onColor
            color.setAlpha(self.alphaLevels[self.blinkState])

            if not self.resetBit and self.autoclicker.leds_active:
                self.autoclicker.setColor(color)
        self.autoclicker.setColor(self.autoclicker.offColor)
        self.autoclicker.leds_active = False


    def reset(self):
        self.resetBit = True
        self.autoclicker.score = 0

        self.ind = 0
        self.blinkState = self.numBlinkLevels - 1


class ChangeUserList(QWidget):
    def __init__(self, win, database):
        super(ChangeUserList, self).__init__()
        self.win = win
        self.database = database
        self.listWidget = QtWidgets.QListWidget()
        self.layout = QHBoxLayout()

        self.createList()

    def createList(self):
        # create layout
        self.listWidget.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.listWidget.setGeometry(QtCore.QRect(10, 10, 211, 291))

        # fetch player names and populate list
        rows = self.database.load_all()
        for player in rows:
            firstName = player[0]
            lastName = player[1]
            item = QtWidgets.QListWidgetItem(f"{firstName} {lastName}")
            self.listWidget.addItem(item)

        # on player choice - load player
        self.listWidget.itemClicked.connect(self.loadPlayerFromList)
        self.layout.addWidget(self.listWidget)

        button_new_player = QPushButton()
        button_new_player.setText("Add new")
        button_new_player.clicked.connect(self.addPlayer)

        self.layout.addWidget(button_new_player)
        self.setLayout(self.layout)

    def addPlayer(self):
        print("Adding player")

        def getAge():
            i, okPressed = QInputDialog.getInt(self, "Player age", "Your age:", 40, 18, 100, 1)
            if okPressed:
                return i

        def getName(type="first"):
            text, okPressed = QInputDialog.getText(self, "Player name", f"Your {type} name:", QLineEdit.Normal, "")
            if okPressed and text != '':
                return text

        playerFirstName = getName("first")
        playerLastName = getName("last")
        playerAge = getAge()

        self.database.insert(playerFirstName, playerLastName, playerAge)
        self.loadPlayer(playerFirstName, playerLastName)

    def loadPlayer(self, firstName, lastName):
        self.database.load(firstName, lastName)

        # close player menu
        self.win.active_screen.show()
        self.close()

    def loadPlayerFromList(self):
        chosen_players = self.listWidget.selectedItems()
        x = str(chosen_players[0].text())
        name = x.split(" ")
        firstName = name[0]
        lasttName = name[1]
        self.loadPlayer(firstName, lasttName)



