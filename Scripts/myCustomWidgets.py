from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QWidget, QSizePolicy, QVBoxLayout, QPushButton, QApplication, QHBoxLayout, QInputDialog, QLineEdit
from PyQt5.QtGui import QColor, QRegion, QPainter, QBrush
from PyQt5.QtCore import Qt, QRect
import time
from Scripts.threads import ClickerBlinkThread
import numpy as np

class AutoClicker(QWidget):
    transparentColor = QColor(50, 50, 50, 30)

    def __init__(self, parent):
        super().__init__(parent)
        self.color = 'black'
        self.parent = parent
        self.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)

    def sizeHint(self):
        return QtCore.QSize(60, 120)

    def mouseMoveEvent(self, e):
        # click and drag
        pass

    def mousePressEvent(self, e):
        # click
        self.setColor(self.transparentColor)

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
        # rect = QtCore.QRect(0, 0, width, height)
        painter.fillRect(rect, brush)


class AutoClickerAnimation(QWidget):
    """
    Custom Qt Widget to show a power bar and dial.
    Demonstrating compound and custom-drawn widget.
    """
    blink_speed = 16
    on_time = 3  # sec
    blink_time = 1.5  # sec
    onColor = QColor(255, 0, 0, 255)
    offColor = QColor(42, 13, 0, 255)

    def __init__(self, ind):
        super(AutoClickerAnimation, self).__init__()
        self.color = self.onColor
        layout = QVBoxLayout()
        self.autoclicker = AutoClicker(self)
        layout.addWidget(self.autoclicker)

        self.btn = QPushButton()
        self.btn.clicked.connect(self.btnClick)
        layout.addWidget(self.btn)

        self.setLayout(layout)

        self.ind = ind
        self.blinkOn = 2
        self.resetBit = False
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)

    def btnClick(self):
        self.blinkOn = 2
        self.autoclicker.setColor(self.onColor)

        time.sleep(self.on_time - self.blink_time)
        self.blink()

    def blink(self):
        dt = 1/self.blink_speed
        list = range(int(self.blink_time*self.blink_speed))
        for i in list:
            if self.resetBit:
                self.resetBit = False
                break
            time.sleep(dt)
            self.blinkVar = self.blink_speed
            self.blinkOn = (self.blinkOn - 1) % 3
            print(f"blink state: {self.blinkOn} for clicker {self.ind}")

            new_color = self.autoclicker.color

            if self.blinkOn == 0:
                new_color.setAlpha(30)
            elif self.blinkOn == 1:
                new_color.setAlpha(150)
            else:
                new_color.setAlpha(255)

            if not self.resetBit:
                self.autoclicker.setColor(new_color)

        print("finished!")
        self.autoclicker.setColor(self.offColor)

    def reset(self):
        self.resetBit = True
        # self.color = self.offColor
        self.ind = 0
        self.blinkOn = 2


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
        self.listWidget.setSelectionMode(
            QtWidgets.QAbstractItemView.SingleSelection
        )
        self.listWidget.setGeometry(QtCore.QRect(10, 10, 211, 291))

        # fetch player names and populate list
        rows = self.database.load_all_players()

        for player in rows:
            firstName = player[0]
            lastName = player[1]
            item = QtWidgets.QListWidgetItem(f"{firstName} {lastName}")
            self.listWidget.addItem(item)

        # on player choise - load player
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

        self.database.insert_person(playerFirstName, playerLastName, playerAge)
        self.loadPlayer(playerFirstName, playerLastName)

    def loadPlayerFromList(self):
        chosen_players = self.listWidget.selectedItems()
        x = str(chosen_players[0].text())
        name = x.split(" ")
        firstName = name[0]
        lasttName = name[1]
        self.loadPlayer(firstName, lasttName)

    def loadPlayer(self, firstName, lastName):
        self.database.load_person(firstName, lastName)

        # close player menu
        self.win.active_screen.show()
        # self.win.show()
        self.close()


if __name__ == '__main__':
    app = QApplication([])
    # volume = PowerBar()
    db = DataBase()
    volume = ChangeUserList(app, db)
    volume.show()
    app.exec_()