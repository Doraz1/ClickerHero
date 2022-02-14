import time
import numpy as np
import PyQt5
from PyQt5.QtWidgets import QWidget, QSizePolicy, QVBoxLayout, QPushButton, QHBoxLayout, QInputDialog, QLineEdit,\
    QListWidget, QListWidgetItem
from PyQt5.QtGui import QColor, QPainter, QBrush, QPen
from PyQt5.QtCore import Qt
from Scripts.threads import ACBlinkThread, ACMoveThread
from Scripts.threads import Ros2QTSubscriber, QT2RosLEDPublisher, QT2RosNavPublisher

class AutoClicker(QWidget):
    '''
    Robot class with movement and LED simulators as well as real-time engines
    '''
    def __init__(self, win, ind):
        super().__init__(win)
        self.win = win
        self.ind = ind
        self.animation = AutoClickerAnimation(self, self.ind)
        self.score = 0
        self.x = 0
        self.y = 0
        'Movement thread - only for on-screen animations'
        self.MoveThread = ACMoveThread(self.win, self, self.ind) # both for sim and real
        self.MoveThread.clicker_pos.connect(self.move)

        'LED thread'
        self.clickerBlinkThread = ACBlinkThread(self.win, self, self.ind)

        if not self.win.simActive:
            'Location subscriber'
            self.rosSubscriberThread = Ros2QTSubscriber(self.win)
            self.rosSubscriberThread.camera_msg.connect(self.MoveThread.move_based_on_real_inputs)

            'Navigation publisher'
            self.rosNavPublisherThread = QT2RosNavPublisher(self.ind)
        else:
            pass

    def move(self, x, y, index):
        self.x = x
        self.y = y

        if self.win.DEBUG:
            print(f"Moving clicker to new coords: {x, y}")

        self.animation.move(x, y)

    def reset_blink(self):
        self.animation.reset_blink()

    def start_threads(self):
        print("Starting threads")
        self.clickerBlinkThread.start()
        self.MoveThread.start()

        if not self.win.simActive:
            self.rosNavPublisherThread.start()
            self.rosSubscriberThread.start()
        else:
            pass

    def reset(self):
        self.score = 0

        # reset threads
        self.reset_blink()
        self.MoveThread.stop()
        if not self.win.simActive:
            self.rosLEDPublisherThread.stop()
            self.rosNavPublisherThread.stop()
            self.rosSubscriberThread.stop()
        else:
            self.clickerBlinkThread.stop()


class AutoClickerAnimation(QWidget):
    blink_speed = 16
    on_time = 2  # sec
    blink_time = 1  # sec

    def __init__(self, parent, ind):
        """
        Activates the blinking animation of AutoClicker simulators on the right timing.
        """
        super(AutoClickerAnimation, self).__init__()
        self.ind = ind
        self.parent = parent

        self.helper = ACAnimMouseHandler(self)
        layout = QVBoxLayout()
        layout.addWidget(self.helper)
        self.setLayout(layout)

        self.resetBit = False

        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)

        self.numBlinkLevels = 8
        self.blinkState = self.numBlinkLevels - 1
        minLevel, maxLevel = 30, 255
        step = ((maxLevel - minLevel) / (self.numBlinkLevels - 1)) - 1
        self.alphaLevels = np.arange(30, 255, int(step), dtype=np.int32)

    def activateLEDs(self):
        self.helper.leds_active = True

        self.blinkState = self.numBlinkLevels - 1
        color = self.helper.onColor
        color.setAlpha(self.alphaLevels[self.blinkState])
        self.helper.setColor(color)

        time.sleep(self.on_time - self.blink_time)
        self.blink()

    def blink(self):
        dt = 1/self.blink_speed
        list = range(int(self.blink_time * self.blink_speed))
        for i in list:
            if not self.helper.leds_active:
                break
            time.sleep(dt)

            self.blinkState = (self.blinkState - 1) % self.numBlinkLevels

            color = self.helper.onColor
            color.setAlpha(self.alphaLevels[self.blinkState])

            if not self.resetBit and self.helper.leds_active:
                self.helper.setColor(color)

        self.reset_blink()

    def reset_blink(self):
        # self.resetBit = True
        self.helper.setColor(self.helper.offColor)
        self.helper.leds_active = False
        self.ind = 0
        self.blinkState = self.numBlinkLevels - 1


class ACAnimMouseHandler(QWidget):
    # transparentColor = QColor(50, 50, 50, 30)
    onColor = QColor(255, 0, 0, 255)
    offColor = QColor(42, 13, 0, 255)

    def __init__(self, parent):
        """
        Detects mouse clicks on AutoClicker animation and triggers color changes.
        :param parent:
        """
        super().__init__(parent)
        self.parent = parent
        self.color = self.offColor

        self.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
        self.isClicked = False
        self.leds_active = False
        self.firstRun = True
        self.region = None
        self.radius = 100

    def sizeHint(self):
        return PyQt5.QtCore.QSize(60, 120)


    def mouseMoveEvent(self, e):
        # click and drag
        pass

    def mousePressEvent(self, e):
        # click
        if self.leds_active:
            self.setColor(self.offColor)
            self.leds_active = False
            self.parent.parent.score += 1 # AutoClicker score

    def setColor(self, color):
        self.color = color
        self.update()

    def paintEvent(self, e):
        bg_color = self.color

        painter = QPainter(self)
        painter.setPen(QPen(Qt.black, 1, Qt.SolidLine))
        painter.setBrush(QBrush(bg_color, Qt.SolidPattern))
        painter.drawEllipse(0, 0, self.radius, self.radius)



class ChangeUserList(QWidget):
    def __init__(self, win, database):
        super(ChangeUserList, self).__init__()
        self.win = win
        self.database = database
        self.listWidget = QListWidget()
        self.layout = QHBoxLayout()
        self.width = 200
        self.height = 300

        self.createList()

    def createList(self):
        # create layout
        self.listWidget.setSelectionMode(PyQt5.QtWidgets.QAbstractItemView.SingleSelection)
        self.listWidget.setGeometry(PyQt5.QtCore.QRect(int((self.win.width - self.width) / 2), int((self.win.height - self.height)/2), self.width, self.height))
        self.move(int((self.win.width - self.width) / 2), int((self.win.height - self.height)/2))
        # fetch player names and populate list
        rows = self.database.load_all()
        for player in rows:
            firstName = player[0]
            lastName = player[1]
            item = QListWidgetItem(f"{firstName} {lastName}")
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



