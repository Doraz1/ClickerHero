import time
import numpy as np
from PyQt5.QtWidgets import QWidget, QSizePolicy, QVBoxLayout
from PyQt5.QtGui import QColor, QPainter, QBrush, QPen
from PyQt5 import QtCore
from PyQt5.QtCore import Qt
from Scripts.threads import ACNavBlinkThread, ACMoveThread
from Scripts.threads import Ros2QTSubscriber


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
        self.clicked = False

        'LED + navigation thread'
        self.blinkNavThread = ACNavBlinkThread(self.win, self)

        'Movement thread - only for on-screen animations'
        self.MoveThread = ACMoveThread(self.win, self) # both for sim and real, in real it's camera locations fed into the animation locations
        self.MoveThread.clicker_pos.connect(self.move)

        if not self.win.simActive:
            'Location + clicked subscriber'
            self.rosSubscriberThread = Ros2QTSubscriber(self.win, self.ind)
            self.rosSubscriberThread.camera_msg.connect(self.MoveThread.move_based_on_real_inputs)
            self.rosSubscriberThread.clicked_msg.connect(self.robot_clicked_method)
        else:
            pass

    def robot_clicked_method(self, num_clicks):
        pass

    def move(self, x, y, index):
        self.x = x
        self.y = y

        if self.win.DEBUG:
            # print(f"Moving clicker to new coords: {x, y}")
            pass

        self.animation.move(x, y)

    def reset_blink(self):
        self.animation.reset_blink()

    def start_threads(self):
        if self.win.DEBUG:
            print("Starting threads")
        self.blinkNavThread.start()
        self.MoveThread.start()

        if not self.win.simActive:
            # self.rosNavPublisherThread.start()
            self.rosSubscriberThread.start()
        else:
            pass

    def reset(self):
        self.score = 0

        # reset threads
        if self.win.simActive:
            self.reset_blink()  # simulated blink
        else:
            self.rosSubscriberThread.stop()

        self.MoveThread.stop()  # animation movement - in sim it's ant movement, in non-sim it's cam locations
        self.blinkNavThread.stop()


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
        """
        Initialize a background thread that blinks the animation, resets the color and terminates.

        :return:
        """

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
        self.region = None
        self.radius = 100

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






