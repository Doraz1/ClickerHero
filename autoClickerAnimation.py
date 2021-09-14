from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QWidget, QSizePolicy, QVBoxLayout, QPushButton, QApplication
from PyQt5.QtGui import QColor, QRegion, QPainter, QBrush
from PyQt5.QtCore import Qt, QRect

class AutoClicker(QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        self.color = 'black'
        self.parent = parent
        self.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)


    def sizeHint(self):
        return QtCore.QSize(60, 120)

    def mouseMoveEvent(self, e):
        print("click and drag")
        # self._calculate_clicked_value(e)

    def mousePressEvent(self, e):
        print("clicked")
        # self.setColor('transparent')
        self.setColor(QColor(30, 0, 30, 20))

    def _calculate_clicked_value(self, e):
        parent = self.parent()
        vmin, vmax = parent.minimum(), parent.maximum()
        click_y = e.y() - self._padding - 20 / 2

    def setColor(self, color):
        self.color = color
        self.update()

    def paintEvent(self, e):
        painter = QPainter(self.parent)
        width = self.parent.frameGeometry().width()
        height = self.parent.frameGeometry().height()
        region = QRegion(QRect(40, 30, int(width/2), int(height/2)), QRegion.Ellipse)
        self.setMask(region)

        bg_color = self.color

        painter = QPainter(self)
        brush = QBrush()
        brush.setStyle(Qt.SolidPattern)
        brush.setColor(QColor(bg_color))

        rect = QtCore.QRect(0, 0, painter.device().width(), painter.device().height())
        painter.fillRect(rect, brush)


class PowerBar(QWidget):
    """
    Custom Qt Widget to show a power bar and dial.
    Demonstrating compound and custom-drawn widget.
    """

    def __init__(self, *args, **kwargs):
        super(PowerBar, self).__init__(*args, **kwargs)
        self.color = 'red'

        layout = QVBoxLayout()
        self.autoclicker = AutoClicker(self)
        layout.addWidget(self.autoclicker)

        self.btn = QPushButton()
        self.btn.clicked.connect(self.btnClick)
        layout.addWidget(self.btn)

        self.setLayout(layout)
        self.ind = 0
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)

    def btnClick(self):
        colors = ['red', 'green', 'blue']
        self.autoclicker.setColor(colors[self.ind])
        self.ind = (self.ind + 1) % len(colors)



if __name__ == '__main__':
    app = QApplication([])
    volume = PowerBar()
    volume.show()
    app.exec_()