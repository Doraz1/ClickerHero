import os
import sys
from PyQt5 import QtCore
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QLabel,
    QPushButton,
    QLineEdit,
    QMessageBox,
    QProgressBar
)
from PyQt5.QtGui import QPixmap, QIcon, QFont, QPainter
from PyQt5.QtCore import Qt, QThread
import pyautogui
from webcolors import rgb_to_hex
import pygame.mixer
from mutagen.mp3 import MP3
import time
from autoClickerAnimation import PowerBar
import numpy as np


# region init
# load assets and songs
ROOT_PATH = os.path.dirname(os.path.abspath(__file__))
SONGS_PATH = os.path.join(ROOT_PATH, "Songs")
# ASSET_PATH = os.path.join(ROOT_PATH, "Assets")
ASSET_PATH = r'Assets/'

# endregion


class ACThread(QThread):
    clicker_pos = QtCore.pyqtSignal(int, int)

    def __init__(self, win):
        QThread.__init__(self)
        self.win = win

        # self.clicker1 = win.screens['game'].autoClickerAnimations[0]
        # self.clicker3 = win.screens['game'].autoClickerAnimations[1]
        # self.clicker2 = win.screens['game'].autoClickerAnimations[2]

    def __del__(self):
        self.wait()

    def run(self):
        # clicker animation constants
        curr_w = int(self.win.width / 2)
        curr_h = int(self.win.height / 2)
        r = 80
        center_of_rotation = (curr_w, curr_h)
        angle = np.radians(45)
        omega = 0.1
        noise = 0
        dt = 0.2
        while self.win.running:
            # #animations
            if noise == 0:
                rand = [0, 0]
            else:
                rand = np.random.randint(-noise, noise, 2)

            x1 = center_of_rotation[0] + r * np.cos(angle) + rand[0]
            y1 = center_of_rotation[1] - r * np.sin(angle) + rand[1]
            # x2 = center_of_rotation[0] + r * np.cos(angle) + rand[2]
            # y2 = center_of_rotation[1] - r * np.sin(angle) + rand[3]
            # x3 = center_of_rotation[0] + r * np.cos(angle) + rand[4]
            # y3 = center_of_rotation[1] - r * np.sin(angle) + rand[5]
            #
            angle = (angle + omega * dt) % 360
            self.clicker_pos.emit(int(x1), int(y1)) # index 0
            # self.clicker_pos.emit(int(x2), int(y2), 1) # index 1
            # self.clicker_pos.emit(int(x3), int(y3), 2) # index 2

            time.sleep(dt)


class ProgressBarThread(QThread):
    dt = 1 #  sec
    progress = QtCore.pyqtSignal(int)

    def __init__(self, win, music):
        QThread.__init__(self)
        self.win = win
        self.music = music

    def __del__(self):
        self.wait()

    def run(self):
        dt = 1

        while self.win.running:
            if self.music.get_pos() == -1:
                # song finished running
                self.btn_stop_game()
                return

            curr_progress = self.music.get_pos() / 1000  # in seconds
            progress_percentage = curr_progress / self.win.total_song_length * 100
            self.progress.emit(int(progress_percentage))

            # self.win.screens['game'].progressBars[0].setValue(int(progress_percentage))
            time.sleep(dt)


class Screen:
    # Label style
    lbl_color = rgb_to_hex((180, 180, 255))
    lbl_width = 1300
    lbl_height = 140
    lbl_margin_y = 50
    lbl_text_color = rgb_to_hex((240, 240, 240))
    lbl_bg_color = Qt.transparent
    lbl_text_font = QFont('Times', 25, QFont.Bold)

    # Button style
    btn_color = rgb_to_hex((240, 240, 240))
    btn_width = 700
    btn_height = 140
    btn_margin_x = 30
    btn_margin_y = 50
    btn_text_color = rgb_to_hex((240, 240, 240))
    btn_bg_color = rgb_to_hex((100, 100, 255))
    btn_text_font = QFont('Times', 25, QFont.Bold)

    prog_bar_bg_color = rgb_to_hex((150, 120, 180))

    def __init__(self, win, name):
        self.win = win
        self.name = name
        self.labels = []
        self.buttons = []
        self.autoClickerAnimations = []
        self.progressBars = []

    def show(self):
        self.win.setWindowTitle(self.name)

        for lbl in self.labels:
            lbl.show()
        for btn in self.buttons:
            btn.show()
        for bar in self.progressBars:
            bar.show()
        for clicker in self.autoClickerAnimations:
            clicker.show()

    def hide(self):
        for lbl in self.labels:
            lbl.hide()
        for btn in self.buttons:
            btn.hide()
        for bar in self.progressBars:
            bar.hide()
        for clicker in self.autoClickerAnimations:
            clicker.hide()

    # def update(self):
    #     for lbl in self.buttons:
    #         lbl.adjustSize()
    #     for btn in self.buttons:
    #         btn.adjustSize()
    #     for bar in self.progressBars:
    #         bar.adjustSize()
    #     for clicker in self.autoClickerAnimations:
    #         clicker.adjustSize()

    def create_background(self, label, image_path):
        pix = QPixmap(image_path)
        pixmap_resized = pix.scaled(self.win.width, self.win.height, Qt.KeepAspectRatio)
        # pixmap_resized = pix.scaledToWidth(self.win.width)
        label.setPixmap(pixmap_resized)
        label.resize(self.win.width, self.win.height)

    def stylize_lbl(self, lbl, pos_x, pos_y, logic, text=""):
        lbl.setText(text)
        lbl.clicked.connect(logic)

        lbl.setFont(self.btn_text_font)
        lbl.setGeometry(pos_x, pos_y, self.btn_width, self.btn_height)
        lbl.setStyleSheet(f"background-color: {self.btn_bg_color}; "
                          f"color: {self.btn_text_color};"
                          "border-radius: 10;"
                          "border: 1px solid red;"
                          "text-align: center;"
                          "opacity: 255;"  # 0-255
                          "padding: 3px;"
                          "margin: 2px;")

    def stylize_btn(self, btn, pos_x, pos_y, logic, text=""):
        # self.closeButton.setShortcut('Ctrl+D')  # shortcut key
        # self.closeButton.setToolTip("Close the widget")  # Tool tip

        btn.setText(text)
        btn.clicked.connect(logic)

        btn.setFont(self.btn_text_font)
        btn.setGeometry(pos_x, pos_y, self.btn_width, self.btn_height)
        btn.setStyleSheet(f"background-color: {self.btn_bg_color}; "
                          f"color: {self.btn_text_color};"
                          "border-radius: 10;"
                          "border: 1px solid red;"
                          "text-align: center;"
                          "opacity: 255;"  # 0-255
                          "padding: 3px;"
                          "margin: 2px;")

    def stylize_progress_bar(self, bar):
        prog_width = 800
        prog_height = 150
        bar.setGeometry(int((self.win.width - prog_width) / 2), 250, prog_width, prog_height)

        bar.setAlignment(Qt.AlignCenter)
        bar.setFont(self.btn_text_font)
        # bar.setFont(QFont('Arial', 20))
        bar.setStyleSheet(f"background-color: {self.prog_bar_bg_color};"
                          f"border: 1px solid red;"
                          f"border-radius: 10;")

    def stylize_autoclicker(self, clicker, pos_x, pos_y, logic, text=""):
        clicker.setText(text)
        clicker.clicked.connect(logic)

        clicker.setFont(self.btn_text_font)
        clicker.setGeometry(pos_x, pos_y, self.btn_width, self.btn_height)


        image_pixmap = QPixmap("Assets/green button.png")
        scaled_pixmap = image_pixmap.scaled(self.win.width, self.win.height, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        painter = QPainter(self.win)
        painter.drawPixmap(0, 0, image_pixmap)
        # painter.drawPixmap(QRect(50, 0, 50, 50), image_pixmap)
        # painter.setRenderHint(QPainter.SmoothPixmapTransform)
        # painter.drawPixmap(QRect(0, 0, 50, 50), image_pixmap)
        # img = img.scaled(50, 50, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
        # p.drawPixmap(100, 0, img);


class MainScreen(Screen):
    def __init__(self, win, name):
        super().__init__(win, name)
        self.create_screen()
        self.hide()

    def create_screen(self):
        # Background
        bg_image_path = ASSET_PATH + "BG_main.jpg"
        bg_label = QLabel(self.win)
        self.create_background(bg_label, bg_image_path)
        self.labels.append(bg_label)

        # Labels
        label1 = QLabel(self.win)
        label1.setText("label1")
        label1.move(30, 30)
        self.labels.append(label1)

        label2 = QLabel(self.win)
        label2.setText("Welcome!")
        label2.move(80, 30)
        self.labels.append(label2)

        # Buttons
        btn_x, btn_y = int((self.win.width - self.btn_width) / 2), 100  # first button coordinates
        dy = self.btn_height + self.btn_margin_y

        button_start = QPushButton(self.win)
        self.stylize_btn(button_start, btn_x, btn_y, self.win.btn_start_game, "התחל משחק")
        self.buttons.append(button_start)

        button_change_user = QPushButton(self.win)
        self.stylize_btn(button_change_user, btn_x, btn_y + dy, self.win.btn_change_user, "החלף משתמש")
        self.buttons.append(button_change_user)

        button_exit = QPushButton(self.win)
        self.stylize_btn(button_exit, btn_x, btn_y + 2*dy, self.win.btn_exit, "יציאה")
        self.buttons.append(button_exit)


class SecondScreen(Screen):

    def __init__(self, win, name):
        super(SecondScreen, self).__init__(win, name)
        self.create_screen()
        self.hide()

    def create_screen(self):
        # Background
        bg_image_path = ASSET_PATH + "BG_sec.jpg"
        bg_label = QLabel(self.win)
        self.create_background(bg_label, bg_image_path)
        self.labels.append(bg_label)

        # Labels
        label1 = QLabel(self.win)
        label1.setText("One kiss")
        label1.move(30, 30)
        self.labels.append(label1)

        # Buttons
        btn_x, btn_y = int((self.win.width - self.btn_width) / 2), 100  # first button coordinates
        dy = self.btn_height + self.btn_margin_y

        button_start = QPushButton(self.win)
        self.stylize_btn(button_start, btn_x, btn_y, self.win.btn_choose_song, "One kiss")
        self.buttons.append(button_start)

        button_return_to_main = QPushButton(self.win)
        self.stylize_btn(button_return_to_main, btn_x, btn_y + dy, self.win.btn_return_to_main, "חזרה לתפריט הראשי")
        self.buttons.append(button_return_to_main)


class GameScreen(Screen):

    def __init__(self, win, name):
        super().__init__(win, name)
        self.create_screen()
        self.hide()

    def create_screen(self):
        # Background
        bg_image_path = ASSET_PATH + "BG_game.jpg"
        bg_label = QLabel(self.win)
        self.create_background(bg_label, bg_image_path)
        self.labels.append(bg_label)

        # Labels
        label1 = QLabel(self.win)
        label1.setText("שיר 1")
        label1.move(30, 30)
        self.labels.append(label1)

        # Buttons
        button_pause = QPushButton(self.win)
        self.stylize_btn(button_pause, 0, 10, self.win.btn_pause_game, "P")
        self.buttons.append(button_pause)

        button_exit_to_main = QPushButton(self.win)
        end_of_screen_pos = self.win.width - self.btn_width
        self.stylize_btn(button_exit_to_main, end_of_screen_pos, 10, self.win.btn_stop_game, "חזרה לתפריט הראשי")
        self.buttons.append(button_exit_to_main)

        progress_bar = QProgressBar(self.win)
        self.stylize_progress_bar(progress_bar)
        self.progressBars.append(progress_bar)

        # autoclicker animations


        # self.win.autoClicker1Thread = self.create_thread()

    # def create_thread(self):
    #     thread = QThread()
    #     worker = Worker()
    #     worker.moveToThread(thread)
    #     thread.started.connect(worker.run)
    #     worker.finished.connect(thread.quit)
    #     worker.finished.connect(worker.deleteLater)
    #     thread.finished.connect(thread.deleteLater)
    #     worker.progress.connect(self.win.animation1)
    #     return thread


class ScoreScreen(Screen):

    def __init__(self, win, name):
        super().__init__(win, name)
        self.create_screen()
        self.hide()

    def create_screen(self):
        # Background
        bg_image_path = os.path.join(ASSET_PATH, "BG_score.jfif")
        bg_label = QLabel(self.win)
        self.create_background(bg_label, bg_image_path)
        self.labels.append(bg_label)

        # Labels
        label1 = QLabel(self.win)
        label1.setText("כל הכבוד!")
        label1.move(130, 130)
        self.labels.append(label1)

        # Buttons
        btn_x, btn_y = int((self.win.width - self.btn_width) / 2), 100  # first button coordinates
        dy = self.btn_height + self.btn_margin_y

        button_return_to_main = QPushButton(self.win)
        self.stylize_btn(button_return_to_main, btn_x, btn_y + dy, self.win.btn_return_to_main, "חזרה לתפריט הראשי")
        self.buttons.append(button_return_to_main)


class MyWindow(QMainWindow):
    def __init__(self, width, height, title):
        super().__init__()
        full_screen_size = pyautogui.size()
        self.width = width
        self.height = height
        self.setGeometry(int((full_screen_size[0] - width)/2), int((full_screen_size[1] - height)/2), width, height)

        # Set screen title, icon and application style
        self.setWindowTitle(title)
        self.setWindowIcon(QIcon(os.path.join(ASSET_PATH, 'green button.png')))

        self.active_screen = "None"

        # Define the different screens
        main_screen = MainScreen(self, "Main Menu Screen")
        secondary_screen = SecondScreen(self, "Secondary Menu Screen")
        game_screen = GameScreen(self, "Game Screen")
        score_screen = ScoreScreen(self, "Score Screen")
        self.screens = {'main': main_screen,
                        'second': secondary_screen,
                        'game': game_screen,
                        'score': score_screen}

        self.total_song_length = -1
        self.changing_user = False
        self.running = False
        self.paused = False

        self.activate_screen("main")  # Load the main screen on game startup
        self.show()  # render

    def activate_screen(self, screen):
        if self.screens.get(self.active_screen) is not None:
            self.screens[self.active_screen].hide()

        for key, value in self.screens.items():
            if key == screen:
                self.screens[screen].show()
                self.active_screen = key
                print(f"currently active: {value.name}")

    # region Buttons
    def btn_start_game(self):
        pygame.mixer.quit()
        self.activate_screen("second")

    def btn_return_to_main(self):
        self.activate_screen("main")

    def btn_change_user(self):
        if not self.changing_user:
            self.changing_user = True
            le = QLineEdit("User name here:", self)
            le.move(50, 50)
            le.show()

    def btn_exit(self):
        button_reply = QMessageBox.question(self, 'PyQt5 message', "Exit the application?",
                                            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if button_reply == QMessageBox.Yes:
            sys.exit()

    def btn_choose_song(self):
        self.running = True
        song_path = os.path.join(SONGS_PATH, "One Kiss.mp3")
        audio = MP3(song_path)
        self.total_song_length = audio.info.length  # in seconds 223.128

        pygame.mixer.init()  # init pygame mixer
        pygame.mixer.music.load(song_path)  # charge la musique
        pygame.mixer.music.play()

        _progressBarThread = ProgressBarThread(self, pygame.mixer.music)
        _progressBarThread.progress.connect(self.updateProgressBar)
        _progressBarThread.start()

        # add autoclicker animations to screen
        autoClicker1 = PowerBar()
        autoClicker1.move(int(self.width / 2), int(self.height / 2))
        self.screens["game"].autoClickerAnimations.append(autoClicker1)

        # autoClicker2 = PowerBar()
        # autoClicker2.move(int(self.width / 2), int(self.height / 2))
        # self.screens["game"].autoClickerAnimations.append(autoClicker2)

        # autoClicker3 = PowerBar()
        # autoClicker3.move(int(self.width / 2), int(self.height / 2))
        # self.screens["game"].autoClickerAnimations.append(autoClicker3)

        _autoClicker1Thread = ACThread(self)
        _autoClicker1Thread.clicker_pos.connect(self.moveAutoClickers)
        _autoClicker1Thread.start()





        self.activate_screen("game")

        # animationThread = Thread(target=animation_thread, args=(self, autoClicker1))
        # animationThread.start()

    # @pyqtSlot(int)
    def updateProgressBar(self, progress):
        self.screens['game'].progressBars[0].setValue(progress)

    def moveAutoClickers(self, x, y):
        self.screens['game'].autoClickerAnimations[0].move(x, y)


    def btn_pause_game(self):
        if self.paused:
            pygame.mixer.music.unpause()
        else:
            pygame.mixer.music.pause()

        self.paused = not self.paused

    def btn_stop_game(self):
        self.running = False
        pygame.mixer.music.fadeout(2000)
        self.activate_screen("score")
    # endregion



def launch_game():
    app = QApplication(sys.argv)
    fullscreen = True
    if fullscreen:
        size = pyautogui.size()
        width, height = size[0], size[1]
    else:
        width, height = 1800, 900

    MyWindow(width, height, "Clicker Hero")

    sys.exit(app.exec_())


if __name__ == "__main__":
    launch_game()
