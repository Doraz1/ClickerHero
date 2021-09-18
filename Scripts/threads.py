from PyQt5 import QtCore
from PyQt5.QtCore import QThread
import time
import numpy as np


class ResetGuiThread(QThread):
    def __init__(self, win):
        self.win = win
        self.threadactive = False


    def run(self):
        self.threadactive = True

        self.win.resetGui()
        self.stop()

    def stop(self):
        self.threadactive = False
        self.wait()


class ACBlinkThread(QThread):
    easierLevel = 8

    def __init__(self, win, clicker_number, bpm):
        QThread.__init__(self)
        self.win = win
        self.clicker = win.screens['game'].autoClickerAnimations[clicker_number - 1]
        self.bpm = bpm

        self.threadactive = False

    def run(self):
        self.threadactive = True

        # clicker animation constants
        start_time = time.time()
        beats_per_sec = self.bpm / 60  # 2
        dt = self.easierLevel / beats_per_sec  # every this many seconds
        light_up_times = np.arange(0, self.win.total_song_length, dt)
        iteration = 1
        while self.win.running:
            # time_to_sleep = start_time + iteration * dt - time.time()
            time_to_sleep = start_time - time.time() + light_up_times[iteration]
            time.sleep(time_to_sleep)

            self.clicker.btnClick()
            iteration += 1

    def stop(self):
        self.threadactive = False
        self.wait()


class ACMoveThread(QThread):
    clicker_pos = QtCore.pyqtSignal(int, int, int)

    def __init__(self, win):
        QThread.__init__(self)
        self.win = win
        self.clicker1 = win.screens['game'].autoClickerAnimations[0]
        self.clicker2 = win.screens['game'].autoClickerAnimations[1]
        self.clicker3 = win.screens['game'].autoClickerAnimations[2]
        self.threadactive = False


    def __del__(self):
        self.wait()

    def run(self):
        self.threadactive = True

        # clicker animation constants
        r = 80
        angle = np.radians(90)
        omega = 1
        noise = 0
        dt = 0.05

        geo1 = self.clicker1.geometry()  # QRect
        ac1_x, ac1_y = geo1.x(), geo1.y()
        center_of_rot_1 = (ac1_x - r, ac1_y)  # Center of Rotation

        geo2 = self.clicker2.geometry()
        ac2_x, ac2_y = geo2.x(), geo2.y()
        center_of_rot_2 = (ac2_x - r, ac2_y)  # Center of Rotation

        geo3 = self.clicker3.geometry()
        ac3_x, ac3_y = geo3.x(), geo3.y()
        center_of_rot_3 = (ac3_x - r, ac3_y)  # Center of Rotation

        while self.win.running:
            # #animations
            if noise == 0:
                rand = [0, 0, 0, 0, 0, 0]
            else:
                rand = np.random.randint(-noise, noise, 6)

            x1 = center_of_rot_1[0] + r * np.cos(angle) + rand[0]
            y1 = center_of_rot_1[1] - r * np.sin(angle) + rand[1]
            x2 = center_of_rot_2[0] + r * np.cos(angle) + rand[2]
            y2 = center_of_rot_2[1] - r * np.sin(angle) + rand[3]
            x3 = center_of_rot_3[0] + r * np.cos(angle) + rand[4]
            y3 = center_of_rot_3[1] - r * np.sin(angle) + rand[5]
            #
            angle = (angle + omega * dt) % 360
            self.clicker_pos.emit(int(x1), int(y1), 0) # index 0
            self.clicker_pos.emit(int(x2), int(y2), 1) # index 1
            self.clicker_pos.emit(int(x3), int(y3), 2) # index 2

            time.sleep(dt)

    def stop(self):
        self.threadactive = False
        self.wait()


class ProgressBarThread(QThread):
    dt = 1  # sec
    progress = QtCore.pyqtSignal(int)

    def __init__(self, win, music):
        QThread.__init__(self)
        self.win = win
        self.music = music

        self.threadactive = False

    def __del__(self):
        self.wait()

    def run(self):
        self.threadactive = True

        while self.win.running:
            if self.music.get_pos() == -1:
                # song finished running
                self.btn_stop_game()
                return

            curr_progress = self.music.get_pos() / 1000  # in seconds
            progress_percentage = curr_progress / self.win.total_song_length * 100
            self.progress.emit(int(progress_percentage))

            time.sleep(self.dt)

    def stop(self):
        self.threadactive = False
        self.wait()