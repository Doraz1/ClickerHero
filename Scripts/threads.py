from PyQt5 import QtCore
from PyQt5.QtCore import QThread
import time
import numpy as np
import pyautogui
from PyQt5.QtGui import QRegion

class ClickerBlinkThread(QThread):
    def __init__(self, win, clicker):
        QThread.__init__(self)
        self.win = win
        self.clicker = clicker
        self.threadactive = False

    def run(self):
        try:
            self.threadactive = True
            self.clicker.activateLEDs()
            self.stop()
        except Exception as e:
            print(e)
            exit(2)


    def stop(self):
        self.threadactive = False
        # self.clicker.resetBlink()
        self.quit()

class ResetGuiThread(QThread):
    def __init__(self, win):
        QThread.__init__(self)
        self.win = win
        self.threadactive = False

    def run(self):
        try:
            print("entered gui reset thread")
            self.threadactive = True
            self.win.killAllThreads()
            for clicker in self.win.screens["game"].autoClickerAnimations:
                clicker.resetBlink()

            self.stop()
        except Exception as e:
            print(e)
            exit(3)


    def stop(self):
        self.threadactive = False
        self.quit()


class ACBlinkThread(QThread):
    easierLevel = 8

    def __init__(self, win, clicker_number, bpm, notes):
        QThread.__init__(self)
        self.win = win
        self.clicker = win.screens['game'].autoClickerAnimations[clicker_number - 1]
        self.bpm = bpm
        self.notes = notes
        self.blinkThread = ClickerBlinkThread(win, self.clicker)

        self.threadactive = False

    def run(self):
        try:
            self.threadactive = True

            start_time = time.time()
            iteration = 1
            note_ind = 0

            while self.win.running and self.threadactive:
                time_to_sleep = start_time - time.time() + self.win.note_times[iteration]
                time.sleep(time_to_sleep)
                next_note = self.notes[note_ind]
                if next_note == iteration:
                    prev_note = next_note
                    note_ind += 1
                    next_note = self.notes[note_ind]
                    self.blinkThread.start()
                iteration += 1
        except Exception as e:
            print(e)
            exit(4)


    def stop(self):
        self.threadactive = False
        self.quit()

class ACMoveThread(QThread):
    clicker_pos = QtCore.pyqtSignal(int, int, int)

    def __init__(self, win):
        QThread.__init__(self)
        self.win = win
        clicker1Anim = win.screens['game'].autoClickerAnimations[0]
        clicker2Anim = win.screens['game'].autoClickerAnimations[1]
        clicker3Anim = win.screens['game'].autoClickerAnimations[2]

        self.win = win
        self.clickers = [clicker1Anim, clicker2Anim, clicker3Anim]

        # Initialize constants
        self.noise = 100
        self.dt = 0.05
        self.screen_w, self.screen_h = pyautogui.size()
        self.clicker_radius = clicker1Anim.autoclicker.radius
        self.max_x, self.max_y = self.screen_w - 1.1*self.clicker_radius, self.screen_h - 1.1*self.clicker_radius
        self.min_x, self.min_y = self.clicker_radius, 450
        self.initial_x = []
        self.initial_y = []
        self.curr_x = []
        self.curr_y = []
        self.curr_ax = []
        self.curr_ay = []

        self.threadactive = False


    def run(self):
        try:
            self.threadactive = True
            self.move_ants()

        except Exception as e:
            print(e)
            exit(4)

    def move_ants(self):
        def in_bounds_x(curr_x, a_x):
            new_x = curr_x + 0.5 * a_x * (self.dt ** 2)
            if new_x > self.max_x:
                new_x = self.max_x
            elif new_x < self.min_x:
                new_x = self.min_x
            return new_x

        def in_bounds_y(curr_y, a_y):
            new_y = curr_y + 0.5 * a_y * (self.dt ** 2)
            if new_y > self.max_y:
                new_y = self.max_y
            elif new_y < self.min_y:
                new_y = self.min_y
            return new_y

        def sense_collision(new_x, new_y, i):
            collided = False
            for j in range(len(self.clickers)):
                if i != j:
                    if (new_x - self.curr_x[j])**2 + ((new_y - self.curr_y[j])**2) <= self.clicker_radius ** 2:
                        collided = True
            return collided

        for ac in self.clickers:
            geo = ac.geometry()
            x0, y0 = geo.x(), geo.y()
            self.initial_x.append(x0)
            self.initial_y.append(y0)
            self.curr_x.append(x0)
            self.curr_y.append(y0)
            self.curr_ax.append(0)
            self.curr_ay.append(0)

        while self.win.running:
            if not self.win.paused:
                for i, ac in enumerate(self.clickers):

                    if self.noise == 0:
                        rand = [0, 0]
                    else:
                        rand = np.random.randint(-self.noise, self.noise, 2)

                    new_ax = self.curr_ax[i] + rand[0]
                    new_ay = self.curr_ay[i] + rand[1]

                    new_x = in_bounds_x(self.curr_x[i], new_ax)
                    new_y = in_bounds_y(self.curr_y[i], new_ay)
                    collided = sense_collision(new_x, new_y, i)

                    if not collided:
                        self.curr_ax[i] = new_ax
                        self.curr_ay[i] = new_ay

                        self.curr_x[i] = new_x
                        self.curr_y[i] = new_y
                        # x, y = int(self.curr_x[i]), int(self.curr_y[i])
                        self.clicker_pos.emit(new_x, new_y, i)
                    else:
                        self.curr_ax[i] = 0
                        self.curr_ay[i] = 0

                time.sleep(self.dt)


    def stop(self):
        self.threadactive = False
        self.quit()


class ProgressBarThread(QThread):
    dt = 0.2  # sec
    progress = QtCore.pyqtSignal(int)

    def __init__(self, win, music):
        QThread.__init__(self)
        self.win = win
        self.music = music

        self.threadactive = False

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
        self.quit()
