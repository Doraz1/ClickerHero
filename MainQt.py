import os
import sys
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QMessageBox,
)
from PyQt5.QtGui import QIcon
import pyautogui
import pygame.mixer
from mutagen.mp3 import MP3
from Scripts.myCustomWidgets import AutoClickerAnimation, ChangeUserList
from Scripts.database import PlayerDataBase
from Scripts.threads import ProgressBarThread, ACMoveThread, ACBlinkThread, ResetGuiThread, ClickerBlinkThread
from Scripts.screens import MainScreen, SecondScreen, GameScreen, ScoreScreen
import numpy as np

# region init
# load assets and songs
ROOT_PATH = os.path.dirname(os.path.abspath(__file__))
SONGS_PATH = os.path.join(ROOT_PATH, "Songs")
# ASSET_PATH = os.path.join(ROOT_PATH, "Assets")
ASSET_PATH = r'Assets/'
# endregion


class MyWindow(QMainWindow):
    def __init__(self, width, height, title):
        super().__init__()
        full_screen_size = pyautogui.size()
        self.playerDataBase = PlayerDataBase()
        self.width = width
        self.height = height
        self.setGeometry(int((full_screen_size[0] - width)/2), int((full_screen_size[1] - height)/2), width, height)

        # Set screen title, icon and application style
        self.setWindowTitle(title)
        self.setWindowIcon(QIcon(os.path.join(ASSET_PATH, 'green button.png')))

        self.active_screen = None

        # Define the different screens
        main_screen = MainScreen(self, "Main Menu Screen")
        secondary_screen = SecondScreen(self, "Secondary Menu Screen")
        game_screen = GameScreen(self, "Game Screen")
        score_screen = ScoreScreen(self, "Score Screen")
        self.screens = {'main': main_screen,
                        'second': secondary_screen,
                        'game': game_screen,
                        'score': score_screen
                        }

        self.total_song_length = -1
        self.changing_user = False
        self.running = False
        self.paused = False
        self.first_run = True
        self.beats_per_min = -1
        self.song_path = ""
        self.audio = None
        self.activate_screen("main")  # Load the main screen on game startup
        self.user_list = []  # player list
        self.show()  # render

    def activate_screen(self, screenName):
        if self.active_screen is not None:
            self.active_screen.hide()

        for screen_name, screen in self.screens.items():
            if screen_name == screenName:
                screen.show()
                self.active_screen = screen
                print(f"currently active: {screen.name}")

    # region Buttons
    def btn_start_game(self):
        pygame.mixer.quit()
        self.activate_screen("second")

    def btn_return_to_main(self):
        self.activate_screen("main")

    def btn_change_user(self):
        self.user_list = ChangeUserList(self, self.playerDataBase)
        self.user_list.show()

    def btn_exit(self):
        button_reply = QMessageBox.question(self, 'PyQt5 message', "Exit the application?",
                                            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if button_reply == QMessageBox.Yes:
            sys.exit()

    def btn_choose_song(self):
        self.song_path = os.path.join(SONGS_PATH, "One Kiss_cropped.mp3")
        self.audio = MP3(self.song_path)
        self.beats_per_min = 123.3  # using audacity we got 123, updated a bit
        notes = self.get_autoclicker_notes('one kiss')
        self.begin_game(notes)

    def get_autoclicker_notes(self, song_name):
        beats_per_sec = self.beats_per_min / 60  # 2
        dt = 1 / beats_per_sec  # no easy up
        self.total_song_length = self.audio.info.length
        note_times = np.arange(0, self.total_song_length, dt)
        iters = range(len(note_times))[1:]  # get rid of note 0
        notes = []

        if song_name == 'one kiss':
            # iterations to play on, assuming each iteration is the shortest beat
            # up to 430 iterations
            easyUp = ACBlinkThread.easierLevel
            ac1_notes = [easyUp * iter for iter in iters if iter<(iters[-1]//easyUp) + 1]
            ac2_notes = [easyUp * iter for iter in iters if iter<(iters[-1]//easyUp) + 1]
            ac3_notes = [easyUp * iter + 4 for iter in iters if iter<(iters[-1]//easyUp) + 1]
            notes.append(ac1_notes)
            notes.append(ac2_notes)
            notes.append(ac3_notes)
        return notes

    def begin_game(self, notes):
        self.running = True

        if self.first_run:
            self.first_run = False

            self.progressBarThread = ProgressBarThread(self, pygame.mixer.music)
            self.progressBarThread.progress.connect(self.updateProgressBar)

            auto_clicker1 = AutoClickerAnimation(0)
            auto_clicker2 = AutoClickerAnimation(1)
            auto_clicker3 = AutoClickerAnimation(2)
            self.screens["game"].autoClickerAnimations.append(auto_clicker1)
            self.screens["game"].autoClickerAnimations.append(auto_clicker2)
            self.screens["game"].autoClickerAnimations.append(auto_clicker3)

            self.clickerMoveThread = ACMoveThread(self)
            self.clickerMoveThread.clicker_pos.connect(self.moveAutoClickers)

            self.clickerBlinkThread1 = ACBlinkThread(self, 1, self.beats_per_min, notes[0])
            self.clickerBlinkThread2 = ACBlinkThread(self, 2, self.beats_per_min, notes[1])
            self.clickerBlinkThread3 = ACBlinkThread(self, 3, self.beats_per_min, notes[2])

            self.resetGuiThread = ResetGuiThread(self)

        click_offset_x = 200
        click_offset_y = 200

        ac1_x, ac1_y = int(self.width / 2), int(self.height / 2)
        ac2_x, ac2_y = int(self.width / 2) - click_offset_x, int(self.height / 2) + click_offset_y
        ac3_x, ac3_y = int(self.width / 2) + click_offset_x, int(self.height / 2) + click_offset_y
        self.screens["game"].autoClickerAnimations[0].move(ac1_x, ac1_y)
        self.screens["game"].autoClickerAnimations[1].move(ac2_x, ac2_y)
        self.screens["game"].autoClickerAnimations[2].move(ac3_x, ac3_y)

        self.activate_screen("game")

        pygame.mixer.init()  # init pygame mixer
        pygame.mixer.music.load(self.song_path)  # charge la musique
        pygame.mixer.music.play()

        self.progressBarThread.start()
        self.clickerMoveThread.start()
        self.clickerBlinkThread1.start()
        self.clickerBlinkThread2.start()
        self.clickerBlinkThread3.start()

    def updateProgressBar(self, progress):
        self.screens['game'].progressBars[0].setValue(progress)

    def moveAutoClickers(self, x, y, index):
        self.screens['game'].autoClickerAnimations[index].move(x, y)

    def resetGui(self):
        # self.resetGuiThread.start()
        self.killAllThreads()
        for clicker in self.screens["game"].autoClickerAnimations:
            clicker.reset()

    def killAllThreads(self):
        self.progressBarThread.stop()
        self.clickerMoveThread.stop()
        self.clickerBlinkThread1.stop()
        self.clickerBlinkThread2.stop()
        self.clickerBlinkThread3.stop()

    def btn_pause_game(self):
        if self.paused:
            pygame.mixer.music.unpause()
        else:
            pygame.mixer.music.pause()

        self.paused = not self.paused

    def btn_stop_game(self):
        self.running = False
        self.activate_screen("score")
        pygame.mixer.music.fadeout(2000)

        self.resetGui()

    # endregion


def launch_game():
    app = QApplication(sys.argv)
    fullscreen = True
    if fullscreen:
        size = pyautogui.size()
        width, height = size[0], size[1]
    else:
        width, height = 1600, 900

    MyWindow(width, height, "Clicker Hero")

    sys.exit(app.exec_())


if __name__ == "__main__":
    launch_game()
