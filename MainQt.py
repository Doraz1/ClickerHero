import os
import sys

import rclpy
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QMessageBox,
)
from PyQt5.QtGui import QIcon
import pyautogui
import pygame.mixer
from mutagen.mp3 import MP3
import numpy as np
from Scripts.myCustomWidgets import AutoClicker, ChangeUserList
from Scripts.database import PlayerDataBase
from Scripts.threads import ProgressBarThread, ACBlinkThread
from Scripts.screens import MainScreen, InstructionsScreen, SecondScreen, GameScreen, ScoreScreen

# region init
# load assets and songs
ROOT_PATH = os.path.dirname(os.path.abspath(__file__))
SONGS_PATH = os.path.join(ROOT_PATH, "Songs")
ASSET_PATH = os.path.join(ROOT_PATH, "Assets")


# endregion


class MyWindow(QMainWindow):
    def __init__(self, width, height, title, DEBUG):
        super().__init__()
        full_screen_size = pyautogui.size()
        # self.simActive = False # simulate camera locations vs. use true locations
        self.simActive = True
        self.width = width
        self.height = height
        self.setGeometry(int((full_screen_size[0] - width) / 2), int((full_screen_size[1] - height) / 2), width, height)

        # Database
        self.playerDataBase = PlayerDataBase()
        self.score = -1

        # Set screen title, icon and application style
        self.setWindowTitle(title)
        self.setWindowIcon(QIcon(os.path.join(ASSET_PATH, 'green button.png')))

        # Define the different screens
        self.active_screen = None
        main_screen = MainScreen(self, "Main Menu Screen")
        instructions_screen = InstructionsScreen(self, "Instructions Screen")
        secondary_screen = SecondScreen(self, "Secondary Menu Screen")
        game_screen = GameScreen(self, "Game Screen")
        score_screen = ScoreScreen(self, "Score Screen")

        self.screens = {'main': main_screen,
                        'instructions': instructions_screen,
                        'second': secondary_screen,
                        'game': game_screen,
                        'score': score_screen
                        }

        # song parameters and definitions
        self.song_name = ""
        self.audio = None
        self.total_song_length = -1
        self.bpm = -1
        self.song_path = ""
        self.note_times = None
        self.notes = []

        # game parameters
        self.running = False
        self.first_run = True
        self.paused = False
        self.DEBUG = DEBUG
        self.number_of_robots = 1
        self.activate_screen("main")  # Load the main screen on game startup
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
    def btn_move_to_song_choice_screen(self):
        if self.playerDataBase.firstName == "" or self.playerDataBase.lastName == "":
            self.btn_change_user()
            if self.playerDataBase.firstName == "" or self.playerDataBase.lastName == "":
                return

        pygame.mixer.quit()
        self.activate_screen("second")

    def btn_return_to_main(self):
        self.activate_screen("main")

    def btn_change_user(self):
        self.user_list = ChangeUserList(self, self.playerDataBase)
        self.user_list.show()

    def btn_instructions(self):
        self.activate_screen("instructions")

    def btn_exit(self):
        button_reply = QMessageBox.question(self, 'PyQt5 message', "Exit the application?",
                                            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if button_reply == QMessageBox.Yes:
            sys.exit()

    def btn_pause_game(self):
        if self.paused:
            pygame.mixer.music.unpause()
        else:
            pygame.mixer.music.pause()

        self.paused = not self.paused

    def btn_stop_game(self):
        self.activate_screen("score")
        pygame.mixer.music.fadeout(2000)

        self.playerDataBase.update_score(self.song_name,
                                         self.score)  # attempt to update based on currently active player
        self.reset()

    def btn_choose_song(self):
        self.song_name = "song2"

        self.song_path = os.path.join(SONGS_PATH, "One Kiss_cropped.mp3")
        self.audio = MP3(self.song_path)
        self.bpm = 123.85  # One kiss BPM - using audacity we got 123, updated a bit

        self.get_autoclicker_notes('one kiss')
        if self.first_run:
            self.first_run = False
            self.initialize_game_widgets()

        self.start_game()

    # endregion

    def get_autoclicker_notes(self, song_name):
        beats_per_sec = self.bpm / 60  # 2
        dt = 1 / beats_per_sec  # every this many seconds
        self.total_song_length = self.audio.info.length
        self.note_times = np.arange(0, self.total_song_length, dt)  # 1.9sec, 3.8sec, etc.

        iters = range(len(self.note_times))[1:]  # get rid of note 0
        notes = []

        if song_name == 'one kiss':
            # iterations to play on, assuming each iteration is the shortest beat
            # up to 430 iterations
            easyUp = ACBlinkThread.easyUpLevel
            ac1_notes = [easyUp * iter for iter in iters if
                         iter < (iters[-1] // easyUp) + 1]  # notes only till song end
            ac2_notes = [easyUp * iter for iter in iters if iter < (iters[-1] // easyUp) + 1]
            ac3_notes = [easyUp * iter for iter in iters if iter < (iters[-1] // easyUp) + 1]
            notes.append(ac1_notes)
            notes.append(ac2_notes)
            notes.append(ac3_notes)
        self.notes = notes

    def initialize_game_widgets(self):
        '''
        Create the threads and the autoclickers for the first time
        '''
        self.progressBarThread = ProgressBarThread(self, pygame.mixer.music)
        self.progressBarThread.progress.connect(self.update_progress_bar)

        # Create AutoClickers and their animations (the animations MUST belong to the win for some reason)
        if not self.simActive:
            rclpy.init()

        self.screens["game"].autoclickers = []

        clicker_1 = AutoClicker(self, 0)
        self.screens["game"].autoclickers.append(clicker_1)
        self.screens["game"].autoClickerAnims.append(clicker_1.animation)
        if self.number_of_robots >= 2:
            clicker_2 = AutoClicker(self, 1)
            self.screens["game"].autoclickers.append(clicker_2)
            self.screens["game"].autoClickerAnims.append(clicker_2.animation)
        if self.number_of_robots >= 3:
            clicker_3 = AutoClicker(self, 2)
            self.screens["game"].autoclickers.append(clicker_3)
            self.screens["game"].autoClickerAnims.append(clicker_3.animation)


    # region thread events

    def start_game(self):
        self.running = True
        for clicker in self.screens["game"].autoclickers:
            clicker.start_threads()

        if self.simActive:
            print("Simulators are active!")
        else:
            print("Real life data!")

            msg = "Please place the AutoClickers in their initial positions."
            # button_reply = QMessageBox.question(self, 'PyQt5 message', msg,
            #                                     QMessageBox.Close | QMessageBox.Cancel, QMessageBox.Cancel)
            #
            # if button_reply != QMessageBox.Close:
            #     return
        self.activate_screen("game")

        pygame.mixer.init()
        pygame.mixer.music.load(self.song_path)
        pygame.mixer.music.play()

        self.progressBarThread.start()

    def update_progress_bar(self, progress):
        self.screens['game'].progressBars[0].setValue(progress)

        self.score = 0
        for clicker in self.screens["game"].autoclickers:
            self.score += 15 * clicker.score

        self.screens['game'].show()

    def reset(self):
        self.running = False
        self.score = 0
        self.progressBarThread.stop()
        for clicker in self.screens["game"].autoclickers:
            clicker.reset()

        if not self.simActive:
            rclpy.shutdown()

    # endregion


def launch_game():
    app = QApplication(sys.argv)
    fullscreen = True
    if fullscreen:
        size = pyautogui.size()
        width, height = size[0], size[1]
    else:
        width, height = 1600, 900

    try:
        MyWindow(width, height, "Clicker Hero", DEBUG=True)
        sys.exit(app.exec_())
    except Exception as e:
        print(e)
        exit(1)


if __name__ == "__main__":
    launch_game()
