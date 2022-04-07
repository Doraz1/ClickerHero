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
from Scripts.myCustomWidgets import AutoClicker
from Scripts.database import PlayerDataBase, ListGenerator
from Scripts.threads import ProgressBarThread
from Scripts.songengine import SongEngine
from Scripts.screens import MainScreen, CustomizationScreen, InstructionsScreen, SecondScreen, GameScreen, ScoreScreen

# region Define paths
# load assets and songs
ROOT_PATH = os.path.dirname(os.path.abspath(__file__))
SONGS_PATH = os.path.join(ROOT_PATH, "Songs")
ASSET_PATH = os.path.join(ROOT_PATH, "Assets")
# endregion


class MyWindow(QMainWindow):
    def __init__(self, width, height, title):
        """
        Create main window, screens, database, game parameters.
        Handles movement between screens and button presses.
        Song buttons create the song engine and the clicker notes.
        Once notes are set, the clicker threads are created and launched.

        :param width:
        :param height:
        :param title:
        """
        super().__init__()

        'Create main window'
        full_screen_size = pyautogui.size()
        self.width = width
        self.height = height
        self.setGeometry(int((full_screen_size[0] - width) / 2), int((full_screen_size[1] - height) / 2), width, height)

        'Create player database handler'
        self.db = PlayerDataBase()
        self.score = -1

        'Set screen title, icon and application style'
        self.setWindowTitle(title)
        self.setWindowIcon(QIcon(os.path.join(ASSET_PATH, 'green button.png')))

        self.simActive = False
        # self.simActive = True
        self.DEBUG = False
        self.active_screen = None

        main_screen = MainScreen(self, "Main Menu Screen")
        customization_screen = CustomizationScreen(self, "Customization Screen")
        instructions_screen = InstructionsScreen(self, "Instructions Screen")
        secondary_screen = SecondScreen(self, "Song choice Screen")
        game_screen = GameScreen(self, "Game Screen")
        score_screen = ScoreScreen(self, "Score Screen")

        self.screens = {'main': main_screen,
                        'customization': customization_screen,
                        'instructions': instructions_screen,
                        'song_choice': secondary_screen,
                        'game': game_screen,
                        'score': score_screen
                        }

        'Game parameters'
        self.rclpy_initialized = False
        self.running = False
        self.first_run = True
        self.paused = False
        self.number_of_robots = 1
        self.activate_screen("main")  # Load the main screen on game startup
        self.show()  # render

        'Song parameters'
        self.song_t0 = -1
        self.song_engine = None  # will be created once the user selects a song

        'Miscellaneous'
        self.user_list = None
        self.prog_bar_thread = None

    # region Screen navigation buttons
    def refresh_screen(self):
        self.active_screen.show()

    def activate_screen(self, screenName):
        if self.active_screen is not None:
            self.active_screen.hide()

        for screen_name, screen in self.screens.items():
            if screen_name == screenName:
                screen.show()
                self.active_screen = screen
                if self.DEBUG:
                    print(f"currently active: {screen.name}")

    def btn_move_to_song_choice_screen(self):
        if not self.db.playerLoaded:
            self.btn_change_user()
            if not self.db.playerLoaded:
                return

        self.activate_screen("song_choice")

    def btn_move_to_customization_screen(self):
        self.activate_screen("customization")

    def btn_return_to_main(self):
        self.activate_screen("main")

    def btn_change_user(self):
        self.user_list = ListGenerator(self, self.db)
        self.user_list.make_change_user_list()
        self.user_list.show()

    def btn_change_difficulty(self):
        self.user_list = ListGenerator(self, self.db)
        self.user_list.make_list("difficulty")
        self.user_list.show()

    def btn_instructions(self):
        self.activate_screen("instructions")

    def btn_exit(self):
        button_reply = QMessageBox.question(self, 'PyQt5 message', "Exit the application?",
                                            QMessageBox.Yes | QMessageBox.No, defaultButton=QMessageBox.Yes)

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

        self.db.update_score(self.song_engine.song_name,
                             self.score)  # attempt to update based on currently active player
        self.reset()
    # endregion

    'Song buttons'
    def btn_play_one_kiss(self):
        song_name = "one_kiss"
        song_path = os.path.join(SONGS_PATH, "One Kiss_cropped.mp3")

        self.song_engine = SongEngine(song_name, song_path)
        self.start_game()

    def btn_play_cant_help_falling_in_love(self):
        song_name = "cant_help_falling_in_love"
        song_path = os.path.join(SONGS_PATH, "Can't Help Falling In Love.mp3")

        self.song_engine = SongEngine(song_name, song_path)
        self.start_game()

    'Thread events'
    def start_game(self):
        """
        Initialize robots, start all robot threads, start the mustic and move to game screen.

        :return:
        """
        self.running = True

        if self.first_run:
            self.initialize_game_widgets()



        pygame.mixer.init()
        pygame.mixer.music.load(self.song_engine.song_path)

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

        for clicker in self.screens["game"].autoclickers:
            clicker.start_threads()

        self.prog_bar_thread.start()

        pygame.mixer.music.play(start=self.song_engine.start_from_second)

    def initialize_game_widgets(self):
        """
        Create the threads and the AutoClickers for the first time

        :return:
        """
        self.notes = self.song_engine.notes
        self.moves = self.song_engine.moves

        if not self.simActive and not self.rclpy_initialized:
            rclpy.init()
            self.rclpy_initialized = True

        # Create AutoClickers and their animations (the animations MUST belong to the win for some reason)
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

        self.prog_bar_thread = ProgressBarThread(self, pygame.mixer.music)
        self.prog_bar_thread.progress.connect(self.update_progress_bar)

    def update_progress_bar(self, progress):
        self.screens['game'].progressBars[0].setValue(progress)

        self.score = 0
        for clicker in self.screens["game"].autoclickers:
            self.score += 15 * clicker.score

        self.screens['game'].show()

    def reset(self):
        self.running = False
        self.score = 0
        self.prog_bar_thread.stop()
        for clicker in self.screens["game"].autoclickers:
            clicker.reset()

        if not self.simActive:
            rclpy.shutdown()
            self.rclpy_initialized = False


def launch_game():
    app = QApplication(sys.argv)
    fullscreen = True
    if fullscreen:
        size = pyautogui.size()
        width, height = size[0], size[1]
    else:
        width, height = 1600, 900

    try:
        MyWindow(width, height, "Clicker Hero")
        sys.exit(app.exec_())
    except Exception as e:
        print(e)
        exit(1)



if __name__ == "__main__":
    launch_game()
