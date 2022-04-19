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
from Scripts.songengine import NormalSongEngine, ComboSongEngine
from Scripts.screens import *
import numpy as np

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
        self.score = 0

        'Set screen title, icon and application style'
        self.setWindowTitle(title)
        self.setWindowIcon(QIcon(os.path.join(ASSET_PATH, 'green button.png')))

        self.simActive = False
        # self.simActive = True
        self.DEBUG = False
        self.active_screen = None

        'Game parameters'
        self.active_game = None
        self.rclpy_initialized = False
        self.running = False
        self.first_run = True
        self.paused = False
        self.number_of_robots = 1
        self.time_note_played = self.number_of_robots*[-1]
        self.reaction_times = {0: [], 1: [], 2: []}
        self.btnHandler = BtnHandler(self)

        main_screen = MainScreen(self, "Main Menu Screen")
        customization_screen = CustomizationScreen(self, "Customization Screen")
        instructions_screen = InstructionsScreen(self, "Instructions Screen")
        secondary_screen = SecondScreen(self, "Song choice Screen")
        combo_game_screen = ComboGameScreen(self, "Combo game Screen")
        game_screen = NormalGameScreen(self, "Game Screen")
        score_screen = ScoreScreen(self, "Score Screen")

        self.screens = {'main': main_screen,
                        'customization': customization_screen,
                        'instructions': instructions_screen,
                        'song_choice': secondary_screen,
                        'combo_game': combo_game_screen,
                        'game': game_screen,
                        'score': score_screen
                        }
        self.activate_screen("main")  # Load the main screen on game startup
        self.show()  # render

        'Song parameters'
        self.song_t0 = -1
        self.song_engine = None  # will be created once the user selects a song

        'Miscellaneous'
        self.user_list = None
        self.prog_bar_thread = None

    def activate_screen(self, screenName):
        if self.active_screen is not None:
            self.active_screen.hide()

        for screen_name, screen in self.screens.items():
            if screen_name == screenName:
                screen.show()
                self.active_screen = screen
                if self.DEBUG:
                    print(f"currently active: {screen.name}")

    def start_game(self):
        """
        Initialize robots, start all robot threads, start the mustic and move to game screen.

        :return:
        """
        self.running = True

        self.initialize_game_widgets()

        if self.simActive:
            print("Simulators are active!")
        else:
            print("Real life data!")

            # msg = "Please place the AutoClickers in their initial positions."
            # button_reply = QMessageBox.question(self, 'PyQt5 message', msg,
            #                                     QMessageBox.Close | QMessageBox.Cancel, QMessageBox.Cancel)
            #
            # if button_reply != QMessageBox.Close:
            #     return
        self.activate_screen("game")

        for clicker in self.screens["game"].autoclickers:
            clicker.start_threads()

        if self.active_game == 'normal game':
            self.prog_bar_thread.start()

        pygame.mixer.music.play(start=self.song_engine.start_from_second)

    def initialize_game_widgets(self):
        """
        Create the threads and the AutoClickers for the first time

        :return:
        """
        if not self.simActive and not self.rclpy_initialized:
            rclpy.init()
            self.rclpy_initialized = True

        # Create AutoClickers and their animations (the animations MUST belong to the win for some reason)
        self.screens["game"].autoclickers = []

        for i in range(self.number_of_robots):
            if i >= 3:
                break
            clicker = AutoClicker(self, i)
            self.screens["game"].autoclickers.append(clicker)
            self.screens["game"].autoClickerAnims.append(clicker.animation)

        if self.active_game == 'normal game':
            self.prog_bar_thread = ProgressBarThread(self, pygame.mixer.music)
            self.prog_bar_thread.progress.connect(self.update_progress_bar)
            difficulty = self.db.get_parameter("difficulty")  # blink difficulty - either 1, 2, or 3
        elif self.active_game == 'combo game':
            difficulty = 1

        for clicker in self.screens["game"].autoclickers:
            clicker.pubThread.set_difficulty(difficulty)

        pygame.mixer.init()
        pygame.mixer.music.load(self.song_engine.song_path)

    def update_progress_bar(self, progress):
        self.active_screen.progress_bar.setValue(progress)
        self.refresh_screen()

    def update_score(self, clicker_ind, click_time):
        note_time = self.time_note_played[clicker_ind]
        reaction_time = click_time - note_time
        self.reaction_times[clicker_ind].append(reaction_time)
        # print(f"logged reaction time: {round(reaction_time, 2)}[s] since note time is {note_time} and clicktime is {click_time}")
        if self.active_game == 'normal game':
            self.score += int(10 / reaction_time)  # score based on reaction time
        elif self.active_game == 'combo game':
            self.score += 1
            difficulty_incr_thresh = 1
            required_difficulty = 1 + int(self.score / difficulty_incr_thresh)
            print(f"curr score: {self.score} req dif: {required_difficulty}")
            if self.song_engine.difficulty < required_difficulty:
                self.increase_difficulty()

        self.refresh_screen()

    def increase_difficulty(self):
        for clicker in self.screens["game"].autoclickers:
            clicker.pubThread.increase_difficulty()

        self.song_engine.set_difficulty(self.screens["game"].autoclickers[0].pubThread.difficulty)

    def decrease_difficulty(self):
        if self.score > self.song_engine.max_combo:
            self.song_engine.max_combo = self.score
            print("Reached new combo highscore!")

        self.score = 0
        for clicker in self.screens["game"].autoclickers:
            clicker.pubThread.decrease_difficulty()

        self.song_engine.set_difficulty(self.screens["game"].autoclickers[0].pubThread.difficulty)

    def refresh_screen(self):
        self.active_screen.show()

    def reset(self):
        self.running = False
        self.reset_score()

        pygame.mixer.music.fadeout(2000)

        if self.active_game == 'combo game':
            pass
        elif self.active_game == 'normal game':
            self.prog_bar_thread.stop()
            pass

        for clicker in self.screens["game"].autoclickers:
            clicker.reset()

        if not self.simActive and self.rclpy_initialized:
            rclpy.shutdown()
            self.rclpy_initialized = False

        self.active_game = None

    def reset_score(self):
        self.reaction_times = {0: [], 1: [], 2: []}
        self.score = 0


class BtnHandler:
    def __init__(self, win):
        self.win = win

    'Screen navigation buttons'


    def __move_to_game(self):
        '''
        move to game screen
        normal - move to song choice
        combo - move to gym

        :return:
        '''
        'Ensure a player from the database is selected'
        if not self.win.db.playerLoaded:
            self.btn_change_user()
            return

        if self.win.active_game == 'normal game':
            screenName = "song_choice"
        elif self.win.active_game == 'combo game':
            screenName = "combo_game"
            self.win.start_game()

        self.win.activate_screen(screenName)


    def btn_move_to_customization_screen(self):
        self.win.activate_screen("customization")

    def btn_return_to_main(self):
        self.win.activate_screen("main")

    def btn_change_user(self):
        self.win.user_list = ListGenerator(self.win, self.win.db)
        self.win.user_list.make_change_user_list()
        self.win.user_list.show()

    def btn_change_difficulty(self):
        self.win.user_list = ListGenerator(self.win, self.win.db)
        self.win.user_list.make_list("difficulty")
        self.win.user_list.show()

    def btn_instructions(self):
        self.win.activate_screen("instructions")

    def btn_exit(self):
        button_reply = QMessageBox.question(self.win, 'PyQt5 message', "Exit the application?",
                                            QMessageBox.Yes | QMessageBox.No, defaultButton=QMessageBox.Yes)

        if button_reply == QMessageBox.Yes:
            sys.exit()

    def btn_pause_game(self):
        if self.win.paused:
            pygame.mixer.music.unpause()
        else:
            pygame.mixer.music.pause()

        self.win.paused = not self.win.paused

    def btn_stop_game(self):
        self.win.activate_screen("score")
        self.win.db.update_score(self.win.song_engine.song_name, self.win.score)  # attempt to update based on currently active player
        self.win.reset()

    'Song buttons'
    def btn_move_to_song_choice_screen(self):
        self.win.active_game = 'normal game'
        self.__move_to_game()

    def btn_play_one_kiss(self):
        song_name = "one_kiss"

        self.win.song_engine = NormalSongEngine(SONGS_PATH, song_name)
        self.win.start_game()

    def btn_play_cant_help_falling_in_love(self):
        song_name = "cant_help_falling_in_love"

        self.win.song_engine = NormalSongEngine(SONGS_PATH, song_name)
        self.win.start_game()

    def btn_move_to_combo_game_screen(self):
        self.win.active_game = 'combo game'

        song_name = "upbeat_trans_music"
        # song_name = "cant_help_falling_in_love"
        self.win.song_engine = ComboSongEngine(SONGS_PATH, song_name)
        self.__move_to_game()


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
