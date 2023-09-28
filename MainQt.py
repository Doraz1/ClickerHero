import sys
import time
import os

from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QMessageBox,
    QWidget,
)
from PyQt5.QtGui import QIcon
import pyautogui
import pygame.mixer
from Scripts.database import PlayerDataBase, ListGenerator
from Scripts.gameengines import GameEngineBongos, GameEngineTrafficLight, GameEngineSimonSays
from Scripts.gameengines import RosHandler, RobotHandler
from Scripts.screens import *
from Scripts.threads import ProgressBarThreadBongos, ProgressBarThreadTL, ProgressBarThreadSS
from PyQt5.QtMultimedia import QMediaContent, QMediaPlayer
from Scripts.Video import VideoPlayer
from threading import Thread
from Scripts.params import get_params
from Scripts.enums import Screens, Songs, Games
import Scripts.database as dbLib

class BtnHandler:
    def __init__(self, win, params):
        self.win = win
        self.params = params

        self.firstTutorial = True

    'Screen navigation buttons'
    def btn_move_to_customization_screen(self):
        self.win.screenHandler.activate_screen(Screens.CUSTOMIZATION)

    def btn_return_to_main(self):
        self.win.screenHandler.activate_screen(Screens.MAIN)

    def btn_play_KB_tutorial(self):
        self.__play_tutorial(Games.KING_OF_THE_BONGOS)

    def btn_play_TL_tutorial(self):
        self.__play_tutorial(Games.TRAFFIC_LIGHT)

    def btn_play_SS_tutorial(self):
        self.__play_tutorial(Games.SIMON_SAYS)

    def __play_tutorial(self, game_enum):
        name = game_enum.value
        print(f"Playing tutorial of {name}")
        path = self.params[f"{name} tutorial path"]

        # if self.win.player is None:
        W, H = self.params["window_width"], self.params["window_height"]

        self.win.player = VideoPlayer(parent=self.win)
        self.win.player.resize(W, H)

        self.win.player.setWindowTitle(f"{name} tutorial")
        t = Thread(target=self.win.player.play, args=(path, self.firstTutorial))
        t.start()

        if self.firstTutorial:
            'Repeat'
            self.firstTutorial = False
            while (self.win.player.isActive):
                print("waiting for previous run to end (Video player)")
                time.sleep(0.1)
            self.__play_tutorial(game_enum)


    def btn_change_user(self):
        self.win.user_list = ListGenerator(self.win, self.win.db)
        self.win.user_list.make_change_user_list()
        self.win.user_list.show()

    def btn_change_difficulty(self):
        self.win.user_list = ListGenerator(self.win, self.win.db)
        self.win.user_list.make_list("difficulty")
        self.win.user_list.show()

    def btn_instructions(self):
        self.win.screenHandler.activate_screen(Screens.INSTRUCTIONS)

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
        self.win.gameEngine.stop_game()

    'Song buttons'
    def btn_move_to_song_choice_screen(self):

        if self.params["check_communication"] and self.win.robotHandler.com_error:
            msg = QMessageBox.about(self.win, "Communication error",
                                    "Not communicating with one of the robots. Please ensure communication and try again \n"
                                    "(Did you turn them off and on again and wait enough time?)")
            return

        self.win.active_game = Games.KING_OF_THE_BONGOS

        'Ensure a player from the database is selected'
        if not self.win.db.playerLoaded:
            self.btn_change_user()
            return

        self.win.screenHandler.activate_screen(Screens.SONG_CHOICE)

    # King of the Bongos songs
    def btn_play_kol_hakavod(self):
        song_name = "kol_hakavod"
        self.win.gameEngine = GameEngineBongos(self.win, self.params, start_from_sec=self.params["KB_start_from_sec"], song_name=song_name)
        self.win.start_game()

    def btn_play_pgisha_bamiluim(self):
        song_name = "pgisha_bamiluim"
        self.win.gameEngine = GameEngineBongos(self.win, self.params, start_from_sec=self.params["KB_start_from_sec"], song_name=song_name)
        self.win.start_game()
    def btn_play_karnaval_banachal(self):
        song_name = "karnaval_banachal"

        self.win.gameEngine = GameEngineBongos(self.win, self.params, start_from_sec=self.params["KB_start_from_sec"], song_name=song_name)
        self.win.start_game()


    def game_init_checks(self):
        'Ensure a player from the database is selected'
        check_ok = True
        if hasattr(self.win, "robotHandler"):
            if self.win.robotHandler.com_error:
                msg = QMessageBox.about(self.win, "Communication error",
                                        "Comminication with robot failed. Please ensure communication and try again \n"
                                        "(Did you turn them on and wait enough time?)")
                check_ok = False
        else:

            check_ok = False

        if self.win.gameEngine is not None:
            msg = QMessageBox.about(self.win, "Game reset error",
                                    "Previous game still active. \n"
                                    "Please wait a few seconds and try again.")
            check_ok = False

        if not self.win.db.playerLoaded:
                self.btn_change_user()
                check_ok = False
        return check_ok
    def btn_move_to_traffic_light_game_screen(self):
        check_ok = self.game_init_checks()
        if not check_ok:
            return

        self.win.active_game = Games.TRAFFIC_LIGHT

        print("now using TL game engine")
        self.win.gameEngine = GameEngineTrafficLight(self.win, self.params, start_from_sec=313.5, song=Songs.TRAFFIC_LIGHT_MUSIC)

        self.win.start_game()
        self.win.screenHandler.activate_screen(Screens.TRAFFIC_LIGHT)


    def btn_move_to_simon_says_game_screen(self):
        check_ok = self.game_init_checks()
        if not check_ok:
            return

        self.win.active_game = Games.SIMON_SAYS

        print("now using SS game engine")
        self.win.gameEngine = GameEngineSimonSays(self.win, self.params, start_from_sec=0, song=Songs.SIMON_SAYS_MUSIC)
        # self.win.gameEngine = GameEngineSimonSays(self.win, self.params, start_from_sec=2568, song_name=song_name)  # march song

        self.win.start_game()
        self.win.screenHandler.activate_screen(Screens.SIMON_SAYS)


class ScreenHandler:
    def __init__(self, win, params):
        self.win = win
        self.params = params
        self.active_screen = None
        self.screen_dict = self.create_screens()
        self.prog_bar_thread = None

        self.resetting_game = False
        self.reset_in_progress = False

    def create_screens(self):
        main_screen = MainScreen(self.win, self.params, "Main Menu Screen")
        customization_screen = CustomizationScreen(self.win, self.params, "Customization Screen")
        instructions_screen = InstructionsScreen(self.win, self.params, "Instructions Screen")
        song_choice_screen = SongChoiceScreen(self.win, self.params, "Song choice Screen")
        bongo_game_screen = BongoGameScreen(self.win, self.params, "Bongo game Screen")
        traffic_light_game_screen = TrafficLightGameScreen(self.win, self.params, "Traffic Light Screen")
        simon_says_game_screen = TrafficLightGameScreen(self.win, self.params, "Simon Says Screen")
        score_screen = ScoreScreen(self.win, self.params, "Score Screen")

        screen_dict = {Screens.MAIN.value: main_screen,
                       Screens.CUSTOMIZATION.value: customization_screen,
                       Screens.INSTRUCTIONS.value: instructions_screen,
                       Screens.SONG_CHOICE.value: song_choice_screen,
                       Screens.KING_OF_THE_BONGOS.value: bongo_game_screen,
                       Screens.TRAFFIC_LIGHT.value: traffic_light_game_screen,
                       Screens.SIMON_SAYS.value: simon_says_game_screen,
                       Screens.SCORE.value: score_screen
                       }

        return screen_dict

    def activate_screen(self, screen_enum):
        if self.active_screen is not None:
            self.active_screen.hide()

        screen_name = screen_enum.value
        print(f"activating {screen_name} screen")
        des_screen = self.screen_dict[screen_name]
        self.active_screen = des_screen
        self.win.screenHandler.refresh_screen()

    def start_prog_bar_bongos(self):
        self.prog_bar_thread = ProgressBarThreadBongos(self.win, pygame.mixer.music)
        self.prog_bar_thread.progress.connect(self.update_progress_bar_bongos)

        self.prog_bar_thread.start()

    def update_progress_bar_bongos(self, progress):
        bongo_screen = self.screen_dict[Screens.KING_OF_THE_BONGOS.value]
        bongo_screen.progress_bar.setValue(progress)

        self.refresh_screen()

    def stop_prog_bar_bongos(self):
        self.prog_bar_thread.progress.disconnect(self.update_progress_bar_bongos)
        self.prog_bar_thread.stop()


    def start_prog_bar_TL(self):
        # print("Creating and starting progress bar thread")
        self.prog_bar_thread = ProgressBarThreadTL(self.win)
        self.prog_bar_thread.progress.connect(self.update_progress_bar_TL)

        self.prog_bar_thread.start()

    def update_progress_bar_TL(self, progress):
        TL_screen = self.screen_dict[Screens.TRAFFIC_LIGHT.value]
        TL_screen.progress_bar.setValue(progress)

        self.refresh_screen()

    def stop_prog_bar_TL(self):
        print("Disconnecting prog bar thread of TL")
        self.prog_bar_thread.progress.disconnect(self.update_progress_bar_TL)
        self.prog_bar_thread.stop()


    def start_prog_bar_SS(self):
        # print("Creating and starting progress bar thread")
        self.prog_bar_thread = ProgressBarThreadSS(self.win)
        self.prog_bar_thread.progress.connect(self.update_progress_bar_SS)

        self.prog_bar_thread.start()

    def update_progress_bar_SS(self, progress):
        SS_screen = self.screen_dict[Screens.SIMON_SAYS.value]
        SS_screen.progress_bar.setValue(progress)

        self.refresh_screen()

    def stop_prog_bar_SS(self):
        self.prog_bar_thread.progress.disconnect(self.update_progress_bar_SS)
        self.prog_bar_thread.stop()

    def refresh_screen(self):
        # print("refreshing screen")
        self.render_screen()
        if self.resetting_game and not self.reset_in_progress:
            self.resetting_game = False
            tss = time.time() - self.win.gameEngine.t0
            if tss < 1:
                'race condition - ignore and play as usual'
                return

            print(f"Time since start: {tss}")

            self.reset_in_progress = True

            print("Resetting game through a refresh")
            self.win.reset()
            self.reset_in_progress = False

    def render_screen(self):
        self.win.show()  # render main window
        self.active_screen.show()


#endregion


class MyWindow(QMainWindow):
    def __init__(self,
                 params,
                 database: PlayerDataBase
                 ):

        """
        Create main window, screens, database, game parameters.
        Song buttons create the song engine and the clicker notes.
        Once notes are set, the clicker threads are created and launched.

        :param width:
        :param height:
        :param title:
        """
        super().__init__()

        'Create main window'
        self.params = params
        self.db = database
        self.player = None
        self.width = params["window_width"]
        self.height = params["window_height"]
        self.config_window(params["window_title"])

        print("creating button, screen and robot handlers")
        self.btnHandler = BtnHandler(self, self.params)
        self.screenHandler = ScreenHandler(self, self.params)

        self.rosHandler = RosHandler(self)
        self.robotHandler = RobotHandler(self, self.params, num_robots=self.params["num_robots"])
        self.simActive = params["sim_active"]

        'Game parameters'
        self.active_game = None

        self.first_run = True
        self.paused = False

        self.gameEngine = None  # will be created once the user selects a song

        'Miscellaneous'
        self.user_list = None
        self.screenHandler.activate_screen(Screens.MAIN)

    def config_window(self, title):
        """ set window geometry, title and icon"""
        full_screen_W, full_screen_H = self.params["window_width"], self.params["window_height"]

        self.setGeometry(int((full_screen_W - self.width) / 2), int((full_screen_H - self.height) / 2), self.width, self.height)
        self.setWindowTitle(title)
        self.setWindowIcon(QIcon(os.path.join(self.params["ASSET_PATH"], 'green button.png')))

    def start_game(self):
        """
        Initialize robots, start all robot threads, start the music and move to game screen.

        :return:
        """

        # msg = "Please place the AutoClickers in their initial positions."
            # button_reply = QMessageBox.question(self, 'PyQt5 message', msg,
            #                                     QMessageBox.Close | QMessageBox.Cancel, QMessageBox.Cancel)
            #
            # if button_reply != QMessageBox.Close:
            #     return

        self.gameEngine.start()

    def reset(self):

        if self.robotHandler.com_error:
            msg = QMessageBox.about(self.gameEngine.win, "Communication error",
                                    "Game ended since there's a communication error with one of the robots. \n")
        else:
            print("Resetting window since Reached end of practice.")

        print("Updating score in DB")
        RTs = list(self.gameEngine.scoreHandler.reaction_times.values())  # dont care about robot index
        if self.params["debug_database"]:
            print(f"Game raw data to be added in db: {RTs}")
        notes_hit = []
        if self.active_game == Games.KING_OF_THE_BONGOS:
            for robo_inputs in RTs:
                for input in robo_inputs:
                    notes_hit.append(input)
        elif self.active_game == Games.TRAFFIC_LIGHT:
            for robo_inputs in RTs:
                for input in robo_inputs:
                    notes_hit.append(input)
        elif self.active_game == Games.SIMON_SAYS:
            for robo_inputs in RTs:
                for input in robo_inputs:
                    notes_hit.append(input)

        print(f"Game raw data to be added in db: {notes_hit}")
        self.db.update_db_score_and_RT(self.gameEngine.song_name, self.gameEngine.scoreHandler.score, reaction_time=notes_hit)  # attempt to update based on currently active player


        print("Stopping game engine - first the window screen handler (progress bar disconnect), \n"
              "then via the super() the ros handler and the sound handler\n"
              "and finally the score handler")
        self.gameEngine.stop()

        self.screenHandler.activate_screen(Screens.SCORE)
        'Disable "return to main" till game ended finishes'
        self.gameEngine.soundHandler.play_game_ended()

        print("Active game and gameEngine = None")
        self.active_game = None
        self.gameEngine = None


def launch_app():
    app = QApplication(sys.argv)

    params = get_params()
    try:
        'Create player database handler'
        db = PlayerDataBase()
        rows = db.load_all()
        for row in rows:
            print(f"db row: {row}")
        # db = dbLib.testDB()

        print("launching main window")
        MyWindow(params, db)
        sys.exit(app.exec_())
    except Exception as e:
        print(e)
        exit(1)

if __name__ == "__main__":
    launch_app()
