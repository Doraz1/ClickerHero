import os

import numpy as np
from PyQt5.QtWidgets import (
    QLabel,
    QPushButton,
    QProgressBar,
)
from PyQt5.QtMultimediaWidgets import QVideoWidget
import pyqtgraph
from PyQt5.QtGui import QPixmap, QFont
from PyQt5.QtCore import Qt
from Scripts.gameengines import GameEngineBongos, GameEngineTrafficLight, GameEngineSimonSays


class Screen:
    transparent_color = Qt.transparent

    # Label style
    title_lbl_width = 1500
    title_lbl_height = 140
    title_lbl_margin_y = 35
    title_lbl_text_color = "(255, 255, 255, 1.0)"
    title_lbl_bg_color = "(0, 0, 0, 0.2)"
    title_lbl_text_font = QFont('Serif', 85, QFont.DemiBold)

    lbl_width = 200
    lbl_height = 120
    lbl_margin_y = 5
    lbl_text_color = "(255, 255, 255, 1.0)"
    lbl_bg_color = "(0, 0, 0, 0.2)"
    lbl_text_font = QFont('Serif', 45, QFont.DemiBold)

    # Button style
    btn_color = "(100, 100, 255, 0.8)"
    btn_initial_y = 280
    btn_width = 760
    btn_height = 140
    btn_margin_y = 50
    btn_text_color = "(240, 240, 240, 1.0)"
    btn_bg_color = "(100, 100, 255, 0.9)"
    btn_text_font = QFont('Serif', 85, QFont.DemiBold)
    btn_opacity = 30

    #start button style
    start_btn_bg_color = "(150, 180, 255, 0.9)"
    start_btn_width = 615
    start_btn_height = 170
    start_btn_margin_x = 0
    start_btn_dx = 1

    # progress bar style
    prog_bar_bg_color = "(150, 120, 180, 1.0)"
    prog_bar_width = 1100
    prog_bar_height = 150
    prog_bar_margin_y = 10
    prog_bar_font = QFont('Serif', 1, QFont.Light)

    def __init__(self, win, params, name):
        self.win = win
        self.params = params
        self.name = name
        self.labels = []
        self.scoreButtonLabels = []
        self.buttons = []
        self.autoclickers = []
        self.autoClickerAnims = []
        self.progress_bar = None

        WALLPAPER_PATH = os.path.join(self.params["ASSET_PATH"], 'Wallpapers')
        self.bg_dict = {
            "main": os.path.join(WALLPAPER_PATH, "main_bg.jpg"),
            "bongos": os.path.join(WALLPAPER_PATH, "violin_bg.jpg"),
            "TL": os.path.join(WALLPAPER_PATH, "guitar_bg.jpg"),
            "SS": os.path.join(WALLPAPER_PATH, "main_bg.jpg"),
            "score": os.path.join(WALLPAPER_PATH, "sky_bg.jpg"),
        }

    def show(self):
        self.win.setWindowTitle(self.name)

        for lbl in self.labels:
            lbl.show()

        for btn in self.buttons:
            btn.show()



    def hide(self):
        for lbl in self.labels:
            lbl.hide()

        for btn in self.buttons:
            btn.hide()

    def create_background(self, label, image_path):
        pix = QPixmap(image_path)
        pixmap_resized = pix.scaled(self.win.width, self.win.height, Qt.KeepAspectRatio)
        # pixmap_resized = pix.scaledToWidth(self.win.width)
        label.setPixmap(pixmap_resized)
        label.resize(self.win.width, self.win.height)

    def stylize_lbl(self, lbl, pos_x, pos_y, text="", width=-1, height=-1, center=True, title=False):
        lbl.setText(text)

        if title:
            lbl.setFont(self.title_lbl_text_font)
            bg_color = self.title_lbl_bg_color
            text_color = self.title_lbl_text_color
            if width == -1:
                width = self.title_lbl_width
            if height == -1:
                height = self.title_lbl_height

        else:
            lbl.setFont(self.lbl_text_font)
            bg_color = self.lbl_bg_color
            text_color = self.lbl_text_color
            if width == -1:
                width = self.lbl_width
            if height == -1:
                height = self.lbl_height

        if center:
            pos_x = int((self.win.width - width) / 2)

        lbl.setGeometry(pos_x, pos_y, width, height)
        lbl.setStyleSheet(f"background-color: rgba{bg_color};"
                          f"opacity: 255;"  # 0-255
                          f"color: rgba{text_color};"
                          "text-align: center;"
                          "opacity: 255;"  # 0-255
                          "padding: 3px;"
                          "margin: 2px;")

        lbl.setAlignment(Qt.AlignCenter)

    def stylize_start_btn(self, btn, pos_x, pos_y, logic, text=""):
        btn.setText(text)
        btn.clicked.connect(logic)

        btn.setFont(self.btn_text_font)
        btn.setGeometry(pos_x, pos_y, self.start_btn_width, self.start_btn_height)
        btn.setStyleSheet(f"background-color: rgba{self.start_btn_bg_color}; "
                          f"color: rgba{self.btn_text_color};"
                          "border-radius: 10;"
                          "border: 1px solid black;"
                          "text-align: center;"
                          f"opacity: {self.btn_opacity};"  # 0-255
                          "padding: 3px;"
                          "margin: 2px;")

    def stylize_btn(self, btn, pos_x, pos_y, logic, text=""):
        # self.closeButton.setShortcut('Ctrl+D')  # shortcut key
        # self.closeButton.setToolTip("Close the widget")  # Tool tip

        btn.setText(text)
        btn.clicked.connect(logic)

        btn.setFont(self.btn_text_font)
        btn.setGeometry(pos_x, pos_y, self.btn_width, self.btn_height)
        btn.setStyleSheet(f"background-color: rgba{self.btn_bg_color}; "
                          f"color: rgba{self.btn_text_color};"
                          "border-radius: 10;"
                          "border: 1px solid black;"
                          "text-align: center;"
                          f"opacity: {self.btn_opacity};"  # 0-255
                          "padding: 3px;"
                          "margin: 2px;")


    def stylize_btn_instructions(self, btn, pos_x, pos_y, logic, text=""):
        # self.closeButton.setShortcut('Ctrl+D')  # shortcut key
        # self.closeButton.setToolTip("Close the widget")  # Tool tip

        btn.setText(text)
        btn.clicked.connect(logic)

        btn.setFont(self.btn_text_font)
        btn.setGeometry(pos_x, pos_y, self.btn_width, self.btn_height)
        btn.setStyleSheet(f"background-color: rgba{self.btn_bg_color}; "
                          f"color: rgba{self.btn_text_color};"
                          "border-radius: 10;"
                          "border: 1px solid black;"
                          "text-align: center;"
                          f"opacity: {self.btn_opacity};"  # 0-255
                          "padding: 3px;"
                          "margin: 2px;")

    def stylize_progress_bar(self, bar, pos_y, width=-1, height=-1):

        if width == -1:
            width = self.prog_bar_width
        if height == -1:
            height = self.prog_bar_height

        pos_x = int((self.win.width - width) / 2)
        bar.setGeometry(pos_x, pos_y, width, height)
        bar.setAlignment(Qt.AlignCenter)
        bar.setFont(self.prog_bar_font)
        bar.setStyleSheet(f"background-color: rgba{self.prog_bar_bg_color};"
                          f"border: 1px solid red;"
                          f"border-radius: 10;")


class MainScreen(Screen):
    def __init__(self, win, params, name):
        super().__init__(win, params, name)
        self.create_screen()
        self.hide()

    def create_screen(self):
        # Background
        bg_image_path = self.bg_dict["main"]
        bg_label = QLabel(self.win)
        self.create_background(bg_label, bg_image_path)
        self.labels.append(bg_label)

        # Labels


        label_welcome = QLabel(self.win)


        self.stylize_lbl(label_welcome, pos_x=10, pos_y=30, text="ברוכים הבאים!", center=True, title=True)
        self.labels.append(label_welcome)

        label_battery_1 = QLabel(self.win)
        label_battery_2 = QLabel(self.win)

        self.stylize_lbl(label_battery_1, pos_x=self.win.width - self.lbl_width - 5, pos_y=30, text="bat1", center=False, title=False)
        self.stylize_lbl(label_battery_2, pos_x=self.win.width - self.lbl_width - 5, pos_y=30 + self.lbl_height, text="bat2", center=False, title=False)

        self.labels.append(label_battery_1)
        self.labels.append(label_battery_2)

        # Buttons
        btn_x = int((self.win.width - self.btn_width) / 2)  # x coordinate of button centralized in screen
        start_btn_x = int((self.win.width - self.start_btn_width) / 2)  # x coordinate of start button centralized in screen
        btn_y = self.btn_initial_y
        dy = self.btn_height + self.btn_margin_y

        'Start normal game'
        button_start_normal = QPushButton(self.win)
        self.stylize_start_btn(button_start_normal, start_btn_x - (self.start_btn_dx + self.start_btn_width), btn_y,
                               self.win.btnHandler.btn_move_to_song_choice_screen, "מלך התופים")
        # button_start.setShortcut('Return')  # shortcut key
        button_start_normal.setToolTip("Start playing normal game")  # Tool tip
        self.buttons.append(button_start_normal)

        'Start Traffic Light game'
        button_start_inhib = QPushButton(self.win)
        self.stylize_start_btn(button_start_inhib, start_btn_x, btn_y,
                               self.win.btnHandler.btn_move_to_traffic_light_game_screen, "רמזור")
        button_start_inhib.setToolTip("Start playing traffic light game")  # Tool tip
        self.buttons.append(button_start_inhib)
        # button_start.setShortcut('Return')  # shortcut key

        'Start Simon Says game'
        button_start_SS = QPushButton(self.win)
        self.stylize_start_btn(button_start_SS, start_btn_x + (self.start_btn_dx + self.start_btn_width), btn_y,
                               self.win.btnHandler.btn_move_to_simon_says_game_screen, "המלך אמר")
        button_start_SS.setToolTip("Start playing Simon Says game")  # Tool tip
        # button_start.setShortcut('Return')  # shortcut key
        self.buttons.append(button_start_SS)

        button_customization = QPushButton(self.win)
        self.stylize_btn(button_customization, btn_x, btn_y + dy, self.win.btnHandler.btn_move_to_customization_screen, "התאמה אישית")
        self.buttons.append(button_customization)

        button_instructions = QPushButton(self.win)
        self.stylize_btn(button_instructions, btn_x, btn_y + 2*dy, self.win.btnHandler.btn_instructions, "הוראות")
        self.buttons.append(button_instructions)

        button_exit = QPushButton(self.win)
        self.stylize_btn(button_exit, btn_x, btn_y + 3*dy, self.win.btnHandler.btn_exit, "יציאה")
        button_exit.setShortcut('Escape')  # shortcut key
        button_exit.setToolTip("Exit the game application")  # Tool tip
        self.buttons.append(button_exit)

    def show(self):
        super().show()

        player_name = self.win.db.firstName + " " + self.win.db.lastName
        if player_name == " ":  # empty string
            welcome_string = "ברוכים הבאים!"
        else:

            if self.win.db.gender == 'Male':
                welcome_string = f"ברוך הבא, {player_name}!"
            else:
                welcome_string = f"ברוכה הבאה, {player_name}!"

        self.labels[1].setText(welcome_string)  # welcome label



class CustomizationScreen(Screen):
    def __init__(self, win, params, name):
        super().__init__(win, params, name)
        self.create_screen()
        self.hide()

    def create_screen(self):
        # Background
        bg_image_path = self.bg_dict["main"]

        bg_label = QLabel(self.win)
        self.create_background(bg_label, bg_image_path)
        self.labels.append(bg_label)

        # Labels

        # Buttons
        btn_x, btn_y = int((self.win.width - self.btn_width) / 2), self.btn_initial_y  # first button coordinates
        dy = self.btn_height + self.btn_margin_y

        button_change_user = QPushButton(self.win)
        self.stylize_btn(button_change_user, btn_x, btn_y + 0 * dy, self.win.btnHandler.btn_change_user, "החלף משתמש")
        self.buttons.append(button_change_user)

        button_change_difficulty = QPushButton(self.win)
        self.stylize_btn(button_change_difficulty, btn_x, btn_y + 1 * dy, self.win.btnHandler.btn_change_difficulty, "שנה רמת קושי")
        self.buttons.append(button_change_difficulty)

        button_return_to_main = QPushButton(self.win)
        self.stylize_btn(button_return_to_main, btn_x, btn_y + 2*dy, self.win.btnHandler.btn_return_to_main, "לתפריט הראשי")
        self.buttons.append(button_return_to_main)


class InstructionsScreen(Screen):
    def __init__(self, win, params, name):
        super().__init__(win, params, name)
        self.create_screen()
        self.hide()

    def create_screen(self):
        # Background
        bg_image_path = self.bg_dict["main"]
        bg_label = QLabel(self.win)
        self.create_background(bg_label, bg_image_path)
        self.labels.append(bg_label)

        # Labels
        label_instructions = QLabel(self.win)
        # text = "In this game, you have robots named AutoClickers.\n" \
        #        "\n" \
        #        "The AutoClickers will move around and light up in colors according to the \nbeat of the song.\n" \
        #        "\n" \
        #        "When the robots are lit up, they are clickable. Clicking them will award \nyou points. \n" \
        #        "\n" \
        #        "The goal of the game is to earn as many points as possible for all songs.\n" \
        #        "Good luck and have fun!"
        text = "לחצו על סרטון ההדרכה של התרגול אותו תרצו להכיר"
        lbl_y = self.lbl_height / 2
        self.stylize_lbl(label_instructions, pos_x=0, pos_y=lbl_y, text=text, center=True, title=True)

        label_instructions.setFont(QFont('MV Boli', 30, QFont.Bold))
        # label_instructions.setGeometry(30, 30, self.win.width - 30, self.win.height - 30)
        label_instructions.setAlignment(Qt.AlignLeft)
        self.labels.append(label_instructions)

        # Buttons
        btn_x, btn_y = int((self.win.width - self.btn_width) / 2), self.btn_initial_y  # first button coordinates
        dy = self.btn_height + self.btn_margin_y

        button_KB = QPushButton(self.win)
        self.stylize_btn_instructions(button_KB, btn_x, btn_y + 0 * dy, self.win.btnHandler.btn_play_KB_tutorial, "אלוף התופים")
        self.buttons.append(button_KB)

        button_TL = QPushButton(self.win)
        self.stylize_btn_instructions(button_TL, btn_x, btn_y + 1 * dy, self.win.btnHandler.btn_play_TL_tutorial, "תרגול רמזור")
        self.buttons.append(button_TL)

        button_SS = QPushButton(self.win)
        self.stylize_btn_instructions(button_SS, btn_x, btn_y + 2 * dy, self.win.btnHandler.btn_play_SS_tutorial, "המלך אמר")
        self.buttons.append(button_SS)

        button_return_to_main = QPushButton(self.win)
        self.stylize_btn(button_return_to_main, btn_x, btn_y + 3*dy, self.win.btnHandler.btn_return_to_main, "לתפריט הראשי")
        self.buttons.append(button_return_to_main)

        videoWidget = QVideoWidget()


class SongChoiceScreen(Screen):
    # song choice label style
    song_choice_lbl_width = 310
    song_choice_lbl_text_font = QFont('Times', 20, QFont.Bold)
    song_choice_lbl_text_color = "(180, 180, 255, 1.0)"
    song_choice_lbl_margin_y = 100

    # Button style
    song_choice_btn_width = 1700
    def __init__(self, win, params, name):
        super(SongChoiceScreen, self).__init__(win, params, name)
        self.create_screen()
        self.hide()

    def create_screen(self):
        # Background
        bg_image_path = self.bg_dict["bongos"]


        bg_label = QLabel(self.win)
        self.create_background(bg_label, bg_image_path)
        self.labels.append(bg_label)

        # Buttons
        btn_x, btn_y = int((self.win.width - self.song_choice_btn_width) / 2), self.btn_initial_y  # first button coordinates
        dy = self.btn_height + self.btn_margin_y

        'Songs'
        button_song1 = QPushButton(self.win)
        self.stylize_song_choice_btn(button_song1, btn_x, btn_y - dy,
                                     self.win.btnHandler.btn_play_kol_hakavod, "כל הכבוד")
        # button_song1.setToolTip("Start playing current song")  # Tool tip
        self.buttons.append(button_song1)

        button_song2 = QPushButton(self.win)
        self.stylize_song_choice_btn(button_song2, btn_x, btn_y,
                                     self.win.btnHandler.btn_play_pgisha_bamiluim, "פגישה במילואים")

        self.buttons.append(button_song2)
        button_song2.setShortcut('Return')  # shortcut key

        button_song3 = QPushButton(self.win)
        self.stylize_song_choice_btn(button_song3, btn_x, btn_y + dy,
                                     self.win.btnHandler.btn_play_karnaval_banachal, "קרנבל בנחל")
        button_song3.setEnabled(False)
        self.buttons.append(button_song3)

        'Return to main'
        button_return_to_main = QPushButton(self.win)
        self.stylize_song_choice_btn(button_return_to_main, btn_x, btn_y + 2*dy,
                         self.win.btnHandler.btn_return_to_main, "לתפריט הראשי")
        button_return_to_main.setShortcut('Escape')  # shortcut key
        self.buttons.append(button_return_to_main)

    def stylize_song_choice_lbl(self, lbl, pos_x, pos_y, text=""):

        lbl.setText(text)
        lbl.setFont(self.song_choice_lbl_text_font)
        lbl.setGeometry(pos_x, pos_y, self.song_choice_lbl_width, self.lbl_height)
        lbl.setStyleSheet(f"background-color: {self.transparent_color}; "
                          f"color: rgba{self.song_choice_lbl_text_color};"
                          "border: 0px;"
                          "text-align: center;"
                          "opacity: 255;"  # 0-255
                          )
        lbl.setAlignment(Qt.AlignLeft)

    def stylize_song_choice_btn(self, btn, pos_x, pos_y, logic, text=""):
        self.stylize_btn(btn, pos_x, pos_y, logic, text)
        btn.setGeometry(pos_x, pos_y, self.song_choice_btn_width, self.btn_height)

        'Score label'
        label_score = QLabel(btn)
        lbl_pos_x = self.song_choice_btn_width - self.song_choice_lbl_width
        lbl_pos_y = self.song_choice_lbl_margin_y
        self.stylize_song_choice_lbl(label_score, lbl_pos_x, lbl_pos_y, "Top score: 0.0")
        self.scoreButtonLabels.append(label_score)

    def show(self):
        super().show()

        player_scores = self.win.db.get_kb_scores()
        i = 0
        # print(f"Player scores: {player_scores}")
        for score in player_scores[:-2]:  # disregard TL and SS scores
            score_label = self.scoreButtonLabels[i]
            text = f"Top Score: {score}"
            score_label.setText(text)
            i += 1


class BongoGameScreen(Screen):
    def __init__(self, win, params, name):
        super().__init__(win, params, name)
        self.create_screen()
        self.hide()

    def create_screen(self):
        # Background

        bg_image_path = self.bg_dict["bongos"]
        bg_label = QLabel(self.win)
        self.create_background(bg_label, bg_image_path)
        self.labels.append(bg_label)

        # Buttons
        button_pause = QPushButton(self.win)
        self.stylize_btn(button_pause, 0, 10, self.win.btnHandler.btn_pause_game, "P")
        self.buttons.append(button_pause)

        exit_button_height = 10
        button_exit_to_main = QPushButton(self.win)
        end_of_screen_pos = self.win.width - self.btn_width
        self.stylize_btn(button_exit_to_main, end_of_screen_pos, exit_button_height, self.win.btnHandler.btn_stop_game, "סיום משחק")
        button_exit_to_main.setShortcut('Escape')  # shortcut key
        self.buttons.append(button_exit_to_main)

        score_lbl_height = exit_button_height + self.btn_height
        label_score = QLabel(self.win)
        self.stylize_lbl(label_score, pos_x=10, pos_y=score_lbl_height, text="Score: ", center=True, title=True)
        self.labels.append(label_score)

        progress_bar = QProgressBar(self.win)
        prog_bar_height = score_lbl_height + self.title_lbl_height + self.prog_bar_margin_y
        self.stylize_progress_bar(progress_bar, pos_y=prog_bar_height)
        # self.stylize_progress_bar(progress_bar, int((self.win.width - self.prog_bar_width) / 2), prog_bar_height)

        self.progress_bar = progress_bar

    def show(self):
        super().show()

        self.progress_bar.show()

        curr_score = self.win.gameEngine.scoreHandler.score
        self.labels[1].setText(f"Score: {curr_score}")  # score label

    def hide(self):
        super().hide()

        self.progress_bar.hide()

        for clicker in self.autoclickers:
            clicker.animation.hide()


class TrafficLightGameScreen(Screen):

    def __init__(self, win, params, name):
        super().__init__(win, params, name)
        self.create_screen()
        self.hide()

    def create_screen(self):
        # Background
        bg_image_path = self.bg_dict["TL"]
        bg_label = QLabel(self.win)
        self.create_background(bg_label, bg_image_path)
        self.labels.append(bg_label)

        # Buttons
        button_pause = QPushButton(self.win)
        self.stylize_btn(button_pause, 0, 10, self.win.btnHandler.btn_pause_game, "P")
        self.buttons.append(button_pause)

        exit_button_height = 10
        button_exit_to_main = QPushButton(self.win)
        end_of_screen_pos = self.win.width - self.btn_width
        self.stylize_btn(button_exit_to_main, end_of_screen_pos, exit_button_height, self.win.btnHandler.btn_stop_game, "סיים משחק")
        button_exit_to_main.setShortcut('Escape')  # shortcut key
        self.buttons.append(button_exit_to_main)

        # score_lbl_height = exit_button_height + 2*self.btn_height
        # label_score = QLabel(self.win)
        # self.stylize_lbl(label_score, int((self.win.width - self.lbl_width) / 2), score_lbl_height, f"קומבו: ")
        # self.labels.append(label_score)

        # max_score_lbl_height = score_lbl_height + self.lbl_height
        # label_max_score = QLabel(self.win)
        # self.stylize_lbl(label_max_score, int((self.win.width - self.lbl_width) / 2), max_score_lbl_height, f"שיא נוכחי: {-1}")
        # self.labels.append(label_max_score)

        # dif_lbl_height = max_score_lbl_height + self.lbl_height
        # label_curr_difficulty = QLabel(self.win)
        # self.stylize_lbl(label_curr_difficulty, int((self.win.width - self.lbl_width) / 2), dif_lbl_height, f"רמת קושי: {-1}")
        # self.labels.append(label_curr_difficulty)


        progress_bar = QProgressBar(self.win)
        prog_bar_height = exit_button_height + self.title_lbl_height + self.prog_bar_margin_y
        self.stylize_progress_bar(progress_bar, pos_y=prog_bar_height)

        self.progress_bar = progress_bar

    def show(self):
        super().show()

        self.progress_bar.show()
        # if self.win.gameEngine:
            # self.labels[1].setText(f"קומבו: {self.win.gameEngine.scoreHandler.score}")  # score label
            # self.labels[2].setText(f"שיא נוכחי: {self.win.gameEngine.scoreHandler.max_combo}")
        # self.labels[3].setText(f"רמת קושי: {self.win.gameEngine.difficulty}")

    def hide(self):
        super().hide()  # hide all labels and buttons

        self.progress_bar.hide()

        for clicker in self.autoclickers:
            clicker.animation.hide()


class ScoreScreen(Screen):
    # score screen label style
    score_text_color = "(102, 204, 102, 1.0)"

    def __init__(self, win, params, name):
        super().__init__(win, params, name)
        self.create_screen()
        self.hide()

    def create_screen(self):
        # Background
        bg_image_path = self.bg_dict["score"]

        bg_label = QLabel(self.win)
        self.create_background(bg_label, bg_image_path)
        self.labels.append(bg_label)

        # Labels
        label_good_job = QLabel(self.win)
        self.stylize_lbl(label_good_job, pos_x=0, pos_y=130, text= "כל הכבוד!", center=True, title=True)
        label_good_job.setStyleSheet(f"color: rgba{self.score_text_color};")
        self.labels.append(label_good_job)

        # Buttons
        btn_x, btn_y = int((self.win.width - self.btn_width) / 2), self.btn_initial_y  # first button coordinates
        dy = self.btn_height + self.btn_margin_y

        button_return_to_main = QPushButton(self.win)
        self.stylize_btn(button_return_to_main, btn_x, btn_y + dy, self.win.btnHandler.btn_return_to_main, "לתפריט הראשי")
        button_return_to_main.setShortcut('Escape')  # shortcut key
        self.buttons.append(button_return_to_main)

        self.RT_graph = pyqtgraph.plot(title="זמן תגובה במהלך המשחק")
        self.RT_graph.setBackground((100, 50, 255, 25))

    def show(self):
        super().show()

        'Show Reaction Time graph'
        self.RT_graph.clear()
        items = []
        if type(self.win.gameEngine) == GameEngineBongos:
            items_per_robot = self.win.gameEngine.scoreHandler.reaction_times.values()
            items = [it for robo_item in items_per_robot for it in robo_item]
            # items = [(0, 0.1), (2, 0.3), (3, 0.5), (6, 0.4), (7, 0.2)]  # for testing
            # self.draw_KB_plot(items)

        elif type(self.win.gameEngine) == GameEngineTrafficLight:
            items_per_robot = self.win.gameEngine.scoreHandler.reaction_times.values()
            items = [it for robo_item in items_per_robot for it in robo_item]
            # items = [(1.2, 1), (2, 2), (3, -1), (4.5, -2)]  # for testing
            # self.draw_TL_plot(items)
        elif type(self.win.gameEngine) == GameEngineSimonSays:
            items_per_robot = self.win.gameEngine.scoreHandler.reaction_times.values()
            # self.draw_SS_plot(items)

    def draw_KB_plot(self, items):
        pen = pyqtgraph.mkPen(color=(115, 130, 200), width=8)
        self.RT_graph.setLabel('bottom', 'מספר לחיצה')
        inds = [el[0] for el in items]
        times = [el[1] for el in items]

        use_percentage = False
        'Convert to percent, out of 100'
        if len(times) > 1 and use_percentage:  # not blank or single time
            min_t = np.min(times)
            max_t = np.max(times)
            times_perc = [100*(max_t - time)/(max_t - min_t) for time in times]
            print(f"Reaction times [%]: {times_perc}")
            self.RT_graph.setLabel('left', 'טיב זמן התגובה [באחוזים]')
            self.RT_graph.plot(inds, times_perc, pen=pen)
        else:
            self.RT_graph.setLabel('left', 'זמן תגובה [שניות]')
            self.RT_graph.plot(inds, times, pen=pen)

        self.RT_graph.show()

    def draw_TL_plot(self, items):
        pass

    def draw_SS_plot(self, items):
        pass

    def hide(self):
        super().hide()
        self.RT_graph.hide()