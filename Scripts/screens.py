import os
from PyQt5.QtWidgets import (
    QLabel,
    QPushButton,
    QProgressBar
)
from PyQt5.QtGui import QPixmap, QFont
from PyQt5.QtCore import Qt
ASSET_PATH = r'Assets/'


class Screen:
    transparent_color = Qt.transparent

    # Label style
    lbl_width = 1500
    lbl_height = 140
    lbl_margin_y = 35
    lbl_text_color = "(255, 255, 255, 1.0)"
    lbl_bg_color = "(0, 0, 0, 0.2)"
    lbl_text_font = QFont('MV Boli', 70, QFont.Bold) # pristina, ravie

    # Button style
    btn_color = "(100, 100, 255, 0.8)"
    btn_initial_y = 250
    btn_width = 730
    btn_height = 140
    btn_margin_x = 50
    btn_margin_y = 50
    btn_text_color = "(240, 240, 240, 1.0)"
    btn_bg_color = "(100, 100, 255, 0.9)"
    btn_text_font = QFont('Times', 45, QFont.Bold)
    btn_opacity = 30

    #start button style
    start_btn_bg_color = "(150, 180, 255, 0.9)"
    start_btn_width = 580
    start_btn_dx = 290

    def __init__(self, win, name):
        self.win = win
        self.name = name
        self.labels = []
        self.scoreButtonLabels = []
        self.buttons = []
        self.autoclickers = []
        self.autoClickerAnims = []
        self.progress_bar = None

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

    def stylize_lbl(self, lbl, pos_x, pos_y, text=""):
        lbl.setText(text)
        lbl.setFont(self.lbl_text_font)
        lbl.setGeometry(pos_x, pos_y, self.lbl_width, self.lbl_height)
        lbl.setStyleSheet(f"background-color: rgba{self.lbl_bg_color};"
                          f"opacity: 255;"  # 0-255
                          f"color: rgba{self.lbl_text_color};"
                          "text-align: center;"
                          "opacity: 255;"  # 0-255
                          "padding: 3px;"
                          "margin: 2px;")
        lbl.setAlignment(Qt.AlignCenter)

    def stylize_start_btn(self, btn, pos_x, pos_y, logic, text=""):
        btn.setText(text)
        btn.clicked.connect(logic)

        btn.setFont(self.btn_text_font)
        btn.setGeometry(pos_x, pos_y, self.start_btn_width, self.btn_height)
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


class MainScreen(Screen):
    def __init__(self, win, name):
        super().__init__(win, name)
        self.create_screen()
        self.hide()

    def create_screen(self):
        # Background
        bg_image_path = ASSET_PATH + "mic_bg.jpg"
        bg_label = QLabel(self.win)
        self.create_background(bg_label, bg_image_path)
        self.labels.append(bg_label)

        # Labels
        label_welcome = QLabel(self.win)
        self.stylize_lbl(label_welcome, int((self.win.width - self.lbl_width)/2), 30, "Welcome!")
        self.labels.append(label_welcome)

        # Buttons
        btn_x, btn_y = int((self.win.width - self.btn_width) / 2), self.btn_initial_y  # first button coordinates
        dy = self.btn_height + self.btn_margin_y
        button_start_normal = QPushButton(self.win)

        'Start normal game'
        st_btn_x = (int(self.win.width - self.start_btn_width)/2)
        self.stylize_start_btn(button_start_normal, st_btn_x - self.start_btn_dx, btn_y, self.win.btnHandler.btn_move_to_song_choice_screen,
                               "התחל משחק קצב")
        # button_start.setShortcut('Return')  # shortcut key
        button_start_normal.setToolTip("Start playing normal game")  # Tool tip
        self.buttons.append(button_start_normal)

        'Start inhibition game'
        button_start_inhib = QPushButton(self.win)
        self.stylize_start_btn(button_start_inhib, st_btn_x + self.start_btn_dx, btn_y, self.win.btnHandler.btn_move_to_combo_game_screen,
                               "התחל משחק קומבו")
        # button_start.setShortcut('Return')  # shortcut key
        button_start_inhib.setToolTip("Start playing inhibition game")  # Tool tip
        self.buttons.append(button_start_inhib)

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
            welcome_string = "Welcome!"
        else:
            welcome_string = f"Welcome, {player_name}!"

        self.labels[1].setText(welcome_string)  # welcome label


class CustomizationScreen(Screen):
    def __init__(self, win, name):
        super().__init__(win, name)
        self.create_screen()
        self.hide()

    def create_screen(self):
        # Background
        bg_image_path = ASSET_PATH + "mic_bg.jpg"
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
        self.stylize_btn(button_return_to_main, btn_x, btn_y + 2*dy, self.win.btnHandler.btn_return_to_main, "חזרה לתפריט הראשי")
        self.buttons.append(button_return_to_main)


class InstructionsScreen(Screen):
    def __init__(self, win, name):
        super().__init__(win, name)
        self.create_screen()
        self.hide()

    def create_screen(self):
        # Background
        bg_image_path = ASSET_PATH + "mic_bg.jpg"
        bg_label = QLabel(self.win)
        self.create_background(bg_label, bg_image_path)
        self.labels.append(bg_label)

        # Labels
        label_instructions = QLabel(self.win)
        text = "In this game, you have robots named AutoClickers.\n" \
               "\n" \
               "The AutoClickers will move around and light up in colors according to the \nbeat of the song.\n" \
               "\n" \
               "When the robots are lit up, they are clickable. Clicking them will award \nyou points. \n" \
               "\n" \
               "The goal of the game is to earn as many points as possible for all songs.\n" \
               "Good luck and have fun!"
        self.stylize_lbl(label_instructions, int((self.win.width - self.lbl_width)/2), 30, text)
        label_instructions.setFont(QFont('MV Boli', 30, QFont.Bold))
        label_instructions.setGeometry(30, 30, self.win.width - 30, self.win.height - 30)
        label_instructions.setAlignment(Qt.AlignLeft)
        self.labels.append(label_instructions)

        # Buttons
        btn_x, btn_y = int((self.win.width - self.btn_width) / 2), self.btn_initial_y  # first button coordinates
        dy = self.btn_height + self.btn_margin_y

        button_return_to_main = QPushButton(self.win)
        self.stylize_btn(button_return_to_main, btn_x, btn_y + 3*dy, self.win.btnHandler.btn_return_to_main, "חזרה לתפריט הראשי")
        self.buttons.append(button_return_to_main)


class SongChoiceScreen(Screen):
    # song choice label style
    song_choice_lbl_width = 310
    song_choice_lbl_text_font = QFont('Times', 20, QFont.Bold)
    song_choice_lbl_text_color = "(180, 180, 255, 1.0)"
    song_choice_lbl_margin_y = 100

    def __init__(self, win, name):
        super(SongChoiceScreen, self).__init__(win, name)
        self.create_screen()
        self.hide()

    def create_screen(self):
        # Background
        bg_image_path = ASSET_PATH + "violin_bg.jpg"
        bg_label = QLabel(self.win)
        self.create_background(bg_label, bg_image_path)
        self.labels.append(bg_label)

        # Buttons
        btn_x, btn_y = int((self.win.width - self.btn_width) / 2), self.btn_initial_y  # first button coordinates
        dy = self.btn_height + self.btn_margin_y

        button_song1 = QPushButton(self.win)
        self.stylize_song_choice_btn(button_song1, btn_x, btn_y,
                                     self.win.btnHandler.btn_play_one_kiss, "One kiss")
        self.buttons.append(button_song1)

        button_song2 = QPushButton(self.win)
        self.stylize_song_choice_btn(button_song2, btn_x, btn_y + dy,
                                     self.win.btnHandler.btn_play_cant_help_falling_in_love, "Can't Help Falling In Love")
        button_song2.setShortcut('Return')  # shortcut key
        button_song2.setToolTip("Start playing current song")  # Tool tip
        self.buttons.append(button_song2)

        button_return_to_main = QPushButton(self.win)
        self.stylize_btn(button_return_to_main, btn_x, btn_y + 2*dy,
                         self.win.btnHandler.btn_return_to_main, "חזרה לתפריט הראשי")
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

        label_score = QLabel(btn)
        lbl_pos_x = self.btn_width - self.song_choice_lbl_width
        lbl_pos_y = self.song_choice_lbl_margin_y
        self.stylize_song_choice_lbl(label_score, lbl_pos_x, lbl_pos_y, "Top score: 0.0")
        self.scoreButtonLabels.append(label_score)



    def show(self):
        super().show()

        player_scores = self.win.db.scores
        i = 0
        print(player_scores)
        for score in player_scores[:-1]:
            score_label = self.scoreButtonLabels[i]
            text = f"Top Score: {score}"
            score_label.setText(text)
            i += 1


class NormalGameScreen(Screen):
    prog_bar_bg_color = "(150, 120, 180, 1.0)"
    prog_bar_width = 800
    prog_bar_height = 150
    prog_bar_margin_y = 5

    def __init__(self, win, name):
        super().__init__(win, name)
        self.create_screen()
        self.hide()

    def create_screen(self):
        # Background
        bg_image_path = ASSET_PATH + "violin_bg.jpg"
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
        self.stylize_btn(button_exit_to_main, end_of_screen_pos, exit_button_height, self.win.btnHandler.btn_stop_game, "חזרה לתפריט הראשי")
        button_exit_to_main.setShortcut('Escape')  # shortcut key
        self.buttons.append(button_exit_to_main)


        score_lbl_height = exit_button_height + self.btn_height + self.prog_bar_margin_y
        label_score = QLabel(self.win)
        self.stylize_lbl(label_score, int((self.win.width - self.lbl_width) / 2), score_lbl_height, f"Score: {self.win.score}")
        self.labels.append(label_score)

        progress_bar = QProgressBar(self.win)
        prog_bar_height = score_lbl_height + self.lbl_height + self.prog_bar_margin_y
        self.stylize_progress_bar(progress_bar, int((self.win.width - self.prog_bar_width) / 2), prog_bar_height)

        self.progress_bar = progress_bar

    def stylize_progress_bar(self, bar, pos_x, pos_y):
        bar.setGeometry(pos_x, pos_y, self.prog_bar_width, self.prog_bar_height)

        bar.setAlignment(Qt.AlignCenter)
        bar.setFont(self.btn_text_font)
        # bar.setFont(QFont('Arial', 20))
        bar.setStyleSheet(f"background-color: rgba{self.prog_bar_bg_color};"
                          f"border: 1px solid red;"
                          f"border-radius: 10;")

    def show(self):
        super().show()

        self.progress_bar.show()

        # for clicker in self.autoclickers:
        #     clicker.animation.show()

        curr_score = self.win.score
        self.labels[1].setText(f"Score: {curr_score}")  # score label

    def hide(self):
        super().hide()

        self.progress_bar.hide()

        for clicker in self.autoclickers:
            clicker.animation.hide()


class ComboGameScreen(Screen):

    def __init__(self, win, name):
        super().__init__(win, name)
        self.create_screen()
        self.hide()

    def create_screen(self):
        # Background
        bg_image_path = ASSET_PATH + "guitar_bg.jpg"
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
        self.stylize_btn(button_exit_to_main, end_of_screen_pos, exit_button_height, self.win.btnHandler.btn_stop_game, "חזרה לתפריט הראשי")
        button_exit_to_main.setShortcut('Escape')  # shortcut key
        self.buttons.append(button_exit_to_main)

        score_lbl_height = exit_button_height + 2*self.btn_height
        label_score = QLabel(self.win)
        self.stylize_lbl(label_score, int((self.win.width - self.lbl_width) / 2), score_lbl_height, f"קומבו:{self.win.score} ")
        self.labels.append(label_score)

        max_score_lbl_height = score_lbl_height + self.lbl_height
        label_max_score = QLabel(self.win)
        self.stylize_lbl(label_max_score, int((self.win.width - self.lbl_width) / 2), max_score_lbl_height, f"שיא נוכחי: {-1}")
        # label_max_score.setFont(QFont('MV Boli', 40, QFont.Bold))
        self.labels.append(label_max_score)

        dif_lbl_height = max_score_lbl_height + self.lbl_height
        label_curr_difficulty = QLabel(self.win)
        self.stylize_lbl(label_curr_difficulty, int((self.win.width - self.lbl_width) / 2), dif_lbl_height, f"רמת קושי: {-1}")
        self.labels.append(label_curr_difficulty)

    def show(self):
        super().show()
        self.labels[1].setText(f"קומבו: {self.win.score}")  # score label
        self.labels[2].setText(f"שיא נוכחי: {self.win.song_engine.max_combo}")
        self.labels[3].setText(f"רמת קושי: {self.win.screens['game'].autoclickers[0].pubThread.difficulty}")



    def hide(self):

        for clicker in self.autoclickers:
            clicker.animation.hide()

        super().hide()  # hide all labels and buttons

class ScoreScreen(Screen):
    # score screen label style
    score_text_color = "(102, 204, 102, 1.0)"

    def __init__(self, win, name):
        super().__init__(win, name)
        self.create_screen()
        self.hide()

    def create_screen(self):
        # Background
        bg_image_path = ASSET_PATH + "sky_bg.jpg"

        bg_label = QLabel(self.win)
        self.create_background(bg_label, bg_image_path)
        self.labels.append(bg_label)

        # Labels
        label_good_job = QLabel(self.win)
        self.stylize_lbl(label_good_job, int((self.win.width - self.lbl_width) / 2), 130, "כל הכבוד!")
        label_good_job.setStyleSheet(f"color: rgba{self.score_text_color};"
)
        self.labels.append(label_good_job)

        # Buttons
        btn_x, btn_y = int((self.win.width - self.btn_width) / 2), self.btn_initial_y  # first button coordinates
        dy = self.btn_height + self.btn_margin_y

        button_return_to_main = QPushButton(self.win)
        self.stylize_btn(button_return_to_main, btn_x, btn_y + dy, self.win.btnHandler.btn_return_to_main, "חזרה לתפריט הראשי")
        button_return_to_main.setShortcut('Escape')  # shortcut key
        self.buttons.append(button_return_to_main)