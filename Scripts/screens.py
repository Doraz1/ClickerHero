import os
from PyQt5.QtWidgets import (
    QLabel,
    QPushButton,
    QProgressBar
)
from PyQt5.QtGui import QPixmap, QFont
from PyQt5.QtCore import Qt
from webcolors import rgb_to_hex

ASSET_PATH = r'Assets/'

class Screen:
    # Label style
    lbl_color = rgb_to_hex((180, 180, 255))
    lbl_width = 1500
    lbl_height = 140
    lbl_margin_y = 35
    lbl_text_color = rgb_to_hex((232, 225, 225))
    lbl_bg_color = Qt.transparent
    lbl_text_font = QFont('MV Boli', 70, QFont.Bold) # pristina, ravie

    # song choice label style
    song_choice_lbl_width = 310
    song_choice_lbl_text_font = QFont('Times', 20, QFont.Bold)
    song_choice_lbl_text_color = rgb_to_hex((180, 180, 255))
    song_choice_lbl_margin_y = 80

    # score screen label style
    score_text_color = rgb_to_hex((102, 204, 102))

    # Button style
    btn_color = rgb_to_hex((240, 240, 240))
    btn_initial_y = 250
    btn_width = 700
    btn_height = 140
    btn_margin_x = 30
    btn_margin_y = 50
    btn_text_color = rgb_to_hex((240, 240, 240))
    btn_bg_color = rgb_to_hex((100, 100, 255))
    btn_text_font = QFont('Times', 25, QFont.Bold)
    btn_opacity = 30

    prog_bar_bg_color = rgb_to_hex((150, 120, 180))
    prog_bar_width = 800
    prog_bar_height = 150
    prog_bar_margin_y = 5

    def __init__(self, win, name):
        self.win = win
        self.name = name
        self.labels = []
        self.scoreButtonLabels = []
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
        lbl.setStyleSheet(f"background-color: {self.lbl_bg_color}; "
                          f"color: {self.lbl_text_color};"
                          "text-align: center;"
                          "opacity: 255;"  # 0-255
                          "padding: 3px;"
                          "margin: 2px;")
        lbl.setAlignment(Qt.AlignCenter)

    def stylize_song_choice_lbl(self, lbl, pos_x, pos_y, text=""):
        lbl.setText(text)
        lbl.setFont(self.song_choice_lbl_text_font)

        lbl.setGeometry(pos_x, pos_y, self.song_choice_lbl_width, self.lbl_height)
        lbl.setStyleSheet(f"background-color: {self.lbl_bg_color}; "
                          f"color: {self.song_choice_lbl_text_color};"
                          "border: 0px;"
                          "text-align: center;"
                          "opacity: 255;"  # 0-255
                          )
        lbl.setAlignment(Qt.AlignLeft)

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
                          "border: 1px solid black;"
                          "text-align: center;"
                          f"opacity: {self.btn_opacity};"  # 0-255
                          "padding: 3px;"
                          "margin: 2px;")

    def stylize_song_choice_btn(self, btn, pos_x, pos_y, logic, text=""):
        self.stylize_btn(btn, pos_x, pos_y, logic, text)

        label_score = QLabel(btn)
        lbl_pos_x = self.btn_width - self.song_choice_lbl_width
        lbl_pos_y = self.song_choice_lbl_margin_y
        self.stylize_song_choice_lbl(label_score, lbl_pos_x, lbl_pos_y, "Top score: 0.0")
        self.scoreButtonLabels.append(label_score)

    def stylize_progress_bar(self, bar, pos_x, pos_y):

        bar.setGeometry(pos_x, pos_y, self.prog_bar_width, self.prog_bar_height)

        bar.setAlignment(Qt.AlignCenter)
        bar.setFont(self.btn_text_font)
        # bar.setFont(QFont('Arial', 20))
        bar.setStyleSheet(f"background-color: {self.prog_bar_bg_color};"
                          f"border: 1px solid red;"
                          f"border-radius: 10;")


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
        label_welcome = QLabel(self.win)
        self.stylize_lbl(label_welcome, int((self.win.width - self.lbl_width)/2), 30, "Welcome!")
        self.labels.append(label_welcome)

        # Buttons
        btn_x, btn_y = int((self.win.width - self.btn_width) / 2), self.btn_initial_y  # first button coordinates
        dy = self.btn_height + self.btn_margin_y

        button_start = QPushButton(self.win)
        self.stylize_btn(button_start, btn_x, btn_y, self.win.btn_move_to_song_choice_screen, "התחל משחק")
        self.buttons.append(button_start)

        button_change_user = QPushButton(self.win)
        self.stylize_btn(button_change_user, btn_x, btn_y + dy, self.win.btn_change_user, "החלף משתמש")
        self.buttons.append(button_change_user)

        button_exit = QPushButton(self.win)
        self.stylize_btn(button_exit, btn_x, btn_y + 2*dy, self.win.btn_exit, "יציאה")
        self.buttons.append(button_exit)

    def show(self):
        super().show()

        player_name = self.win.playerDataBase.firstName + " " + self.win.playerDataBase.lastName
        if player_name == " ":  # empty string
            welcome_string = "Welcome!"
        else:
            welcome_string = f"Welcome, {player_name}!"

        self.labels[1].setText(welcome_string)  # welcome label


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

        # Buttons
        btn_x, btn_y = int((self.win.width - self.btn_width) / 2), self.btn_initial_y  # first button coordinates
        dy = self.btn_height + self.btn_margin_y

        button_start = QPushButton(self.win)
        self.stylize_song_choice_btn(button_start, btn_x, btn_y, self.win.btn_choose_song, "One kiss")
        self.buttons.append(button_start)

        button_return_to_main = QPushButton(self.win)
        self.stylize_btn(button_return_to_main, btn_x, btn_y + dy, self.win.btn_return_to_main, "חזרה לתפריט הראשי")
        self.buttons.append(button_return_to_main)

    def show(self):
        super().show()

        player_scores = self.win.playerDataBase.scores
        i = 0
        for score in player_scores[1:]:
            score_label = self.scoreButtonLabels[i]
            text = f"Top Score: {score}"
            score_label.setText(text)
            i += 1
            # add another label for the second song score
            break


class GameScreen(Screen):

    def __init__(self, win, name):
        super().__init__(win, name)
        self.create_screen()
        self.hide()

    def create_screen(self):
        # Background
        bg_image_path = os.path.join(ASSET_PATH, "bg_game.jpg")
        bg_label = QLabel(self.win)
        self.create_background(bg_label, bg_image_path)
        self.labels.append(bg_label)

        # Buttons
        button_pause = QPushButton(self.win)
        self.stylize_btn(button_pause, 0, 10, self.win.btn_pause_game, "P")
        self.buttons.append(button_pause)

        exit_button_height = 10
        button_exit_to_main = QPushButton(self.win)
        end_of_screen_pos = self.win.width - self.btn_width
        self.stylize_btn(button_exit_to_main, end_of_screen_pos, exit_button_height, self.win.btn_stop_game, "חזרה לתפריט הראשי")
        self.buttons.append(button_exit_to_main)


        score_lbl_height = exit_button_height + self.btn_height + self.prog_bar_margin_y
        label_score = QLabel(self.win)
        self.stylize_lbl(label_score, int((self.win.width - self.lbl_width) / 2), score_lbl_height, f"Score: {self.win.score}")
        self.labels.append(label_score)

        progress_bar = QProgressBar(self.win)
        prog_bar_height = score_lbl_height + self.lbl_height + self.prog_bar_margin_y
        self.stylize_progress_bar(progress_bar, int((self.win.width - self.prog_bar_width) / 2), prog_bar_height)
        self.progressBars.append(progress_bar)

    def show(self):
        super().show()

        curr_score = self.win.score
        self.labels[1].setText(f"Score: {curr_score}")  # score label


class ScoreScreen(Screen):

    def __init__(self, win, name):
        super().__init__(win, name)
        self.create_screen()
        self.hide()

    def create_screen(self):
        # Background
        bg_image_path = ASSET_PATH + "BG_score.jpg"

        bg_label = QLabel(self.win)
        self.create_background(bg_label, bg_image_path)
        self.labels.append(bg_label)

        # Labels
        label_good_job = QLabel(self.win)
        self.stylize_lbl(label_good_job, int((self.win.width - self.lbl_width) / 2), 130, "כל הכבוד!")
        label_good_job.setStyleSheet(f"color: {Screen.score_text_color};"
)
        self.labels.append(label_good_job)

        # Buttons
        btn_x, btn_y = int((self.win.width - self.btn_width) / 2), self.btn_initial_y  # first button coordinates
        dy = self.btn_height + self.btn_margin_y

        button_return_to_main = QPushButton(self.win)
        self.stylize_btn(button_return_to_main, btn_x, btn_y + dy, self.win.btn_return_to_main, "חזרה לתפריט הראשי")
        self.buttons.append(button_return_to_main)