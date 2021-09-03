import os
import threading
import time
from moviepy.editor import *
import pygame as pg
import numpy as np
import math


class Button:
    COLOR = (213, 213, 213)
    WIDTH = 550
    HEIGHT = 70
    MARGIN_Y = 25
    FONT = pg.font.SysFont("davidclm", 35)
    # possible fonts that support hebrew characters: davidclm, alefregular, alef, tahoma
    TEXT_COLOR = (151, 83, 0)

    def __init__(self, screen, pos_x, pos_y, logic, text="", color=COLOR):
        self.screen = screen
        self.color = color
        self.pos = [pos_x, pos_y, Button.WIDTH, Button.HEIGHT]  # x, y, width, height
        self.text = Button.FONT.render("{0}".format(text), True, Button.TEXT_COLOR)
        self.logic = logic

    def draw(self):
        pg.draw.rect(self.screen, Button.COLOR, self.pos)
        t_h = self.text.get_rect().height
        t_w = self.text.get_rect().width
        self.screen.blit(self.text,
                         (self.pos[0] + 0.5 * Button.WIDTH - 0.5 * t_w, self.pos[1] + 0.5 * Button.HEIGHT - 0.5 * t_h))

    def logic(self):
        self.logic()


class Label:
    COLOR = (213, 213, 213)
    WIDTH = 550
    HEIGHT = 70
    MARGIN_Y = 25
    FONT = pg.font.SysFont("davidclm", 35)
    # possible fonts that support hebrew characters: davidclm, alefregular, alef, tahoma
    TEXT_COLOR = (151, 83, 0)

    def __init__(self, screen, pos_x, pos_y, logic, text="", color=COLOR):
        self.screen = screen
        self.color = color
        self.pos = [pos_x, pos_y, Button.WIDTH, Button.HEIGHT]  # x, y, width, height
        self.text = Button.FONT.render("{0}".format(text), True, Button.TEXT_COLOR)

    def draw(self):
        pg.draw.rect(self.screen, Button.COLOR, self.pos)
        t_h = self.text.get_rect().height
        t_w = self.text.get_rect().width
        self.screen.blit(self.text,
                         (self.pos[0] + 0.5 * Label.WIDTH - 0.5 * t_w, self.pos[1] + 0.5 * Label.HEIGHT - 0.5 * t_h))

class Game:

    # region  Button functions
    def btn_change_scrn_logic(self):
        self.load_second_window()

    def btn_back_to_main_logic(self):
        self.load_main_window()

    def btn_start_game_logic(self, song_name):
        # Requested to begin game - play song
        print("Beginning game with song {0}!\n Please wait while the video renders...".format(song_name))
        clip = self.load_clip(song_name, USE_SAVE)
        self.load_clip_window()
        self.play_clip(clip)


    def btn_exit_logic(self):
        self.run = False

    def btn_pause_logic(self):
        self.paused = True

    # endregion

    def __init__(self, width, height, fullscreen):
        self.run = False
        self.active_screen = "None"
        self.width = width
        self.height = height
        self.fullscreen = fullscreen
        self.buttons = []
        self.robot_icons = []
        self.score = 0
        self.back_to_main = False # return to main menu after playing clip
        self.paused = False # pause game

        size, flags = self.initialize_game_window()
        self.screen = pg.display.set_mode(size, flags)

        'Create game loop'
        self.load_main_screen()

    def initialize_game_window(self):
        """
        Initialize the game window and calculate the window size
        """
        pg.init()
        pg.display.set_caption("Clicker Hero")

        if self.fullscreen:
            flags = pg.FULLSCREEN
        else:
            flags = 0

        size = (self.width, self.height)
        return size, flags

    def load_main_screen(self):
        """
        loads the main screen and initializes the game loop
        """

        self.load_main_window()

        self.run = True
        while self.run:
            self.handle_events()
            self.redraw_window()
            # TODO add self.iterate_clip in case clip is playing, else pass. This will require playing audio in main thread and not separate one, updating the pause button functionanlity
            # TODO add preprocessing for video - fourier to get the BPM

        print("Exiting game")
        pg.quit()

    def load_main_window(self):
        self.clear_screen()
        self.screen.blit(BG_main_screen, (0, 0))

        buttons = []
        screen_width, screen_height = self.screen.get_size()
        initial_x, initial_y = (screen_width - Button.WIDTH) / 2, 60  # first button coordinates

        start_game_btn = Button(self.screen, initial_x, initial_y, self.btn_change_scrn_logic, self.hebrew("התחל משחק"))
        buttons.append(start_game_btn)

        exit_btn = Button(self.screen, initial_x, initial_y + 1 * (Button.HEIGHT + Button.MARGIN_Y),
                          self.btn_exit_logic, self.hebrew("יציאה"))
        buttons.append(exit_btn)

        self.active_screen = "Main menu screen"
        self.buttons = buttons

    def load_end_of_game_window(self):
        self.clear_screen()
        self.screen.blit(BG_main_screen, (0, 0))
        buttons = []
        screen_width, screen_height = self.screen.get_size()
        initial_x, initial_y = (screen_width - Button.WIDTH) / 2, 60  # first button coordinates

        back_to_main_btn = Button(self.screen, initial_x, initial_y + 1 * (Button.HEIGHT + Button.MARGIN_Y),
                                  self.btn_back_to_main_logic, self.hebrew("חזרה לתפריט הראשי"))
        buttons.append(back_to_main_btn)

        score_label = main_font.render(self.hebrew(f"תוצאה סופית: {self.score}"), True, (100, 180, 255))
        congrats_label = main_font.render(self.hebrew("כל הכבוד!!"), True, (100, 180, 255))

        self.screen.blit(score_label, (470, 30))
        self.screen.blit(congrats_label, (520, 60))
        self.active_screen = "End-of-game screen"
        self.buttons = buttons

    def clear_screen(self):
        self.screen.fill((245, 245, 220))
        pg.display.flip()

    def handle_events(self):
        """
        Identifies button clicks and exits the game loop on command
        :return:
        """
        for event in pg.event.get():
            if event.type == pg.QUIT or (event.type == pg.KEYDOWN and event.key == pg.K_ESCAPE):
                self.run = False

            # Button clicks
            elif event.type == pg.MOUSEBUTTONDOWN:
                mouse_x, mouse_y = pg.mouse.get_pos()
                for btn in self.buttons:
                    btn_x, btn_y, btn_width, btn_height = btn.pos
                    'Check if user pressed on a button'
                    if btn_x <= mouse_x <= btn_x + btn_width and btn_y <= mouse_y <= btn_y + btn_height:
                        btn.logic()

    def redraw_window(self):
        for btn in self.buttons:
            btn.draw()

        pg.display.update()

    def load_second_window(self):
        self.clear_screen()
        self.screen.blit(BG_secondary_screen, (0, 0))

        buttons = []

        screen_width, screen_height = self.screen.get_size()
        initial_x, initial_y = (screen_width - Button.WIDTH) / 2, 60  # first button coordinates

        song_name = "Take My Breath"
        start_game_btn = Button(self.screen, initial_x, initial_y,
                                lambda: self.btn_start_game_logic(song_name), song_name)
        buttons.append(start_game_btn)

        back_to_main_btn = Button(self.screen, initial_x, initial_y + 1 * (Button.HEIGHT + Button.MARGIN_Y),
                                  self.btn_back_to_main_logic, self.hebrew("חזרה לתפריט הראשי"))
        buttons.append(back_to_main_btn)

        self.active_screen = "Choose song screen"
        self.buttons = buttons

    # region clip

    def load_clip(self, song_name, use_save=False, resize=False):
        song_dir_path = os.path.join(SONGS_PATH, song_name)
        song_path = os.path.join(song_dir_path, "{0}.mp4".format(song_name))
        song_cache_path = os.path.join(song_dir_path, "{0}_cached.mp4".format(song_name))

        # if using manipulations, using the save makes run time go from 6.415[s] to 5.2[s] on avg for 20 runs
        if not use_save:
            # clip = VideoFileClip(video_path, target_resolution=(1000, 500))
            clip = VideoFileClip(song_path)
            subclip = clip.subclip(60)
            if resize:
                clip_resized = subclip.resize(height=260)
            else:
                clip_resized = subclip
            # clip_resized.write_videofile(song_cache_path)
        else:
            clip_resized = VideoFileClip(song_cache_path)

        return clip_resized

    def load_clip_window(self):

        def redraw_window(screen):
            screen.blit(BG_clip_screen, (0, 0))
            pg.draw.rect(screen, (255, 0, 0), pg.Rect(5, 15, 250, 70))
            score_label = main_font.render("Score: 0", True, (255, 255, 255))

            screen.blit(score_label, (10, 30))

            for btn in self.buttons:
                btn.draw()

            pg.display.flip()


        buttons = []
        # pause_game_btn = Button(self.screen, 10, 10, self.btn_pause_logic, "Pause")
        # buttons.append(pause_game_btn)

        self.active_screen = "Clip screen"
        self.buttons = buttons

        redraw_window(self.screen)

    def play_clip(self, clip, fps=15, audio=True, audio_fps=22050, audio_buffersize=3000, audio_nbytes=2):
        """
        Displays the clip in a window, at the given frames per second (of movie)
        rate. It will avoid that the clip be played faster than normal, but it
        cannot avoid the clip to be played slower than normal if the computations
        are complex. In this case, try reducing the ``fps``.
        Parameters
        ----------
        fps : int, optional
          Number of frames per seconds in the displayed video.
        audio : bool, optional
          ``True`` (default) if you want the clip's audio be played during
          the preview.
        audio_fps : int, optional
          The frames per second to use when generating the audio sound.
        audio_buffersize : int, optional
          The sized of the buffer used generating the audio sound.
        audio_nbytes : int, optional
          The number of bytes used generating the audio sound.
        fullscreen : bool, optional
          ``True`` if you want the preview to be displayed fullscreen.
        Examples
        --------
        """

        def imdisplay(imarray, screen=None, clip_pos=(0, 0)):
            """Splashes the given image array on the given pygame screen."""
            if -1 in clip_pos:
                clip_pos = (0, 0)
            a = pg.surfarray.make_surface(imarray.swapaxes(0, 1))
            if screen is None:
                screen = pg.display.set_mode(imarray.shape[:2][::-1])

            screen.blit(a, clip_pos)
            pg.display.flip()

        def handle_events(self):
            """
            Identifies button clicks and exits the game loop on command
            :return:
            """
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    self.run = False
                elif event.type == pg.KEYDOWN and event.key == pg.K_ESCAPE:
                    self.back_to_main = True

                # Button clicks
                elif event.type == pg.MOUSEBUTTONDOWN:
                    mouse_x, mouse_y = pg.mouse.get_pos()
                    for btn in self.buttons:
                        btn_x, btn_y, btn_width, btn_height = btn.pos
                        'Check if user pressed on a button'
                        if btn_x <= mouse_x <= btn_x + btn_width and btn_y <= mouse_y <= btn_y + btn_height:
                            btn.logic()

        def redraw_window(screen, t, score):
            t_tolerance = 1e-08
            BPM = 120
            period = BPM / 60 / 4
            # FPS is about 44100, t is between 0.06666 till 1645333 jumps at 0.06666, rate is about 0.0000226
            # This is a sequence that climbs up to 'period', with rate 'rate', and checks if we're 't_tolerance' close to its multiple
            # print(t, period, t % period)



            if math.isclose(t % period, 0, abs_tol=t_tolerance) or \
                    math.isclose(period - (t % period), 0, abs_tol=t_tolerance):
                print("equal rate!")
                score += 100
            # TODO transfer from song to main menu, add icon of turtlebot lighting up and add pause button
            pg.draw.rect(screen, (255, 0, 0), pg.Rect(5, 15, 250, 70))
            pg.display.flip()
            score_label = main_font.render(f"Score: {score}", True, (255, 255, 255))
            screen.blit(score_label, (10, 30))

            pg.display.update()
            return score

        audio = audio and (clip.audio is not None)

        if audio:
            # the sound will be played in parallel. We are not
            # paralelizing it on different CPUs because it seems that
            # pygame and openCV already use several cpus.

            # two synchro-flags to tell whether audio and video are ready
            video_flag = threading.Event()
            audio_flag = threading.Event()
            # launch the thread
            audiothread = threading.Thread(
                target=clip.audio.preview,
                args=(audio_fps, audio_buffersize, audio_nbytes, audio_flag, video_flag),
            )  # TODO add a pause option for this thread
            audiothread.start()

        img = clip.get_frame(0)
        clip_width, clip_height = clip.size
        imdisplay(img, self.screen, ((self.width - clip_width) / 2, 0))

        if audio:  # synchronize with audio
            video_flag.set()  # say to the audio: video is ready
            audio_flag.wait()  # wait for the audio to be ready

        t0 = time.time()
        t_vector = np.arange(1.0 / fps, clip.duration - 0.001, 1.0 / fps)
        for t in t_vector:
            if self.paused:
                # video_flag.clear()
                time.sleep(2)
                # TODO add option for starting audio in sync at given time, and have the audio thread constantly update the time of self
                # video_flag.set()  # say to the audio: video is ready
                # audio_flag.wait()  # wait for the audio to be ready
                self.paused = False

            img = clip.get_frame(t)

            t1 = time.time()
            time.sleep(max(0, t - (t1 - t0)))
            imdisplay(img, self.screen, ((self.width - clip_width) / 2, 0))
            handle_events(self)
            self.score = redraw_window(self.screen, t, self.score)
            if self.back_to_main:
                self.back_to_main = False
                video_flag.clear() # kill the audio thread
                break

        print(f"Total score is: {self.score}")
        print(f"Returning to Main Menu.")

        self.load_end_of_game_window()
    # endregion

    # region Miscellaneous
    @staticmethod
    def hebrew(text):
        """
        Flips hebrew text direction
        """
        return text[::-1]
    # endregion


if __name__ == '__main__':
    # region Initialization
    USE_SAVE = False
    # USE_SAVE = True
    FULLSCREEN = False
    # FULLSCREEN = True

    if FULLSCREEN:
        WIDTH, HEIGHT = pg.display.Info().current_w, pg.display.Info().current_h # 1536, 864
    else:
        WIDTH, HEIGHT = 1200, 650
    # load assets and songs
    ROOT_PATH = os.path.dirname(os.path.abspath(__file__))
    SONGS_PATH = os.path.join(ROOT_PATH, "Songs")
    ASSET_PATH = os.path.join(ROOT_PATH, "Assets")

    green_btn = pg.image.load(os.path.join(ASSET_PATH, "green button.png"))
    BG_main_screen = pg.transform.scale(pg.image.load(os.path.join(ASSET_PATH, "BG_main.jfif")), (WIDTH, HEIGHT))
    BG_secondary_screen = pg.transform.scale(pg.image.load(os.path.join(ASSET_PATH, "BG_sec.jfif")), (WIDTH, HEIGHT))
    # BG_clip_screen = pg.transform.scale(pg.image.load(os.path.join(ASSET_PATH, "BG_clip.jfif")), (WIDTH, HEIGHT))
    BG_clip_screen = pg.transform.scale(pg.image.load(os.path.join(ASSET_PATH, "BG_clip2.jfif")), (WIDTH, HEIGHT))

    # HMI constants
    main_font = pg.font.SysFont("davidclm", 50)

    # endregion

    game = Game(WIDTH, HEIGHT, fullscreen=FULLSCREEN)
