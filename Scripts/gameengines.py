import copy
import math

import pygame.mixer
from mutagen.mp3 import MP3
import numpy as np
import os
from playsound import playsound
import threading
from abc import ABC, abstractmethod

from Scripts.enums import Screens, Songs, Games
from Scripts.threads import LEDNavPubThreadBongos, LEDNavPubThreadTrafficLight, LedNavPubThreadSimonSays
from Scripts.ros_threads import Ros2QTSubscriber, LivelihoodSubscriberNode
import time
import rclpy
import pickle

from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QMessageBox,
    QWidget,
)

from PyQt5.QtWidgets import QMessageBox

class SoundHandler:
    def __init__(self, gameEngine, params):
        def reset_signal(start_beat):
            move = [-1, start_beat, -1.0, -1]

            return move
        def stop(start_beat):
            move = [0, start_beat,0.0, -1]
            return move

        def line(start_beat, length, frac=-1):
            move = [1, start_beat, length, frac]
            return move

        def circle(start_beat, length, ccw, frac=1):
            if ccw:
                'Move backward clockwise - sign is positive'
                sign = 1
            else:
                sign = -1

            move = [2, start_beat, length, sign * frac]
            return move

        def go_home(start_beat):
            move = [3, start_beat, -1.0, -1.0]
            return move

        def freestyle(start_beat, length):
            move = [4, start_beat, length, -1.0]
            return move

        'PID commands'
        def CL_forward(start_beat, length, interruptable=False):
            move = [6, start_beat, length, -1]

            if interruptable:
                N = 8
            else:
                N = 9
            L = 0.2
            return move

        def CL_rotate_cw(start_beat, length, interruptable=False):
            move = [7, start_beat, length, -1]
            return move

        def CL_rotate_ccw(start_beat, length, interruptable=False):
            move = [8, start_beat, length, -1]
            return move


        self.gameEngine = gameEngine  # for debugging
        self.params = params  # for debugging
        self.song_name = ''
        self.path = ''
        self.phase_list = []
        self.note_inds = []
        self.bpm_list = []
        self.notes_per_phase_list = []
        self.music_playing = False
        self.initialized_t0 = False

        user_reach = self.params["user_reach"]  # number between 80 and 120 cms
        L_line = 30  # in cms
        # L_line = user_reach / 200  # PID accepts values in meters
        R_circle = user_reach / 1.5
        dance_radius_KH = np.deg2rad(8)
        dance_radius_PBM = np.deg2rad(25)

        chorus1_KH = [CL_rotate_cw(0, np.pi/2), stop(4), line(6, L_line/2)]
        chorus2_KH = [CL_rotate_ccw(0, np.pi/2), line(4, L_line/2)]

        chorus1_PBM = [CL_rotate_ccw(0, dance_radius_PBM/2), CL_rotate_cw(4, dance_radius_PBM)]
        chorus2_PBM = [CL_rotate_ccw(0, dance_radius_PBM), CL_rotate_cw(4, dance_radius_PBM/2)]

        self.sounds_dict = {
            # region Kingo of the Bongos

            "kol_hakavod": {"path": os.path.join(self.params["HEBREW_SONGS_PATH"], "Yehoram gaon kol hakavod.mp3"),
                            "phase_list": [
                                           ([0.4, 3.8], 70),   # robo dancing -                 0
                                           ([7.3, 14.1], 70),  # singer enters -                1
                                           ([21.0, 26.9, 33.3], 68),  # chorus -                2
                                           ([36.9, 43.8], 70),  #                               3
                                           ([50.6, 56.6, 62.8], 68),  # chorus -                4
                                           ([66.7, 73.6], 70),   #                              5
                                           ([80.4, 86.8, 93.0], 68),  # chorus -                6
                                           ([96.8, 100.1, 103.8], 70),   #                      7
                                           ([111.1, 117.3], 68),  # chorus                      8
                                           ([124.5, 130.8, 137.8], 68), # chorus                9
                                           ],
#                        (7, 0, [circle(0, R_circle, ccw=True), stop(4)]), (7, 2, [CL_rotate_ccw(0, np.pi / 3)]),
        "moves_list": [
                                           (0, 0, [CL_rotate_cw(0, dance_radius_KH/2), CL_rotate_ccw(1, dance_radius_KH), CL_rotate_cw(2, dance_radius_KH), CL_rotate_ccw(3, dance_radius_KH)]), (0, 1, [CL_rotate_cw(0, dance_radius_KH), CL_rotate_ccw(1, dance_radius_KH),CL_rotate_cw(2, dance_radius_KH), CL_rotate_ccw(3, dance_radius_KH / 2)]),
                                           (1, 0, [line(0, L_line)]),
                                           (2, 0, chorus1_KH), (2, 1, chorus2_KH),
                                           (3, 0, [line(0, L_line), stop(7), line(9, L_line)]),
                                           (4, 0, chorus1_KH), (4, 1, chorus2_KH),
                                           (5, 0, [CL_rotate_cw(0, np.pi / 4)]), (5, 1, [line(0, L_line / 4)]),  # change it up with circles here
                                           (6, 0, chorus1_KH), (6, 1, chorus2_KH),
                                           (7, 0, [CL_rotate_ccw(0, np.pi/4), stop(4)]), (7, 2, [CL_rotate_ccw(0, np.pi / 3)]),
                                           (8, 0, chorus1_KH), (8, 1, chorus2_KH),
                                           (9, 0, chorus1_KH), (9, 1, chorus2_KH)
                                           ]},

            "pgisha_bamiluim": {"path": os.path.join(self.params["HEBREW_SONGS_PATH"], "Hachalonot hagvohim pgisha bemiluim_cropped.mp3"),
                                "phase_list": [([0.3, 4.3], 120),  #initial drumming                                 0
                                               ([8.3, 12.3], 120),  #                                                1
                                               ([16.3, 20.3, 24.3, 28.3], 120),  # whistle chorus                    2
                                               ([32.3, 36.3, 40.3, 44.3, 48.3, 52.3], 120),  # banu lamiluim         3
                                               ([56.3, 60.3, 64.3, 68.3, 72.3, 76.3], 120),  # rav hasamal           4
                                               ([80.3, 84.3, 88.3, 92.3, 96.3, 100.3], 120), # amar shetsarih lahpor 5
                                               ([104.3, 108.3, 112.3, 116.3, 120.3], 120),  # az bau hayetushim      6
                                               ([128.3, 136.3, 140.3, 144.3, 148.3], 120),  # deep whistles chorus                 7
                                               ([152.2, 156.2, 160.2, 164.2, 168.2], 120),  # yatzanu mitoh habor    8
                                               ([176.2, 180.2], 120)],  # yipol letoh habor fadeout           9

                                "moves_list": [
                                           (0, 0, [line(0, L_line)]),
                                           (1, 0, [CL_rotate_cw(0, np.pi / 4), line(4, L_line)]), (1, 1, [CL_rotate_ccw(0, np.pi / 4), line(4, L_line)]),
                                           (2, 0, chorus1_PBM), (2, 1, chorus2_PBM), (2, 2, chorus1_PBM), (2, 3, chorus2_PBM),
                                           (3, 0, [line(0, L_line)]), (3, 3, [stop(5)]), (3, 4, [line(0, L_line)]),
                                           (4, 0, [line(0, L_line)]), (4, 3, [stop(5)]), (4, 4, [line(0, L_line)]),
                                           (5, 0, [line(0, L_line)]), (5, 3, [stop(5)]), (5, 4, [line(0, L_line)]),
                                           (6, 0, [line(0, L_line)]), (6, 3, [stop(5)]), (6, 4, [line(0, L_line)]),
                                           (7, 0, [line(0 ,L_line)]), (7, 1, chorus1_PBM), (7, 2, chorus2_PBM), (7, 3, chorus1_PBM), (7, 4, chorus2_PBM),
                                           (8, 0, [line(0, L_line)]), (8, 3, [stop(5)]), (8, 4, [line(0, L_line)]),
                                           (9, 0, [CL_rotate_cw(0, np.pi / 2)]), (9, 1, [CL_rotate_ccw(0, np.pi / 2)]), (9, 1, [CL_rotate_cw(0, np.pi / 2)]),
                                           ]},
            "karnaval_banachal": {"path": os.path.join(self.params["HEBREW_SONGS_PATH"], "Lehakat hanachal karnaval banachal.mp3"),
                                  "phase_list": [([0.2, 7.8, 11.3, 14.8], 138),
                                                 ([18.1, 22.2, 27.4, 32.3], 138),
                                                 ([35.3, 38.8, 44.0], 138),
                                                 ([45.5, 49.0, 52.2, 59.1, 62.6, 65.7, 69.1, 71.7], 138),
                                                 ([72.6, 83.0], 138),
                                                 ([84.8, 91.7, 95.2, 98.7, 105.6, 110.8], 138),
                                                 ([112.7, 114.5, 123.2], 138),
                                                 ([124.0], 110),
                                                 ([159.4], 138)],
                                  "moves_list": [(0, 0, [stop(0)]),  # phase, subphase, moves with
                                                 (1, 0, [line(0, 15.0)]),
                                                 (2, 0, [circle(0, 15.0, ccw=True)])],
                                  },
            "yehezkel": {"path": os.path.join(self.params["HEBREW_SONGS_PATH"], "Hachalonot hagvohim yehezkel.mp3"),
                         "phase_list": [],
                         "bpm_list": []},
            "kaha_stam": {"path": os.path.join(self.params["HEBREW_SONGS_PATH"], "kaha stam.mp3"),
                          "phase_list": [],
                          "bpm_list": []},
            "hallelujah": {"path": os.path.join(self.params["HEBREW_SONGS_PATH"], "Lehakat hanachal Hallelujah.mp3"),
                           "phase_list": [],
                           "bpm_list": []},
            "we_will_rock_you": {"path": os.path.join(self.params["SONGS_PATH"], "Queen - We Will Rock You.mp3"),
                                 "phase_list": [],
                                 "bpm_list": []},

            # "pirates_of_the_caribbean": {"path": os.path.join(self.params["SONGS_PATH"], "Pirates Of The Caribbean Theme Song.mp3"),
            "pirates_of_the_caribbean": {"path": os.path.join(self.params["SONGS_PATH"], "Pirates Of The Caribbean Theme Song_slowed.mp3"),
                                 "phase_list": [],
                                 "bpm_list": []},
            "bahaim_hakol_over": {"path": os.path.join(self.params["HEBREW_SONGS_PATH"], "asdf.mp3"),
                                  "phase_list": [],
                                  "bpm_list": []},

            # endregion
            # Traffic light
            "traffic_light_music": {
                "path": os.path.join(self.params["SONGS_PATH"], "March_songs.mp3"),
                "phase_list": [],
                "bpm_list": []},

            # Simon says
            "simon_says_music": {
                "path": os.path.join(self.params["HEBREW_SONGS_PATH"], "Hava albershtein hiuchim_quiet.mp3"),
                "phase_list": [],
                "bpm_list": []},
            # "simon_says_music": {"path": os.path.join(self.params["HEBREW_SONGS_PATH"], "Lehakat hanachal geshem bo.mp3")},
            # "simon_says_music": {"path": os.path.join(self.params["SONGS_PATH"], "Pirates Of The Caribbean Theme Song_slowed.mp3")},

            # region background sounds
            "welcome_yalla": {"path": os.path.join(self.params["ADDITIONAL_AUDIO_PATH"] + '/welcomes', "yalla_slow.mp3")},
            "welcome_lets_begin": {"path": os.path.join(self.params["ADDITIONAL_AUDIO_PATH"] + '/welcomes', "lets_begin_slow.mp3")},
            "welcome_male": {"path": os.path.join(self.params["ADDITIONAL_AUDIO_PATH"] + '/welcomes', "lets_start_practicing_male_slow.mp3")},
            "welcome_female": {"path": os.path.join(self.params["ADDITIONAL_AUDIO_PATH"] + '/welcomes', "lets_start_practicing_female_slow.mp3")},

            "woohoo": {"path": os.path.join(self.params["ADDITIONAL_AUDIO_PATH"], "Woohoo.mp3")},
            "crowd_cheering": {"path": os.path.join(self.params["ADDITIONAL_AUDIO_PATH"], "Crowd cheering.mp3")},
            "uh_oh": {"path": os.path.join(self.params["ADDITIONAL_AUDIO_PATH"], "Uh Oh.mp3")},
            "aww": {"path": os.path.join(self.params["ADDITIONAL_AUDIO_PATH"], "Aww.mp3")},

            "my_turn": {"path": os.path.join(self.params["ADDITIONAL_AUDIO_PATH"], "my_turn_fast_loud.mp3")},

            "your_turn_male": {"path": os.path.join(self.params["ADDITIONAL_AUDIO_PATH"], "your_turn_male_loud.mp3")},

            "your_turn_female": {"path": os.path.join(self.params["ADDITIONAL_AUDIO_PATH"], "your_turn_female_loud.mp3")},
            "game_ended": {"path": os.path.join(self.params["ADDITIONAL_AUDIO_PATH"], "you_win.mp3")},
            # "game_ended": {"path": os.path.join(self.params["ADDITIONAL_AUDIO_PATH"], "you_win2.mp3")},
            # endregion

        }
        self.play_welcome()

    def load_song(self, song_name: str, t0: float=0.0):
        self.song_name = song_name
        if not self.initialized_t0:
            self.start_from_sec = t0  # for debugging
            self.initialized_t0 = True


        self.path = self.sounds_dict[song_name]["path"]
        self.phase_list = self.sounds_dict[song_name]["phase_list"]
        # self.bpm_list = self.sounds_dict[song_name]["bpm_list"]
        # self.notes_per_phase_list = self.sounds_dict[song_name]["notes_per_phase"]

    def play(self, song_name):
        song_path = self.sounds_dict[song_name]["path"]
        pygame.mixer.init()

        while pygame.mixer.get_init() is None:
            print("initializing mixer")
            pygame.mixer.init()
            time.sleep(0.2)

        pygame.mixer.music.load(song_path)
        print(f"Playing {song_name} music")
        pygame.mixer.music.play(start=self.start_from_sec)
        self.music_playing = True

    def get_progress(self):
        prog = self.start_from_sec + (pygame.mixer.music.get_pos() / 1000)  # in seconds
        L = self.get_song_length()  # in seconds
        prog_percent = prog / L * 100
        return prog_percent

    def get_song_length(self):
        song_path = self.sounds_dict[self.song_name]["path"]
        audio = MP3(song_path)
        return audio.info.length

    def song_finished(self):
        prog = self.get_progress()

        if prog >= 99:
            return True
        else:
            return False

    def stop_music(self):
        pygame.mixer.music.fadeout(self.params["fadeout_music_time"])
        self.music_playing = False


    # play short sounds

    def play_welcome(self):
        path_welcome_yalla = self.sounds_dict["welcome_yalla"]["path"]
        path_welcome_lets_begin = self.sounds_dict["welcome_lets_begin"]["path"]
        possibilities = [path_welcome_yalla, path_welcome_lets_begin]

        if self.gameEngine.win.db.gender == 'Male':  # enum or const
            path_welcome_male = self.sounds_dict["welcome_male"]["path"]
            possibilities.append(path_welcome_male)
        else:
            path_welcome_female = self.sounds_dict["welcome_female"]["path"]
            possibilities.append(path_welcome_female)

        choice = np.random.choice([0, 1, 2], 1)[0]
        print(f"Playing choice {choice} out of possibilities: {possibilities}")
        self.__play_short_sound(possibilities[choice], join=True)

    def play_woohoo(self):
        path_woohoo = self.sounds_dict["woohoo"]["path"]
        path_cheering = self.sounds_dict["crowd_cheering"]["path"]
        possibilities = [path_woohoo, path_cheering]

        choice = np.random.choice([0, 1], 1)[0]
        self.__play_short_sound(possibilities[choice])

    def play_uh_oh(self):
        path_uh_oh = self.sounds_dict["uh_oh"]["path"]
        path_aww = self.sounds_dict["aww"]["path"]
        possibilities = [path_uh_oh, path_aww]
        choice = np.random.choice([0, 1], 1)[0]

        self.__play_short_sound(possibilities[choice])

    def play_my_turn(self):
        path = self.sounds_dict["my_turn"]["path"]
        self.__play_short_sound(path)

    def play_your_turn(self):
        path = self.sounds_dict["your_turn_female"]["path"]
        if self.gameEngine.win.db.gender == 'Male':  # enum or const
            path = self.sounds_dict["your_turn_male"]["path"]
        # elif self.gameEngine.win.db.gender == 'Female':
        #     path = self.sounds_dict["your_turn_female"]["path"]

        self.__play_short_sound(path)

    def play_game_ended(self):
        path = self.sounds_dict["game_ended"]["path"]
        self.__play_short_sound(path, join=True)


    def __play_short_sound(self, path, join=False):
        # starts the play sound loop. since we don't call thread.join() we don't wait for execute
        t = threading.Thread(target=playsound, args=(path,))
        t.start()
        if join:
            t.join(timeout=2)


class NoteHandler:
    def __init__(self, gameEngine, params, sound_handler):
        self.gameEngine = gameEngine
        self.params = params
        self.soundHandler = sound_handler

    def get_note_times(self, song_name):
        com_del = self.params["communication_delay"]/100
        self.soundHandler.load_song(song_name)
        phase_list = self.soundHandler.phase_list
        note_times = []
        for i, (phase_note_times, phase_bpm) in enumerate(phase_list):
            bps = phase_bpm / 60
            dt = 1 / bps  # every this many seconds

            phase_start = phase_note_times[0]
            if i + 1 == len(phase_list):
                'End of phase list'
                phase_end = self.soundHandler.get_song_length()
            else:
                'end of this phase is the start of the next one'
                next_phase_notes, next_phase_bpm = phase_list[i + 1]
                phase_end = next_phase_notes[0]

            'Append notes between current phase beats'
            for st_note, en_note in zip(phase_note_times[:-1], phase_note_times[1:]):
                phase_times = np.arange(st_note, en_note, dt)  # 1.9sec, 3.8sec, etc.
                note_times.append(phase_times.tolist())

            'Append notes from last phase beat to next phase start'
            end_phase_times = np.arange(en_note, phase_end, dt)
            note_times.append(end_phase_times[:-1])

        flat_note_list = [item-com_del for sublist in note_times for item in sublist]
        flat_note_list.append(phase_end)  # fin
        return flat_note_list


class RobotHandler:
    win: QMainWindow

    def __init__(self, win, params, num_robots):
        self.win = win
        self.params = params
        self.num_robots = num_robots
        self.robot_alive = num_robots*[False]
        self.time_attempted_to_establish_connection = num_robots*[time.time()]
        self.battery_states = num_robots*[100]
        self.notified_battery_low = num_robots*[False]

        self.livelihood_counter = np.array(self.num_robots*[0])
        self.livelihood_thresh = self.params["livelihood_thresh"]
        self.running = False
        if self.params["check_communication"]:
            self.com_error = True
        else:
            self.com_error = False

        for i in range(num_robots):
            sub = LivelihoodSubscriberNode(self.win, i)
            self.__setattr__(f'robo_{i}_alive_sub', sub)  # listen to livelihood msgs
            self.win.rosHandler.executor.add_node(sub)

        self.start()

    def start(self):
        if not self.running:
            self.running = True
            self.timer_thread = threading.Thread(target=self.livelihood_thread, daemon=True)
            self.timer_thread.start()

    def notify_robot_alive(self, ind):
        was_alive = self.robot_alive[ind]

        self.robot_alive[ind] = True
        self.livelihood_counter[ind] = 0

        if self.robot_alive[ind] and not was_alive:
            self.win.robotHandler.time_attempted_to_establish_connection[ind] = 0  # on connect
            print(f"Robot {ind} connected!")

    def update_battery_state(self, ind, val):
        if val <= self.battery_states[ind]:
            pass
            # print(f"Robot {ind} battery decreased to: {val}")
        self.battery_states[ind] = val

        if val <= self.params["battery_alarm_thresh"] and not self.notified_battery_low[ind]:
            print("Updating battery state")
            if self.win.robotHandler.com_error:
                msg = QMessageBox.about(self.win, "Battery low",
                                        f"Robot {ind} Battery low. Please charge robot to prevent game errors.")

            self.notified_battery_low[ind] = True

    def livelihood_thread(self):
        def connect_robot(i):
            self.win.robotHandler.time_attempted_to_establish_connection[i] = time.time()

            # launch_robot_cmd = "ssh -t ubuntu@${" + f'robot{i}_ip' + "} 'source ~/.bashrc; ros2 launch turtlebot3_bringup clickerHeroRobot.launch.py; bash'"  # keep open when done
            # launch_robot_cmd = "ssh -t ubuntu@${" + f'robot{i}_ip' + "} 'source ~/.bashrc; ros2 launch turtlebot3_bringup clickerHeroRobot.launch.py'"  # close when done
            # launch_robot_in_terminal_tab_cmd = f'gnome-terminal --wait --maximize --tab --title="robot {i}" -- {launch_robot_cmd}'
            launch_robot_in_terminal_tab_cmd = f'sh ~/Desktop/launch_robot{i}.sh'
            first_time_connecting[i] = False
            t = threading.Thread(target=os.system, args=(launch_robot_in_terminal_tab_cmd,))
            t.start()
        def release_lock_files(i):
            cmd = f'rm -rf /tmp/robo{i}_launch_script.lock'
            t = threading.Thread(target=os.system, args=(cmd,))
            t.start()

        dt = 1.0  # sec
        main_screen = self.win.screenHandler.screen_dict[Screens.MAIN.value]

        first_time_connecting = self.num_robots*[True]
        red_color = "(240, 0, 0, 1.0)"
        orange_color = "(255, 127, 80, 1.0)"
        green_color = "(0, 240, 0, 1.0)"
        title_lbl_bg_color = "(0, 0, 0, 0.2)"

        while self.running:
            self.livelihood_counter += 1

            'Check if robot stopped being alive'
            for ind, count in enumerate(self.livelihood_counter):
                was_alive = self.robot_alive[ind]
                if count > self.livelihood_thresh:
                    self.robot_alive[ind] = False

                if was_alive and not self.robot_alive[ind]:
                    print(f"Robot {ind} disconnected :(")
                    self.win.robotHandler.time_attempted_to_establish_connection[ind] = time.time()  # Beginnign of grace period
                    first_time_connecting[ind] = True

                    if self.params["check_communication"]:
                        self.com_error = True

            if self.params["check_communication"]:
                if False not in self.robot_alive:
                    self.com_error = False

                'Handle communication and battery levels'
                if self.win.screenHandler.active_screen == main_screen:
                    if hasattr(self.win, 'robotHandler'):
                        for i in range(self.win.robotHandler.num_robots):
                            if not self.win.robotHandler.robot_alive[i]:
                                'robot disconnected - establish connection if turned on'
                                response = self.ping(i) # 0 if pinging, else not connected to wifi
                                # print(f"Pinging robot {i}. response: {response}")
                                if response == 0:
                                    'ROBOT pinging! try to establish connection'
                                    # when not pinging this is 0, otherwise (initially and when launching) a connect this is time.time(),
                                    last_attempt_time = self.win.robotHandler.time_attempted_to_establish_connection[i]
                                    # print(f"Last robot {i} attempt time: {last_attempt_time}")
                                    if last_attempt_time != 0:
                                        'Robot pinging'
                                        # 'was pinging last time, meaning were at grace period or were connected. check if grace is over'
                                        reconnect_dt =  time.time() - last_attempt_time
                                        if first_time_connecting[i]:
                                            reconnect_dt_thresh = self.params["robot_initial_reconnect_dt"]
                                        else:
                                            reconnect_dt_thresh = self.params["robot_reconnect_dt"]

                                        if reconnect_dt <= reconnect_dt_thresh:
                                            'Still at grace period - dont reconnect'
                                            print(f'\n-reconnect dt {i}: {reconnect_dt} < {reconnect_dt_thresh}\n')
                                            continue
                                        else:
                                            'grace period over - connect'
                                            connect_robot(i)
                                    else:
                                        'robot wasnt pinging last time and just connected - initial grace period'
                                        first_time_connecting[i] = True
                                        self.win.robotHandler.time_attempted_to_establish_connection[i] = time.time()
                                        release_lock_files(i)

                                    # 'Either robot just pinged first time, or grace period is over'
                                    # connect_robot(i)
                                else:
                                    'ROBOT not pinging :('
                                    self.win.robotHandler.time_attempted_to_establish_connection[i] = 0
                            else:
                                'robot alive  - ensure brakes are active and handle battery widget'
                                try:
                                    self.win.rosHandler.pubThread.__getattribute__(f'robo_{i}_trajectory_msg').emit(0, 0.0,
                                                                                                                0.0)  # stop movement through LEDNavPubTHread
                                except:
                                    "couldn't get pubthread of window to stop movement - need to start a game first"
                                    # print("couldn't get pubthread of window to stop movement - need to start a game first")

                            try:
                                'Update the battery labels'
                                if self.win.screenHandler.active_screen == main_screen:
                                    'Only update batteries if still in main screen'
                                    battery_lbl = self.win.screenHandler.active_screen.labels[i + 2]
                                    battery_state = self.win.robotHandler.battery_states[i]

                                    if self.win.robotHandler.robot_alive[i]:
                                        # print(f"robot {i} alive-------------")
                                        battery_lbl.setText(f"רובוט {i}\n{battery_state}%")  # battery level robot 1
                                        color = green_color
                                    elif response == 0:
                                        'robot pinging'
                                        # print(f"Pingingggggggggggggggggggggggggggggggggggggggggggg robot {i}")
                                        str = "  מתחבר " + "\n" + f"לרובוט {i}"
                                        # str = "המתן " + f"{round(reconnect_dt_thresh-reconnect_dt)}" + "\n" + f"לרובוט {i}"
                                        battery_lbl.setText(str)  # battery level robot 1
                                        color = orange_color
                                    else:
                                        # print(f"No response from robot {i}-----------------")
                                        battery_lbl.setText(f"  רובוט {i} \nמנותק")
                                        color = red_color
                                        # battery_lbl.hide()

                                    battery_lbl.setStyleSheet(f"color: rgba{color};"
                                                              f"background-color: rgba{title_lbl_bg_color}")
                                    battery_lbl.show()
                            except Exception as e:
                                print(f"Couldn't update label : {e}")
            time.sleep(dt)


    def ping(self, i):
        'check ping'
        addr = f"192.168.0.20{i}"
        # print(f"--Pinging {addr}")
        response = os.system(("ping -c 1 " + addr))
        return response
    def stop(self):
        self.running = False


class RosHandler:
    def __init__(self, win):
        self.win = win
        self.subThreadList = []
        self.pubThread = None

        print("initializing ros2")
        rclpy.init()  # initialize ros2
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor_thread = None
        self.start()

    def start(self):

        """
        Starts ros subscriber threads

        :return:
        """
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

    def create_sub_threads(self):
        print("Creating sub threads")
        for i in range(self.win.robotHandler.num_robots):
            rosSubscriberThread = Ros2QTSubscriber(self.win, i)
            self.subThreadList.append(rosSubscriberThread)
            rosSubscriberThread.start()

    def create_pub_thread(self, LEDNavPubThread):
        self.pubThread = LEDNavPubThread
        self.pubThread.start()


    def stop(self):
        """
        Stops ros subscriber threads

        :return:
        """
        self.pubThread.stop()

        for thread in self.subThreadList:
            thread.stop()


        # rclpy.shutdown()



class LightUpHandler(ABC):
    def __init__(self, gameEngine, params):
        self.gameEngine = gameEngine
        self.params = params
        self.num_to_color_dict = {
            0: "Off",
            1: "Green",
            2: "Blue",
            3: "Yellow",
            4: "Red",
            5: "White",
            6: "RES1",
            7: "RES2",
            8: "Blinking Success",
            9: "Blinking Failure",
        }

    @abstractmethod
    def generate_notes(self):
        pass



class LightUpHandlerBongos(LightUpHandler):
    def __init__(self, gameEngine, params, song_name):
        super().__init__(gameEngine, params)
        self.song_name = song_name
        self.LED_light_params_value = 0


    def generate_notes(self):
        """
        Create notes based on the beats of the song and hand them out evenly to the robots
        """

        'list of basic intervals in song'
        fileName = self.params["king_of_the_bongos_notes_file"]
        if self.params["use_pickled_song_note_data"]:
            notes_dict = pickle.load(open(fileName, "rb"))
        else:
            notes_dict = {}
            songs_list = ['kol_hakavod', 'pgisha_bamiluim', 'karnaval_banachal']

            'Create notes'
            for song in songs_list:
                song_notes = self.get_notes_based_on_phases(song)  # all note indices
                notes_dict[f"{song}"] = song_notes

            'Create pickle file'
            pickle.dump(notes_dict, open(fileName, "wb"))

        song_notes = notes_dict[f"{self.song_name}"]  # beats of the song
        self.gameEngine.song_note_indices = song_notes
        n_robots = self.gameEngine.win.robotHandler.num_robots
        'randomize notes between robots'
        if n_robots == 1:
            'single robot'
            no_robot_prob = 0.05
            single_robot_prob = 1 - no_robot_prob
            p_list = [no_robot_prob, single_robot_prob]
        elif n_robots == 2:
            'Two robot'
            # no_robot_prob = 0.05
            no_robot_prob = 0.0
            single_robot_prob = (1 - no_robot_prob)/4  # 25% chance per robot
            all_robots_prob = 1 - no_robot_prob - 2*single_robot_prob  # 50% chance for both
            p_list = [no_robot_prob, single_robot_prob, single_robot_prob, all_robots_prob]

        'Get notes based on difficulty'
        diff = self.gameEngine.difficulty
        modulus_var = self.params["KB_max_diff"] + 1 - diff
        modulus_var = 1
        updated_song_notes = []
        for index, nt in enumerate(song_notes):
            if index % modulus_var == 0:
                updated_song_notes.append(nt)
        print(f"\n\ndiff: {diff}. notes and new notes:\n {song_notes}\n {updated_song_notes}")
        lottery_winner = [np.random.choice(range(2**n_robots), 1, p=p_list)[0] for note in updated_song_notes]

        # 0 - None
        # 1 - only robot 0
        # 2 - only robot 1
        # 3 - robots 0 and 1
        # 4 - only robot 2
        # 5 - robots 1 and 2
        # 6 - robots 0 and 2
        # 7 - all robots

        robo_0_notes = [song_notes[ind] for ind, winner in enumerate(lottery_winner) if winner in [1, 3, 6, 7]]
        robo_1_notes = [song_notes[ind] for ind, winner in enumerate(lottery_winner) if winner in [2, 3, 5, 7]]
        robo_2_notes = [song_notes[ind] for ind, winner in enumerate(lottery_winner) if winner in [4, 5, 6, 7]]
        desired_notes = [robo_0_notes, robo_1_notes, robo_2_notes]

        return desired_notes

    def get_notes_based_on_phases(self, song_name: str):
        def get_closest_note_index(times_list, note_t):
            closest_note = min(times_list, key=lambda t: abs(t-note_t))
            return times_list.index(closest_note)

        if self.params["debug_initialization"]:
            print(f"Creating notes for song {song_name}")

        self.gameEngine.soundHandler.load_song(song_name)
        phase_list = self.gameEngine.soundHandler.phase_list
        notes_per_phase_list = self.gameEngine.soundHandler.notes_per_phase_list
        note_times = self.gameEngine.noteHandler.get_note_times(song_name)
        com_del = self.params["communication_delay"]/100  # seconds

        'light up every 8 iterations from phase beginning'
        if self.params["debug_initialization"]:
            print(f"My end notes are: {note_times[30:]}\nmy phase list: {phase_list}")

        notes = []
        for phase_note_list, bpm in phase_list:
            note_inds_sublist = [get_closest_note_index(note_times, note-com_del) for note in phase_note_list]
            notes += note_inds_sublist

        notes.append(len(note_times))  # add song end
        'Skip the initial start note'
        notes = notes[1:]

        return notes  # skip initial start note


class LightUpHandlerTrafficLight(LightUpHandler):
    def __init__(self, gameEngine, params, song_name):
        super().__init__(gameEngine, params)
        self.difficulty_to_LED_params = {
            1: {"avg_TOff": 4, "prob_green": 0.9},
            2: {"avg_TOff": 3.5, "prob_green": 0.8},
            3: {"avg_TOff": 3.5, "prob_green": 0.75},
            4: {"avg_TOff": 3.0, "prob_green": 0.75},
            5: {"avg_TOff": 2.5, "prob_green": 0.7},
            6: {"avg_TOff": 1.5, "prob_green": 0.7},
            7: {"avg_TOff": 1.4, "prob_green": 0.7},
        }
        self.LED_light_params_value = 1


        # for testing
        # self.difficulty_dict = {
        #     1: {"avg_TOff": 5, "prob_green": 0.5},
        #     2: {"avg_TOff": 4, "prob_green": 0.5},
        #     3: {"avg_TOff": 3.5, "prob_green": 0.5},
        #     4: {"avg_TOff": 3.5, "prob_green": 0.5},
        #     5: {"avg_TOff": 3, "prob_green": 0.5},
        #     6: {"avg_TOff": 2.5, "prob_green": 0.6},
        #     7: {"avg_TOff": 2, "prob_green": 0.5},
        #     # 9: {"avg_TOff": 2, "prob_green": 0.5},
        #     # 10: {"avg_TOff": 1.8, "prob_green": 0.5},
        # }

    def generate_notes(self, offset=0):
        difficulty = self.gameEngine.difficulty
        self.LED_light_params_value = difficulty

        LED_params = self.difficulty_to_LED_params[difficulty]
        avg_TOff = LED_params["avg_TOff"]  # [1.3, 4]
        prob = LED_params["prob_green"]
        num_items = 2*self.params["TL_diff_incr_thresh"]

        'Generate notes'
        variance = self.params["TL_variance"]
        preset_on_time = self.params["TL_TOn"]
        dt_btween_cols = self.params["TL_dt_btween_cols"]
        possible_color_keys = [1, 4]  # only green or red
        colors = np.random.choice(possible_color_keys, size=num_items, replace=True, p=[prob, 1-prob])

        TOff = np.random.uniform(avg_TOff - variance, avg_TOff + variance, num_items)

        time_prev = offset + dt_btween_cols
        time_list = []  # 5, 13, 21
        for t_off in TOff:
            time_prev += 2*t_off + dt_btween_cols
            time_list.append(time_prev)
        notes = zip(time_list, colors)

        return list(notes)


class LightUpHandlerSimonSays(LightUpHandler):
    def __init__(self, gameEngine, params, song_name):
        super().__init__(gameEngine, params)
        self.sequence = []
        self.iteration = -1

        'Translate difficulty to number of items'
        self.LED_light_params_value = 9

        'Translate difficulty to number of items'
        self.difficulty_to_num_items = {1: 2,
                                        2: 3,
                                        3: 4,
                                        4: 5,
                                        }


    def generate_notes(self):
        num_items = self.difficulty_to_num_items[self.gameEngine.difficulty]
        print(f"Number of colors in SS sequence: {num_items}!")
        possible_color_keys = [1, 2, 3, 4, 5]  # green, blue, orange, red, white

        if num_items < len(possible_color_keys):
            'Choose different colors'
            chosen_sequence = np.random.choice(possible_color_keys, num_items, replace=False)  # green, or red or white
        else:
            'Allow duplicate colors'
            chosen_sequence = np.random.choice(possible_color_keys, num_items, replace=True)  # green, or red or whited

        color_seq = [self.num_to_color_dict[el] for el in chosen_sequence]
        print("new color sequence: ", color_seq)

        'Generate dummies'
        dummies = []
        num_dummies = self.gameEngine.win.robotHandler.num_robots - 1
        for GT_color in chosen_sequence:
            possible_color_keys_tmp = copy.deepcopy(possible_color_keys)
            ind = possible_color_keys_tmp.index(GT_color)  # index to remove
            possible_color_keys_tmp.pop(ind)

            if num_dummies > 0:
                # print(f"possible colors: {[self.num_to_color_dict[col] for col in possible_color_keys_tmp]}")
                dummy = np.random.choice(possible_color_keys_tmp, num_dummies, replace=False)
                dummies.append(list(dummy))

                # print(f"color: {self.num_to_color_dict[GT_color]}, dummies: {[self.num_to_color_dict[dumdum] for dumdum in dummy]}")

                # print(f"removing {self.num_to_color_dict[ind]} from color list {possible_color_keys_tmp}")
        self.sequence = chosen_sequence

        return chosen_sequence, dummies


class MovesHandlerBongos:
    def __init__(self, gameEngine, params, song_name):
        self.gameEngine = gameEngine
        self.params = params
        self.song_name = song_name

    def get_autoclicker_moves(self, note_times):
        fileName = self.params["king_of_the_bongos_moves_file"]
        if self.params["use_pickled_song_move_data"]:
            moves_dict = pickle.load(open(fileName, "rb"))
        else:
            moves_dict = {}

            songs_list = ['kol_hakavod', 'pgisha_bamiluim', 'karnaval_banachal']

            'Create notes'
            for song in songs_list:
                song_moves = self.get_moves_based_on_phases(song)  # [[ac1_notes], [ac2_notes], [ac3_notes]]
                moves_dict[f"{song}"] = song_moves

            'Create pickle file'
            pickle.dump(moves_dict, open(fileName, "wb"))

        song_moves= moves_dict[f"{self.song_name}"]
        desired_moves = [song_moves[i] for i in range(self.gameEngine.win.robotHandler.num_robots)]

        return desired_moves

    def get_moves_based_on_phases(self, song_name: str):
        def go_home(start_beat):
            move = [3, start_beat, -1.0, -1.0]
            return move

        'PID commands'
        def CL_forward(start_beat, length, interruptable=False):
            move = [6, start_beat, length, -1]

            if interruptable:
                N = 8
            else:
                N = 9
            L = 0.2
            self.curr_note = start_beat + max(1, math.ceil((N / L) * abs(length)))  # it takes n notes to execute command of L cm
            return move

        def CL_rotate_cw(start_beat, length, interruptable=False):
            move = [7, start_beat, length, -1]
            if interruptable:
                N = 7
            else:
                N = 8
            R = 1.57
            self.curr_note = start_beat + max(1, math.ceil((N / R) * abs(length)))  # it takes n notes to execute command of R radians
            return move

        def CL_rotate_ccw(start_beat, length, interruptable=False):
            move = [8, start_beat, length, -1]
            if interruptable:
                N = 7
            else:
                N = 8
            R = 1.57
            self.curr_note = start_beat + max(1, math.ceil((N / R) * abs(length)))  # it takes n notes to execute command of R radians
            return move

        def mirror(moves):
            # move_types = {
            #     -1: "Reset signal",
            #     0: "Stop signal",
            #     1: "Open loop forward",
            #     2: "Open loop circle",
            #     5: "Backward movement",
            #     6: "Forward movement",
            #     7: "Clockwise movement",
            #     8: "Counter-clockwise movement",}
            reset_signal_MT = -1
            stop_signal_MT = 0
            OL_forward_MT = 1
            OL_circle_MT = 2
            CL_backward_MT = 5
            CL_forward_MT = 6
            CL_rotate_CW_MT = 7
            CL_rotate_CCW_MT = 8

            mirrored_moves = []
            for move in moves:
                beat, move_type, start, length, speed = move
                new_move = move.copy()

                if move_type == CL_rotate_CW_MT:
                    'Clockwise movement'
                    new_move[1] = CL_rotate_CCW_MT
                elif move_type == CL_rotate_CCW_MT:
                    new_move[1] = CL_rotate_CW_MT

                mirrored_moves.append(new_move)
            return mirrored_moves

        #region init
        print(f"Creating notes for song {song_name}")
        self.gameEngine.soundHandler.load_song(song_name)
        phase_list = self.gameEngine.soundHandler.phase_list
        note_inds = self.gameEngine.song_note_indices
        note_times = self.gameEngine.noteHandler.get_note_times(song_name)
        move_OL = self.params["KB_move_OL"]

        turn_speed = self.params["robot_turn_speed"]
        march_speed = self.params["robot_linear_speed"]
        user_reach = self.params["user_reach"]  # number between 80 and 120 cms
        testing_OL = False
        comm_delay = self.params["communication_delay"]/100  # milisec to sec
        # testing_OL = True
        #endregion
        if move_OL:
            self.curr_note = 0
            times = self.gameEngine.soundHandler.sounds_dict[song_name]["phase_list"]
            moves = self.gameEngine.soundHandler.sounds_dict[song_name]["moves_list"]

            ac1_moves = []
            for phase, subphase, moves_list in moves:
                t = times[phase][0][subphase] - comm_delay
                t_ind = note_times.index(t)
                for move in moves_list:
                    move_cp = copy.copy(move)
                    note_ind = move_cp[1]
                    move_cp.insert(0, t_ind + note_ind)
                    ac1_moves.append(move_cp)

            # ac2_moves = copy.copy(ac1_moves)
            ac2_moves = mirror(ac1_moves)
            ac3_moves = copy.copy(ac1_moves)
            'Open loop movement'
            # if song_name == Songs.KOL_HAKAVOD.value:
            #     dance_radius = np.deg2rad(8)
            #
            #     if testing_OL:
            #         ac1_moves = [
            #             line(start_beat=0, length=1000, frac=-1),
            #             line(start_beat=1, length=1000, frac=-1),
            #             line(start_beat=2, length=1000, frac=-1),
            #             line(start_beat=3, length=1000, frac=-1),
            #             line(start_beat=4, length=1000, frac=-1),
            #             line(start_beat=5, length=1000, frac=-1),
            #             line(start_beat=6, length=1000, frac=-1),
            #             line(start_beat=7, length=1000, frac=-1),
            #             line(start_beat=8, length=1000, frac=-1),
            #             line(start_beat=9, length=1000, frac=-1),
            #             line(start_beat=10, length=1000, frac=-1),
            #             line(start_beat=11, length=1000, frac=-1),
            #             line(start_beat=12, length=1000, frac=-1),
            #             line(start_beat=13, length=1000, frac=-1),
            #             line(start_beat=14, length=1000, frac=-1),
            #             line(start_beat=16, length=1000, frac=-1),
            #             line(start_beat=18, length=1000, frac=-1),
            #             line(start_beat=24, length=1000, frac=-1),
            #             line(start_beat=30, length=1000, frac=-1),
            #             line(start_beat=36, length=1000, frac=-1),
            #             line(start_beat=42, length=1000, frac=-1),
            #             line(start_beat=50, length=1000, frac=-1),
            #             line(start_beat=60, length=1000, frac=-1),
            #             line(start_beat=66, length=1000, frac=-1),
            #             line(start_beat=80, length=1000, frac=-1),
            #             line(start_beat=80, length=1000, frac=-1),
            #             line(start_beat=90, length=1000, frac=-1),
            #             line(start_beat=100, length=1000, frac=-1),
            #             line(start_beat=110, length=1000, frac=-1),
            #             line(start_beat=120, length=1000, frac=-1),
            #             line(start_beat=130, length=1000, frac=-1),
            #             line(start_beat=140, length=1000, frac=-1),
            #             line(start_beat=150, length=1000, frac=-1),
            #             line(start_beat=160, length=1000, frac=-1),
            #             line(start_beat=170, length=1000, frac=-1),
            #             line(start_beat=180, length=1000, frac=-1),
            #             line(start_beat=190, length=1000, frac=-1),
            #             line(start_beat=200, length=1000, frac=-1),
            #             line(start_beat=210, length=1000, frac=-1),
            #             line(start_beat=220, length=1000, frac=-1),
            #             ]
        else:
            L_line = user_reach / 200  # PID accepts values in meters
            R_circle = user_reach / 1.5

            halfLine = L_line / 2  # PID accepts values in meters
            dance_radius = np.deg2rad(8)
            initial_note = note_inds[1] - 4
            initial_note = 1
            self.curr_note = initial_note

            'Closed loop movement'
            if song_name == Songs.KOL_HAKAVOD.value:
            # if True:
                ac1_moves = [
                    CL_rotate_cw(start_beat=self.curr_note, length=dance_radius/2),
                    CL_rotate_ccw(start_beat=self.curr_note, length=dance_radius),
                    CL_rotate_cw(start_beat=self.curr_note, length=dance_radius),
                    CL_rotate_ccw(start_beat=self.curr_note, length=dance_radius),
                    CL_rotate_cw(start_beat=self.curr_note, length=dance_radius),
                    CL_rotate_ccw(start_beat=self.curr_note, length=dance_radius),
                    CL_rotate_cw(start_beat=self.curr_note, length=dance_radius),
                    CL_rotate_ccw(start_beat=self.curr_note, length=dance_radius/2),

                    CL_forward(start_beat=self.curr_note, length=1.2*halfLine),
                    CL_rotate_cw(start_beat=self.curr_note, length=np.pi/2),
                    CL_forward(start_beat=self.curr_note, length=halfLine),
                    CL_rotate_cw(start_beat=self.curr_note, length=(np.pi/2) + (np.pi/4)),
                    CL_forward(start_beat=self.curr_note, length=1.1*halfLine),
                    CL_rotate_cw(start_beat=self.curr_note, length=np.pi / 2),
                    CL_forward(start_beat=self.curr_note, length=1.2*halfLine),

                    # break free
                    # circle(start_beat=self.curr_note + 11, length=R_circle, ccw=True),
                    # CL_rotate_ccw(start_beat=self.curr_note + 16, length=1.5*np.pi),
                    # check from here
                    CL_rotate_cw(start_beat=self.curr_note, length=np.pi),
                    CL_forward(start_beat=self.curr_note, length=1.2*halfLine),
                    CL_rotate_ccw(start_beat=self.curr_note, length=0.75*np.pi),
                    CL_forward(start_beat=self.curr_note, length=halfLine),
                    CL_rotate_ccw(start_beat=self.curr_note, length=(np.pi/2) + (np.pi/4)),
                    CL_forward(start_beat=self.curr_note, length=0.5*halfLine),
                    CL_rotate_ccw(start_beat=self.curr_note, length=np.pi/4),
                             # go_home(start_beat=note_inds[-2])
                             ]  # centimeters
                ac2_moves = copy.copy(ac1_moves)
                # ac2_moves = mirror(ac1_moves)
                ac3_moves = copy.copy(ac1_moves)
            #
            elif song_name == Songs.PGISHA_BAMILUIM.value:
                ac1_moves = [CL_forward(start_beat=note_inds[1], length=L_line / 2),
                             CL_rotate_cw(start_beat=note_inds[3], length=np.pi / 2),
                             CL_rotate_cw(start_beat=note_inds[5], length=np.pi / 2),
                             CL_forward(start_beat=note_inds[7], length=L_line / 4),
                             go_home(start_beat=note_inds[-2])]  # centimeters
                ac2_moves = ac1_moves
                ac3_moves = ac1_moves
            #
            elif song_name == Songs.KARNAVAL_BANACHAL.value:
                ac1_moves = [
                    CL_rotate_cw(start_beat=self.curr_note, length=2*dance_radius),
                    CL_rotate_ccw(start_beat=self.curr_note, length=dance_radius/2),
                    CL_rotate_cw(start_beat=self.curr_note, length=dance_radius/2),
                    CL_rotate_ccw(start_beat=self.curr_note, length=dance_radius/2),
                    CL_rotate_cw(start_beat=self.curr_note, length=dance_radius/2),
                    CL_rotate_ccw(start_beat=self.curr_note, length=dance_radius/2),
                    CL_rotate_cw(start_beat=self.curr_note, length=dance_radius/2),
                    CL_rotate_ccw(start_beat=self.curr_note, length=dance_radius/2),
                    CL_rotate_cw(start_beat=self.curr_note, length=dance_radius/2),
                    CL_rotate_ccw(start_beat=self.curr_note, length=dance_radius/2),
                    CL_rotate_cw(start_beat=self.curr_note, length=dance_radius/2),
                    CL_rotate_ccw(start_beat=self.curr_note, length=dance_radius/2),
                    CL_rotate_cw(start_beat=self.curr_note, length=dance_radius/2),
                    CL_rotate_ccw(start_beat=self.curr_note, length=dance_radius/2),
                    CL_rotate_cw(start_beat=self.curr_note, length=dance_radius/2),
                    CL_rotate_ccw(start_beat=self.curr_note, length=dance_radius/2),
                    CL_rotate_cw(start_beat=self.curr_note, length=dance_radius/2),
                    CL_rotate_ccw(start_beat=self.curr_note, length=2*dance_radius),

                    CL_forward(start_beat=self.curr_note+2, length=1.2 * halfLine, interruptable=True),
                    CL_rotate_cw(start_beat=self.curr_note, length=np.pi / 2, interruptable=True),
                    CL_forward(start_beat=self.curr_note, length=halfLine),
                    CL_rotate_cw(start_beat=self.curr_note, length=(np.pi / 2) + (np.pi / 4)),
                    CL_forward(start_beat=self.curr_note, length=1.1 * halfLine),
                    CL_rotate_cw(start_beat=self.curr_note, length=np.pi / 2),
                    CL_forward(start_beat=self.curr_note, length=1.2 * halfLine),

                    # break free
                    # circle(start_beat=self.curr_note + 11, length=R_circle, ccw=True),
                    # CL_rotate_ccw(start_beat=self.curr_note + 16, length=1.5*np.pi),
                    # check from here
                    CL_rotate_cw(start_beat=self.curr_note, length=np.pi),
                    CL_forward(start_beat=self.curr_note, length=1.2 * halfLine),
                    CL_rotate_ccw(start_beat=self.curr_note, length=0.75 * np.pi),
                    CL_forward(start_beat=self.curr_note, length=halfLine),
                    CL_rotate_ccw(start_beat=self.curr_note, length=(np.pi / 2) + (np.pi / 4)),
                    CL_forward(start_beat=self.curr_note, length=0.5 * halfLine),
                    CL_rotate_ccw(start_beat=self.curr_note, length=np.pi / 4),
                    # go_home(start_beat=note_inds[-2])
                ]  # centimeters
                ac1_moves = mirror(ac1_moves)
                ac2_moves = mirror(ac1_moves)
                ac3_moves = ac1_moves


        return [ac1_moves, ac2_moves, ac3_moves]


class ScoreHandler(ABC):
    @abstractmethod
    def __init__(self, game_engine, params):
        self.game_engine = game_engine
        self.params = params
        self.reaction_times = {0: [], 1: [], 2: []}
        self.score = 0

    def get_score(self):
        return self.score

    def get_difficulty(self):
        return self.game_engine.difficulty

    @abstractmethod
    def reset_score(self):
        pass


class ScoreHandlerBongos(ScoreHandler):
    def __init__(self, game_engine, params):
        super().__init__(game_engine, params)
        self.woohoo_reaction_time_thresh = self.params["KB_woohoo_RT_thresh"]  # sec
        self.woohoo_modulus_thresh = self.params["KB_success_grace"]

    def increase_score(self, clicker_ind: int, reaction_time: float):

        self.score += int(10 / reaction_time)  # score based on reaction time
        # print(f"curr score: {self.score}")

    def reset_score(self):
        print("resetting score")
        # self.reaction_times = {0: {}, 1: {}, 2: {}}
        self.score = 0
        self.game_engine.win.screenHandler.refresh_screen()


class ScoreHandlerTrafficLight(ScoreHandler):
    def __init__(self, game_engine, params):
        super().__init__(game_engine, params)
        self.woohoo_modulus_thresh = self.params["TL_success_grace"]  # sec
        self.boohoo_modulus_thresh = self.params["TL_fail_grace"]
        self.diff_incr_thresh = self.params["TL_diff_incr_thresh"]
        self.max_combo = 0

    def increase_score(self, clicker_ind: int, reaction_time: float):

        self.score += 1 # streak
        n_notes_played = self.game_engine.win.rosHandler.pubThread.num_notes_played
        # self.reaction_times[clicker_ind].append((n_notes_played, round(reaction_time, 2)))
        # print("Appending reaction time: P{")
        # increase_dif = self.score == self.diff_incr_thresh
        # required_difficulty = self.game_engine.difficulty + int(self.score / self.diff_incr_thresh)
        # if increase_dif:
        #     self.score = 1
        #     self.game_engine.increase_difficulty()  # handles maximal value
        #     print(f"curr score: {self.score} req dif: {required_difficulty}")

        self.game_engine.win.screenHandler.refresh_screen()

    def reset_score(self):
        # self.reaction_times = {0: {}, 1: {}, 2: {}}
        self.score = 0
        self.game_engine.win.screenHandler.refresh_screen()


class ScoreHandlerSimonSays(ScoreHandler):
    def __init__(self, game_engine, params):
        super().__init__(game_engine, params)
        self.max_combo = 0

    def increase_score(self, clicker_ind):
        self.score += 1
        print(f"Another step in the color sequence! score: {self.score}")
        required_difficulty = self.score
        # if self.game_engine.difficulty < required_difficulty:
        self.game_engine.increase_difficulty()

    def reset_score(self):
        print("resetting score")
        # self.reaction_times = {0: {}, 1: {}, 2: {}}
        self.score = 0


class GameEngine(ABC):
    def __init__(self, win, params, start_from_sec, song_name):
        self.win = win
        self.params = params
        self.song_name = song_name

        self.soundHandler = SoundHandler(self, params)
        self.soundHandler.load_song(song_name, start_from_sec)

        self.bongos_min_difficulty = self.params["KB_min_diff"]
        self.bongos_max_difficulty = self.params["KB_max_diff"]

        self.traffic_light_min_difficulty = self.params["TL_min_diff"]
        self.traffic_light_max_difficulty = self.params["TL_max_diff"]

        self.simon_says_min_difficulty = self.params["SS_min_diff"]
        self.simon_says_max_difficulty = self.params["SS_max_diff"]
        self.running = False

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def increase_difficulty(self):
        pass

    @abstractmethod
    def decrease_difficulty(self):
        pass

    def stop_game(self):
        if self.running:
            self.running = False
        else:
            print('already called stop from another thread for this game engine')


    def stop(self):
        print("Stopping ROS")
        self.win.rosHandler.stop()

        print("Stopping music")
        self.soundHandler.stop_music()


class GameEngineBongos(GameEngine):
    def __init__(self, win, params, start_from_sec, song_name):
        super().__init__(win, params, start_from_sec, song_name)
        self.noteHandler = NoteHandler(self, self.params, self.soundHandler)
        self.lightupHandler = LightUpHandlerBongos(self, self.params, song_name)
        self.movesHandler = MovesHandlerBongos(self, params, song_name)
        self.scoreHandler = ScoreHandlerBongos(self, params)

        self.difficulty = self.win.db.get_parameter("KB_difficulty")  # blink difficulty - either 1, 2, or 3

        self.note_times = self.noteHandler.get_note_times(song_name)
        self.notes = self.lightupHandler.generate_notes()
        self.moves = self.movesHandler.get_autoclicker_moves(self.note_times)

        self.win.rosHandler.create_pub_thread(LEDNavPubThreadBongos(win, self, self.params))

    def start(self):
        # Create AutoClickers and their animations
        self.running = True
        self.win.screenHandler.activate_screen(Screens.KING_OF_THE_BONGOS)

        self.win.rosHandler.create_sub_threads()
        self.soundHandler.play(self.song_name)  # initialize mixer and load music
        self.win.screenHandler.start_prog_bar_bongos()  # connect progress bar to music mixer progress

    def stop(self):
        self.win.screenHandler.stop_prog_bar_bongos()
        super().stop()

        self.scoreHandler.reset_score()

    def increase_difficulty(self):
        if self.difficulty < self.bongos_max_difficulty - 1:
            self.difficulty += 1
            print(f"updated KB clicker dif to {self.difficulty}")
        self.win.screenHandler.refresh_screen()

    def decrease_difficulty(self):
        if self.difficulty > self.bongos_min_difficulty + 1:
            self.difficulty -= 1
            print(f"updated KB clicker dif to {self.difficulty}")
        self.win.screenHandler.refresh_screen()


class GameEngineTrafficLight(GameEngine):

    def __init__(self, win, params, start_from_sec, song):
        super().__init__(win, params, start_from_sec, song.value)
        self.noteHandler = NoteHandler(self, params, self.soundHandler)
        self.lightupHandler = LightUpHandlerTrafficLight(self, params, song.value)
        # self.movesHandler = MovesHandlerBongos(self, song_name)
        self.scoreHandler = ScoreHandlerTrafficLight(self, params)
        self.difficulty = self.traffic_light_min_difficulty

        self.win.rosHandler.create_pub_thread(LEDNavPubThreadTrafficLight(win, self, self.params))

        # self.notes = self.lightupHandler.generate_notes()

    def start(self):
        self.running = True
        self.t0 = time.time()
        self.win.rosHandler.create_sub_threads()
        self.soundHandler.play(self.song_name)  # initialize mixer and load music
        self.win.screenHandler.start_prog_bar_TL()  # connect progress bar to music mixer progress

    def stop(self):
        self.win.screenHandler.stop_prog_bar_TL()
        super().stop()
        self.scoreHandler.reset_score()

    def set_difficulty(self, dif):
        if self.params["TL_min_diff"] <= dif <= self.params["TL_max_diff"]:
            self.difficulty = dif
        else:
            raise Exception("invalid difficulty level")

    def increase_difficulty(self):
        if self.difficulty < self.traffic_light_max_difficulty:
            self.difficulty += 1

            print(f"updated TL clicker dif to {self.difficulty}")

        self.win.screenHandler.refresh_screen()

    def decrease_difficulty(self):
        if self.difficulty > self.traffic_light_min_difficulty:
            self.difficulty -= 1
            # self.difficulty = 1
            print(f"Downdated TL clicker dif to {self.difficulty}")

        self.win.screenHandler.refresh_screen()

class GameEngineSimonSays(GameEngine):
    def __init__(self, win, params, start_from_sec, song):
        super().__init__(win, params, start_from_sec, song.value)
        self.noteHandler = NoteHandler(self, params, self.soundHandler)
        self.lightupHandler = LightUpHandlerSimonSays(self, params, song.value)
        self.scoreHandler = ScoreHandlerSimonSays(self, params)
        self.difficulty = self.simon_says_min_difficulty
        self.phase = None
        # self.notes = self.lightupHandler.generate_notes()
        self.win.rosHandler.create_pub_thread(LedNavPubThreadSimonSays(win, self, params))


    def start(self):
        self.running = True

        self.win.rosHandler.create_sub_threads()
        # self.soundHandler.play(self.song_name)  # initialize mixer and load music
        self.win.screenHandler.start_prog_bar_SS()  # connect progress bar to music mixer progress

    def stop(self):
        self.win.screenHandler.stop_prog_bar_SS()
        print("Stopping ROS")
        self.win.rosHandler.stop()
        'Not user the super().stop since no music to stop'
        # super().stop()
        self.scoreHandler.reset_score()

    def increase_difficulty(self):
        if self.difficulty <= self.params["SS_max_diff"]:
            self.difficulty += 1

            print(f"updated SS clicker dif to {self.difficulty}")
        # self.win.screenHandler.refresh_screen()

    def decrease_difficulty(self):
        if self.difficulty > self.params["SS_min_diff"]:
            self.difficulty -= 1
            print(f"updated SS clicker dif to {self.difficulty}")
        # self.win.screenHandler.refresh_screen()







