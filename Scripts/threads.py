import pygame.mixer
from PyQt5 import QtCore
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QMessageBox
import time
import numpy as np

from Scripts.enums import SSPhase
from Scripts.ros_threads import QT2RosPublisher
from threading import Thread


class LEDNavPubThread(QThread):
    '''
        publisher engine (real time and simulation).
        Sends light commands to LED publisher based on notes
        Sends cmd_vel commands to nav publisher based on movements
    '''
    robo_0_LED_msg = QtCore.pyqtSignal(int)
    robo_1_LED_msg = QtCore.pyqtSignal(int)
    robo_2_LED_msg = QtCore.pyqtSignal(int)

    robo_0_trajectory_msg = QtCore.pyqtSignal(int, float, float)  # movement_type, movement_length, movement-velocity
    robo_1_trajectory_msg = QtCore.pyqtSignal(int, float, float)  # movement_type, movement_length, movement-velocity
    robo_2_trajectory_msg = QtCore.pyqtSignal(int, float, float)  # movement_type, movement_length, movement-velocity

    def __init__(self, win, gameEngine, params):
        QThread.__init__(self)
        self.win = win
        self.gameEngine = gameEngine
        self.params = params
        self.ros_publishers = []
        self.t0 = -1
        self.time_note_played = self.gameEngine.win.robotHandler.num_robots * [-1]

        print("Creating publshers of robots")
        for i in range(self.gameEngine.win.robotHandler.num_robots):
            'Create all 3 publishers and connect the robot commands to their execution methods'
            pub = QT2RosPublisher(params, i)

            self.__getattribute__(f'robo_{i}_LED_msg').connect(pub.run_led_publisher)
            self.__getattribute__(f'robo_{i}_trajectory_msg').connect(pub.run_traj_publisher)
            self.ros_publishers.append(pub)

        self.threadactive = False
        self.last_note_time = -1
        self.num_notes_played = 0

    def lightup_robot(self, ind: int, state: int, lightup_time=0):
        word_dict = self.gameEngine.lightupHandler.num_to_color_dict
        led_light_params_value = self.gameEngine.lightupHandler.LED_light_params_value

        'Cipher'
        led = led_light_params_value*10 + state*1

        'Lightup'
        self.__getattribute__(f'robo_{ind}_LED_msg').emit(led)  # emits to LED publisher

        if state != 0:
            'Update note playing time lights activate'
            # print(f"curr time: {time.time()} game engine t0: {t0} diff: {lightup_time}")
            self.time_note_played[ind] = lightup_time
            if lightup_time == 0:
                print(f"robot {ind} state={word_dict[state]}")
            else:
                print(f"robot {ind} state={word_dict[state]} at t={round(lightup_time,1 )}[s] ")
        else:
            print("Turning lights off")


    def move_robots(self, pubs, upcoming_moves, curr_iteration):
        move_vals_to_strings = ["Stop", "OL Line", "OL Circle", "Return home", "Freestyle", "backward CL", "forward CL",
                                "Rotate CW CL", "rotate CCW CL", "Reset"]
        for i, move_i in enumerate(upcoming_moves): # upcoming moves is a list with a move for each robot
            if move_i:
                iter_i, type_i, _, length_i, velocity_i = move_i

                if iter_i == curr_iteration:
                    'moving on current iteration - move and update next move'
                    if self.params["debug_movement"]:
                        # print(f"Emitting move type {type_i} of length {length_i} with velocity {velocity_i}")
                        print(f"Emitting move {move_vals_to_strings[type_i]}")
                    pubs[i].emit(type_i, length_i, velocity_i)  # emits to trajectory publisher

                    # if self.gameEngine.moves[i]:
                    try:
                        'move moves exist'
                        upcoming_moves[i] = self.gameEngine.moves[i].pop(0)
                        # if self.params["debug_movement"]:
                        #     print(f"upcoming for robot {i}: {upcoming_moves[i]} moves left: {self.gameEngine.moves}")
                    # else:
                    except Exception as e:
                        'No more moves - robot finished moving'
                        if self.params["debug_movement"]:
                            print(f"Robot {i} finished moving!! :: {e}")

        return upcoming_moves

    def stop(self):
        'Finished playing'
        self.threadactive = False
        for i in range(self.gameEngine.win.robotHandler.num_robots):
            'Disconnect all 3 publishers from the robot commands to their execution methods'
            pub = self.ros_publishers[i]
            print(f"disconnecting publishers for robot {i}")
            self.__getattribute__(f'robo_{i}_trajectory_msg').emit(0, 0.0, 0.0)
            self.__getattribute__(f'robo_{i}_trajectory_msg').emit(0, 0.0, 0.0)
            self.__getattribute__(f'robo_{i}_trajectory_msg').emit(0, 0.0, 0.0)
            self.__getattribute__(f'robo_{i}_trajectory_msg').disconnect(pub.run_traj_publisher)
            self.__getattribute__(f'robo_{i}_LED_msg').disconnect(pub.run_led_publisher)
            pub.stop()

        # rclpy.shutdown()
        self.quit()


class LEDNavPubThreadBongos(LEDNavPubThread):
    def __init__(self, win, gameEngine, params):
        super().__init__(win, gameEngine, params)

    def run(self):
        self.game_thread = Thread(target=self.run_in_thread)  # listen to tf and click messages
        self.game_thread.start()  # starts the infinite loop. since we don't call thread.join() we don't wait for execute

    def run_in_thread(self):
        try:
            # region Initialize clicker moves and notes for song
            note_times = self.gameEngine.note_times
            # note_inds = self.gameEngine.soundHandler.note_inds
            note_inds = self.gameEngine.notes
            if self.params["debug_initialization"]:
                print(f"notes: {note_times} \nnotes inds: {note_inds}")
            start_from_sec = self.params["KB_start_from_sec"]
            'Get first note index'
            first_note_ind = []
            remaining_note_inds = []
            for note_inds_i in note_inds:
                missed_notes_i = 0
                for ind in note_inds_i:
                    "append note if it's after the start from sec time"
                    t_note = note_times[ind]
                    if t_note >= start_from_sec:
                        first_note_ind.append(ind)
                        remaining_note_inds.append(note_inds_i[missed_notes_i:])
                        # missed_notes.append(missed_notes_i)
                        break
                    missed_notes_i += 1

            self.gameEngine.notes = remaining_note_inds
            upcoming_note_inds = [nts.pop(0) for nts in self.gameEngine.notes]

            'Find initial iteration'
            dt_finder = lambda x: (x - start_from_sec)
            note_time_rel_to_start_time = [dt_finder(el) for el in note_times]  # moved notes
            iteration = next(i for i, el in enumerate(note_time_rel_to_start_time) if el > 0)  # first note time ahead of start
            remaining_note_times = note_times[iteration:]
            if self.params["debug_initialization"]:
                print(f"starting from sec: {start_from_sec}")
                print(f"raw note times: {note_times}")
                print(f"Remaining note times: {remaining_note_times}")
                print(f"raw notes indices: {note_inds}")
                print(f"remaining notes indices: {remaining_note_inds}")
                print(f"Initial iteration: {iteration}")
                print(f"Upcoming notes: {upcoming_note_inds}")

                move_vals_to_strings = ["Stop", "OL Line", "OL Circle", "Return home", "Freestyle", "backward CL", "forward CL", "Rotate CW CL", "rotate CCW CL", "Reset"]
                moves_string = "\n"
                for i, rob_i_moves in enumerate(self.gameEngine.moves):
                    moves_string += f"\nRobot {i} moves: \n"
                    for t, type, note, length, vel in rob_i_moves:
                        moves_string += f"\tt={t} | {move_vals_to_strings[type]} | note={note} | L={np.round(length, 2)} | vel={vel}\n"
                print(moves_string)

            upcoming_moves = []

            for el in self.gameEngine.moves:
                upcoming_moves.append(el.pop(0))
            # upcoming_moves = [el.pop(0) for el in self.gameEngine.moves]


            move_pubs = []
            for i in range(self.win.robotHandler.num_robots):
                move_pub = self.__getattribute__(f'robo_{i}_trajectory_msg')
                move_pub.emit(-1, -1.0, -1.0)  # reset locations through LEDNavPubTHread
                move_pubs.append(move_pub)

            while not self.gameEngine.soundHandler.music_playing:
                print("Waiting for music to start")
                time.sleep(0.1)

            self.t0 = time.time() - start_from_sec # time since run start- regardless of song progress
            self.gameEngine.t0 = self.t0

            while self.gameEngine.running and not self.gameEngine.win.robotHandler.com_error:
                'Sleep till note time'
                if iteration == len(note_times):
                    print(f"Reached end of bongos song since iteration {iteration} is end of notes {len(note_times)}")
                    break

                time_to_sleep = self.t0 - time.time() + note_times[iteration]

                if time_to_sleep < 0:
                    print(f"Cannot sleep a negative amount of time till next note: {time_to_sleep}. Skipping")
                    iteration += 1
                    continue
                else:
                    # print(f"Time is {time.time()-self.t0} sleeping till iter={iteration} at {note_times[iteration]} for {time_to_sleep}[s]")
                    pass

                time.sleep(time_to_sleep)
                self.play_notes(upcoming_note_inds, iteration)
                # print(f"relevant notes: {remaining_notes} next note inds :{next_note_inds}")
                if self.params["move_robots"]:
                    upcoming_moves = self.move_robots(move_pubs, upcoming_moves, iteration)
                iteration += 1

            print("game ending bongos")
            self.win.screenHandler.resetting_game = True

        except Exception as e:
            print(e)
            exit(4)

    def play_notes(self, upcoming_notes, curr_iteration):
        'Play notes'
        off_state = 0
        green_state = 1

        # check if need to play robot note this iteration
        notes_playing_this_iter = [i for i, next_note_i in enumerate(upcoming_notes) if next_note_i == curr_iteration]

        for i in range(self.gameEngine.win.robotHandler.num_robots):
            t0_raw = self.gameEngine.win.rosHandler.pubThread.t0
            lightup_time = time.time() - t0_raw  # time-t0_raw gives us the seconds since start of thread loop, song_t0 gives the time from song's true beginning
            if i == 0:
                print(f"iter {curr_iteration}: t={round(lightup_time, 1)}[s]")
                # print(f"t={round(lightup_time, 1)}[s] | lightup time: {lightup_time} t0: {t0} songt0:{song_t0}")

            if i in notes_playing_this_iter:
                self.lightup_robot(ind=i, state=green_state, lightup_time=lightup_time)
                self.num_notes_played += 1
                try:
                    upcoming_notes[i] = self.gameEngine.notes[i].pop(0)
                except Exception as e:
                    print(f"Couldn't pop note: {e}")
            else:
                'No note to play this iteration - do nothing'


class LEDNavPubThreadTrafficLight(LEDNavPubThread):
    def __init__(self, win, gameEngine, params):
        super().__init__(win, gameEngine, params)

    def run(self):
        self.t0 = time.time()
        self.threadactive = True
        for i in range(self.gameEngine.win.robotHandler.num_robots):
            self.__setattr__(f'robo_{i}_thread', Thread(target=self.run_in_thread, args=(i, self.t0)) ) # listen to tf and click messages
            self.__getattribute__(f'robo_{i}_thread').start()  # starts the infinite loop. since we don't call thread.join() we don't wait for execute

    def run_in_thread(self, ind, t0):
        try:
            max_difficulty = self.params["TL_max_diff"]
            max_streak = self.params["TL_diff_incr_thresh"]
            gameover_time = self.params["TL_gameover_time"]
            sleep_till_prior_time = 1
            last_note_time = 0.0
            min_sleep_time = 0.5
            max_sleep_time = 5
            while self.gameEngine.running and self.threadactive and not self.gameEngine.win.robotHandler.com_error:
                tot_game_time = time.time() - t0
                new_notes = self.gameEngine.lightupHandler.generate_notes(offset=last_note_time)
                notes = new_notes
                print(f"new notes: {notes}")

                'Play all notes'
                for (note_t, note_color), (next_note_t, _) in zip(notes[:-1], notes[1:]):
                    # Sleep till note time
                    time_to_sleep = min(t0 - time.time() + note_t, max_sleep_time)

                    if time_to_sleep < 0:
                        print(f"Cannot sleep a negative amount of time till curr note of TL: {time_to_sleep}. Skipping")
                        continue

                    'Sleep till note time'
                    stop_game = self.pseudosleep(time_to_sleep)
                    if stop_game:
                        break

                    prev_difficulty = self.gameEngine.difficulty

                    last_note_time = time.time() - t0
                    self.lightup_robot(ind=ind, state=note_color, lightup_time=last_note_time)

                    time_to_sleep_prior = min(t0 - time.time() + next_note_t - sleep_till_prior_time, max_sleep_time)

                    if time_to_sleep_prior < 0:
                        print(f"Cannot sleep a negative amount of time till next note of TL: {time_to_sleep_prior}. Skipping")
                        continue

                    stop_game = self.pseudosleep(time_to_sleep_prior)
                    if stop_game or (tot_game_time > gameover_time):
                        break

                    self.lightup_robot(ind=ind, state=0, lightup_time=time.time() - t0)
                    updated_difficulty_cond = prev_difficulty != self.gameEngine.difficulty

                    if updated_difficulty_cond:
                        break

                streak = self.gameEngine.scoreHandler.get_score()
                diff = self.gameEngine.scoreHandler.get_difficulty()

                top_score_reached = diff >= max_difficulty and streak == max_streak
                end_of_time_reached = tot_game_time > gameover_time

                print(f"tot game time: {tot_game_time} end game time: {gameover_time} done? {end_of_time_reached}")
                print(f"streak: {streak} difficulty: {diff}")

                if top_score_reached or end_of_time_reached:
                    break


            print("game ending Traffic Light")

            if not self.win.screenHandler.reset_in_progress:
                self.win.screenHandler.resetting_game = True

        except Exception as e:
            print(e)
            exit(4)

    def pseudosleep(self, T):
        t = 0
        dt = 0.1  # sec
        stop_game = False
        while t <= T:
            time.sleep(dt)
            t += dt

            'Stop if game ended criterion reached'
            stop_signal = not (self.threadactive and self.gameEngine.running) or self.gameEngine.win.robotHandler.com_error
            if stop_signal:
                stop_game = True
                break

        return stop_game


class LedNavPubThreadSimonSays(LEDNavPubThread):
    def __init__(self, win, gameEngine, params):
        super().__init__(win, gameEngine, params)
        self.move_to_next_color = False
        self.start_new_sequence = False


    def run(self):
        self.game_thread = Thread(target=self.run_in_thread)  # listen to tf and click messages
        self.game_thread.start()  # starts the infinite loop. since we don't call thread.join() we don't wait for execute

    def run_in_thread(self):
        self.threadactive = True
        self.t0 = time.time()
        self.gameEngine.t0 = self.t0

        'No music for SS'
        song_length = self.gameEngine.soundHandler.get_song_length()
        song_length = self.params["SS_gameover_time"]

        TOn_host = self.params["TOn_host"]
        TOff_host = self.params["TOff_host"]
        T_change_phase = self.params["T_change_phase"]
        TOn_user = self.params["TOn_user"]
        TOff_user = self.params["TOff_user"]
        time_btwn_color_clicks = self.params["SS_time_between_color_clicks"]
        host_index = self.params["SS_host_index"]
        try:
            while self.gameEngine.running and self.threadactive and (time.time()-self.t0 < song_length) and (self.gameEngine.difficulty <= self.params['SS_max_diff']) and (not self.gameEngine.win.robotHandler.com_error):
                'Get a list of tuples of 3 for the current sequence, '
                color_sequence, dummies = self.gameEngine.lightupHandler.generate_notes()
                # num_sequence = self.gameEngine.lightupHandler.generate_notes(num_items=self.gameEngine.difficulty)

                'Generate a shuffled list of colors'
                shuffled_list = []
                if dummies:
                    'the first elements of each tuple are the real color'
                    list_of_lightups = list(zip(color_sequence, dummies))
                    for l in list_of_lightups:
                        shufd_list_tmp = []
                        for element in l:
                            if hasattr(element, '__len__'):
                                for subel in element:
                                    shufd_list_tmp.append(subel)
                            else:
                                shufd_list_tmp.append(element)
                        shuffled_list.append(shufd_list_tmp)
                else:
                    shuffled_list = [[col] for col in color_sequence]

                names = self.gameEngine.lightupHandler.num_to_color_dict
                named_list = []

                'Print list of colors (for debugging)'
                for sublist in shuffled_list:
                    tmp = []
                    for el in sublist:
                        tmp.append(names[el])
                    named_list.append(tmp)
                print(f"shufld list: {named_list}")

                print("--Teaching phase!--")
                'Stop listening to callbacks'
                self.gameEngine.phase = SSPhase.TEACHING_PHASE

                stop_game = self.pseudosleep(T_change_phase, song_length)

                if stop_game:
                    break
                # stop_game = self.pseudosleep(TOff_host, song_length)
                # if stop_game:
                #     break

                self.gameEngine.soundHandler.play_my_turn()
                self.gameEngine.lightupHandler.LED_light_params_value = 9  # faster pace

                'Wait a sec after declaring turn before lighting up'
                stop_game = self.pseudosleep(time_btwn_color_clicks, song_length)
                if stop_game:
                    break

                'Exhibit lights to user'
                for color in color_sequence:
                    self.lightup_robot(ind=host_index, state=color)

                    'Wait long enough for quick lightup to end + a short break'
                    stop_game = self.pseudosleep(TOn_host, song_length)

                    if stop_game:
                        break
                if stop_game:
                    break
                print("--Recall phase!--")
                self.gameEngine.soundHandler.play_your_turn()
                self.gameEngine.lightupHandler.LED_light_params_value = 8 # slower pace
                stop_game = self.pseudosleep(T_change_phase, song_length)

                if stop_game:
                    break

                'Start listening to LED callbacks'
                self.gameEngine.phase = SSPhase.RECALL_PHASE_CLICK_ENABLED

                'Iterations of a given color sequence'
                for iter in range(len(shuffled_list)):
                    self.gameEngine.lightupHandler.iteration = iter
                    'for each color in sequence'
                    curr_lights = shuffled_list[iter][:self.gameEngine.win.robotHandler.num_robots]
                    # true_color = curr_lights[0]
                    np.random.shuffle(curr_lights)

                    stop_game = self.pseudosleep(time_btwn_color_clicks, song_length)

                    if stop_game:
                        print("stopping game 2")
                        break

                    'light up all robots for current recall phase iteration'
                    self.gameEngine.phase = SSPhase.RECALL_PHASE_CLICK_ENABLED
                    for i in range(self.gameEngine.win.robotHandler.num_robots):
                        lightup_time = time.time() - self.gameEngine.win.rosHandler.pubThread.t0
                        self.lightup_robot(ind=i, state=curr_lights[i], lightup_time=lightup_time)

                    stop_game = self.pseudosleep(TOn_user, song_length)

                    if stop_game or self.start_new_sequence:
                        print("stopping game 1")
                        if self.start_new_sequence:
                            self.start_new_sequence = False
                        break
                    else:
                        'Sleep between colors'
                        # print("Lightup time completed and nothing clicked")
                        # for i in range(self.gameEngine.win.robotHandler.num_robots):
                        #     self.gameEngine.win.rosHandler.pubThread.lightup_robot(ind=i, state=0)

            print("game ending Simon Says")
            self.win.screenHandler.resetting_game = True

        except Exception as e:
            print(e)
            exit(4)

    def pseudosleep(self, T, song_length):
        t = 0
        dt = 0.1  # sec
        stop_game = False
        # print(f"sleeping from time {t} to {T}")
        while t <= T:
            time.sleep(dt)
            t += dt

            'Stop if game ended criterion reached'
            stop_signal = not (self.threadactive and self.gameEngine.running)

            if stop_signal:
                print(f"Received stop signal in pseudosleep. threadactive: {self.threadactive} | gameEngine running: {self.gameEngine.running} | time constraint: {time.time()-self.t0} < {song_length} max diff passed: {self.gameEngine.difficulty} > {self.params['SS_max_diff']}")
                stop_game = True
                break

            'Move on to next color if robots clicked by user'
            if self.move_to_next_color or self.start_new_sequence:
                # print("moving on to next color")
                self.move_to_next_color = False
                break

        return stop_game


class ProgressBarThreadBongos(QThread):
    dt = 0.2  # sec
    progress = QtCore.pyqtSignal(int)

    def __init__(self, win, music):
        QThread.__init__(self)
        self.win = win
        self.music = music

        self.threadactive = False

    def run(self):
        self.threadactive = True

        while pygame.mixer.get_init() is None:
            print("waiting for mixer to initialize")
            time.sleep(self.dt)

        while True:
            if self.win.gameEngine.soundHandler.song_finished():
                'Exit if song finished playing'
                return
            try:
                curr_progress = self.win.gameEngine.soundHandler.get_progress()
                self.progress.emit(int(curr_progress))
            except:
                print("Has no SoundHandler attribute, must have ended in the middle of run")

            time.sleep(self.dt)
            if self.win.gameEngine is None:
                break


    def stop(self):
        self.threadactive = False
        self.quit()


class ProgressBarThreadTL(QThread):
    dt = 0.2  # sec
    progress = QtCore.pyqtSignal(int)

    def __init__(self, win):
        QThread.__init__(self)
        self.win = win
        self.params = self.win.params
        self.threadactive = False

    def run(self):
        self.threadactive = True

        while pygame.mixer.get_init() is None:
            print("waiting for mixer to initialize")
            time.sleep(self.dt)

        while True:
            if self.win.gameEngine is None:
                break

            max_score = self.params["TL_max_score"]
            max_diff = self.params["TL_max_diff"]
            coeff = 100/max_score # proportion out of 100%
            curr_progress = self.win.gameEngine.scoreHandler.get_score() * coeff
            # print(f"Current progress bar score: {self.win.gameEngine.scoreHandler.get_score()}")
            self.progress.emit(int(curr_progress))



            time.sleep(self.dt)



    def stop(self):
        self.threadactive = False
        self.quit()


class ProgressBarThreadSS(QThread):
    dt = 0.2  # sec
    progress = QtCore.pyqtSignal(int)

    def __init__(self, win):
        QThread.__init__(self)
        self.win = win
        self.params = self.win.params
        self.threadactive = False

    def run(self):
        self.threadactive = True

        'Dont play music for SS'
        # while pygame.mixer.get_init() is None:
        #     print("waiting for mixer to initialize")
        #     time.sleep(self.dt)

        while True:
            try:
                if self.win.gameEngine is None:
                    break

                coeff = 100/self.params["SS_max_diff"] # proportion out of 100%
                curr_progress = self.win.gameEngine.scoreHandler.get_score() * coeff
                # print(f"Current progress bar score: {self.win.gameEngine.scoreHandler.get_score()}")
                self.progress.emit(int(curr_progress))

                time.sleep(self.dt)
            except Exception as e:
                print(f"Breaking out of progress update of SS since: {e}")


    def stop(self):
        self.threadactive = False
        self.quit()

