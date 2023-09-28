from abc import ABC, abstractmethod

import numpy as np
from PyQt5 import QtCore
from PyQt5.QtCore import QThread
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float64
from std_msgs.msg import Float32MultiArray
# from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import BatteryState

from Scripts.enums import Games, SSPhase


class Ros2QTSubscriber(QThread):
    camera_msg = QtCore.pyqtSignal(float, float, float)
    clicked_msg = QtCore.pyqtSignal(int)

    def __init__(self, win, ind):
        QThread.__init__(self)
        self.threadactive = False
        self.win = win
        self.ind = ind

        if self.win.active_game == Games.KING_OF_THE_BONGOS:
            cbHandler = CBHandlerBongos(self.win.gameEngine, self.ind)
        elif self.win.active_game == Games.TRAFFIC_LIGHT:
            cbHandler = CBHandlerTL(self.win.gameEngine, self.ind)
        elif self.win.active_game == Games.SIMON_SAYS:
            cbHandler = CBHandlerSS(self.win.gameEngine, self.ind)
        else:
            cbHandler = None

        self.rosSubscriber = SubscriberNode(self.win.gameEngine, self.win.params, self.ind, cbHandler)
        self.win.rosHandler.executor.add_node(self.rosSubscriber)

        self.prev_clicked_state = 0
        # self.wrong_clicks = 0
        self.dt = 0.05

    def run(self):
        self.threadactive = True
        # listener_thread = Thread(target=self.run_ros_listener) # listen to tf and click messages
        # listener_thread.start() # starts the infinite loop. since we don't call thread.join() we don't wait for execute

        # while self.win.gameEngine.running and self.threadactive and rclpy.ok():
        while rclpy.ok() and self.threadactive:
            # rclpy.spin_once(self.rosSubscriber)
            x = self.rosSubscriber.x
            y = self.rosSubscriber.y
            theta = self.rosSubscriber.theta
            self.camera_msg.emit(x, y, theta)


        self.win.rosHandler.executor.remove_node(self.rosSubscriber)
        self.rosSubscriber.stop()
        self.quit()

    def stop(self):
        self.threadactive = False


class SubscriberNode(Node):
    def __init__(self, gameEngine, params, ind, cb_handler):
        super().__init__(f'SubscriberNode_{ind}')
        self.gameEngine = gameEngine
        self.params = params
        self.ind = ind
        self.cbHandler = cb_handler
        freq = self.params["subscriber_frequency"]
        tf_node_name = f'tf'
        self.x = 0
        self.y = 0
        self.theta = 0
        self.tf_subscriber = self.create_subscription(TFMessage, tf_node_name, self.tf_listener_callback, freq)
        self.num_prev_led_states = 5

        led_state_node_name = f'turtlebot_{ind}/led_state'
        self.led_state = 0
        self.led_state_subscriber = self.create_subscription(Int16, led_state_node_name, self.led_listener_callback, freq)
        self.prev_led_state = self.num_prev_led_states*[self.led_state]

        clicked_node_name = f'turtlebot_{ind}/clicked'
        self.prev_clicked_state = 0
        self.clicked_state = 0

        self.clicked_subscriber = self.create_subscription(Int16, clicked_node_name, self.clicked_listener_callback, freq)
        self.dt_clicks = self.params["dt_clicks"]
        self.off_state = self.get_color_state_from_dict("Off")
        self.green_state = self.get_color_state_from_dict("Green")
        self.red_state = self.get_color_state_from_dict("Red")
        self.blue_state = self.get_color_state_from_dict("Blue")
        self.white_state = self.get_color_state_from_dict("White")
        self.yellow_state = self.get_color_state_from_dict("Yellow")


    def tf_listener_callback(self, msg):
        header = msg.transforms[0].header
        translations = msg.transforms[0].transform.translation # x, y, z 3D vector
        rotations = msg.transforms[0].transform.rotation # x, y, z, w quaternion

        x = round(translations.x, 4) * 100 # cm
        y = round(translations.y, 4) * 100 # cm
        theta = round(rotations.z, 4)
        # print(f"\nRobot is currently at ({x}, {y}) with an angle of {theta}")

        self.x = x
        self.y = y
        self.theta = theta

    def led_listener_callback(self, msg):
        t0 = self.gameEngine.win.rosHandler.pubThread.t0

        self.prev_led_state.pop(0)
        self.prev_led_state.append(self.led_state)
        self.led_state = msg.data
        reset_states = False
        # if self.ind == 1:
        #     print(f"LED{self.ind}: prev states: {self.prev_led_state}, curr state: {self.led_state}")

        'Update if clicked this note'
        if self.gameEngine.win.active_game == Games.SIMON_SAYS and self.gameEngine.phase == SSPhase.RECALL_PHASE_CLICK_ENABLED:
            dict = self.gameEngine.lightupHandler.num_to_color_dict
            seq = self.gameEngine.lightupHandler.sequence
            iter = self.gameEngine.lightupHandler.iteration
            true_color = seq[iter]
            is_final_color_in_sequence = iter == (len(seq) - 1)

            prev_was_true = any(state == true_color for state in self.prev_led_state) and (self.led_state == self.off_state)
            prev_led_state =0

            for prev_state in self.prev_led_state[::-1]:
                if prev_state != 0:
                    prev_led_state = prev_state
                    break
            reset_states = self.cbHandler.led_cb(t0, prev_led_state, prev_was_true, [true_color, seq, is_final_color_in_sequence, iter])  # reaction time, is final, true color, clicked color, distraction, sequence

        elif self.gameEngine.win.active_game == Games.KING_OF_THE_BONGOS or self.gameEngine.win.active_game == Games.TRAFFIC_LIGHT:
            clicked = self.clicked_state == 1 or self.prev_clicked_state == 1
            prev_green = any(state == self.green_state for state in self.prev_led_state)
            prev_red = any(state == self.red_state for state in self.prev_led_state)
            was_green = self.led_state == self.off_state and prev_green
            was_red = self.led_state == self.off_state and prev_red

            reset_states = self.cbHandler.led_cb(t0, clicked, was_green, [was_red, self.gameEngine.difficulty])
            # print(
            # f"Prev TL led states: {self.prev_led_state} prev green: {prev_green} prev red: {prev_red} curr state: {self.led_state}")

        if reset_states:
            self.reset_prev_states()

        self.prev_clicked_state = self.clicked_state
        self.clicked_state = 0

    def clicked_listener_callback(self, msg):
        # region Useful vars
        self.clicked_state = msg.data
        click_time = time.time() - self.gameEngine.win.rosHandler.pubThread.t0

        note_play_time = self.gameEngine.win.rosHandler.pubThread.time_note_played[self.ind]
        reaction_time = click_time - note_play_time
        if self.clicked_state and not self.prev_clicked_state:
            print(
            f"curr clicked: {self.clicked_state}, prev state: {self.prev_clicked_state} clicked_color: {self.led_state} prev_colors: {self.prev_led_state} RT: {reaction_time}")

        # if all(state == self.off_state for state in self.prev_led_state):
        #     'Lights are off'
            # print("Lights are off - no clicked callback")
            # return

        'Handle clicks when light is on'
        if self.gameEngine.win.active_game == Games.SIMON_SAYS:
            if self.gameEngine.phase == SSPhase.RECALL_PHASE_CLICK_ENABLED:
                # print(f"phase is {self.gameEngine.phase.value}, checking click callback")
                seq = self.gameEngine.lightupHandler.sequence
                iter = self.gameEngine.lightupHandler.iteration
                true_color = seq[iter]
                is_true = self.led_state == true_color
                is_final_color_in_sequence = iter == (len(seq) - 1)
                # print(f"iteration num: {iter} seq length: {len(seq)}")

                # print(f"curr clicked: {self.clicked_state}, prev state: {self.prev_clicked_state} clicked_color: {self.led_state}")
                # print(f"Clicked: {is_true}, robot!!")

                reset_states = self.cbHandler.clicked_cb(reaction_time, self.clicked_state, is_true, [true_color, self.led_state, seq, is_final_color_in_sequence, iter])  # reaction time, is final, true color, clicked color, distraction, sequence
            else:
                # print(f"phase is {self.gameEngine.phase.value}, Not entering click callback")
                "Do nothing - don't turn off robots, don't award points and don't change difficulty"
        elif self.gameEngine.win.active_game == Games.KING_OF_THE_BONGOS or self.gameEngine.win.active_game == Games.TRAFFIC_LIGHT:
            is_green = self.led_state == self.green_state
            is_red = self.led_state == self.red_state
            reset_states = self.cbHandler.clicked_cb(reaction_time, self.clicked_state, is_green, [is_red, self.gameEngine.difficulty])

        # if reset_states:
        #     self.reset_prev_states()

    def reset_prev_states(self):
        self.prev_led_state = self.num_prev_led_states * [self.off_state]
        self.cbHandler.just_clicked = False
        print(f"Resetting click states of {self.ind}")

    def get_color_state_from_dict(self, state: str):
        dict = self.gameEngine.lightupHandler.num_to_color_dict
        state_ind = list(dict.keys())[list(dict.values()).index(state)]
        return state_ind

    def stop(self):
        self.tf_subscriber.destroy()
        self.clicked_subscriber.destroy()
        self.destroy_node()


class LivelihoodSubscriberNode(Node):
    def __init__(self, win, ind):
        super().__init__(f'LivelihoodSubscriber_{ind}')
        self.win = win
        self.ind = ind

        freq = 10

        livelihood_node_name = f'turtlebot_{ind}/alive'
        self.livelihood_subscriber = self.create_subscription(Int16, livelihood_node_name, self.livelihood_callback, freq)

        battery_node_name = f'turtlebot_{ind}/battery_state'
        self.battery_subscriber = self.create_subscription(BatteryState, battery_node_name, self.battery_callback, freq)

    def livelihood_callback(self, msg):
        # print("received livelihood msg")
        is_alive = msg.data  # always 1
        self.win.robotHandler.notify_robot_alive(self.ind)

    def battery_callback(self, msg):
        battery_remaining = int(msg.percentage)
        self.win.robotHandler.update_battery_state(self.ind, battery_remaining)

    def stop(self):
        self.livelihood_subscriber.destroy()
        self.destroy_node()


class CBHandler(ABC):
    """
    Handles callback for awarding points after successful/failed clicks
    """

    def __init__(self, gameEngine, ind):
        self.gameEngine = gameEngine
        self.ind = ind
        self.grace_iters = 0
        self.just_clicked = False

    @abstractmethod
    def led_cb(self, t0, clicked, was_green, was_red):
        pass

    def clicked_cb(self, t0, clicked, is_green, is_red):
        pass


class CBHandlerBongos(CBHandler):
    def led_cb(self, t0, clicked, was_green, was_red):
        'No consequences to missing notes in this mode'
        reset_states = False
        if not clicked:
            if was_green:
                reset_states = True

            if self.just_clicked:
                print(f"Ignore missed note of {self.ind} since just clicked={self.just_clicked}.")
                pass
            elif was_green:
                curr_t = time.time() - self.gameEngine.t0
                new_datum = (curr_t, -1, self.gameEngine.win.rosHandler.pubThread.num_notes_played, -1, self.ind)
                self.append_datum(new_datum)
        return reset_states

    def clicked_cb(self, reaction_time, clicked, is_green, iter_data):
        'Turn robot lights off, play woohoo and update score'
        reset_states = False
        if clicked:
            curr_t = time.time() - self.gameEngine.t0

            if is_green:
                'Correct click'
                self.just_clicked = True
                self.success(reaction_time)
                new_datum = (curr_t, 1, self.gameEngine.win.rosHandler.pubThread.num_notes_played, round(reaction_time, 2), self.ind)
                reset_states = True
            else:
                'Wrong click'
                new_datum = (curr_t, 0, self.gameEngine.win.rosHandler.pubThread.num_notes_played, -1, self.ind)
                # reset_states = True

            self.append_datum(new_datum)
        return reset_states

    def success(self, reaction_time):
        print("clicked a green :)")

        self.gameEngine.scoreHandler.increase_score(self.ind, reaction_time)

        thresh = self.gameEngine.scoreHandler.woohoo_reaction_time_thresh
        if reaction_time <= thresh:
            self.gameEngine.soundHandler.play_woohoo()

    def failure(self):
        pass

    def append_datum(self, datum):
        interaction_strings = {
            -1: "missed click",
            0: "wrong click",
            1: "correct click",
        }
        self.gameEngine.scoreHandler.reaction_times[self.ind].append(datum)
        if self.gameEngine.params["debug_database"]:
            print(
                f"Appending reaction time for KB: \n\trobo {self.ind} {interaction_strings[datum[1]]} at t={datum[0]} with reaction time  {datum[3]}. total notes played: {datum[2]}")


class CBHandlerTL(CBHandler):
    def led_cb(self, t0, clicked, was_green, iter_data):
        reset_states = False
        was_red, difficulty = iter_data
        'Handle missed notes'
        if not clicked:
            # print(f"----------- just clicked {self.ind} = {self.just_clicked}")
            if was_red or was_green:
                reset_states = True

            if self.just_clicked:
                print(f"Ignore missed note of {self.ind} since just clicked={self.just_clicked}.")
                pass
            elif was_red:
                print("Avoided a red :)")
                self.success(t0, clicked_green=False, missed_red=True, difficulty=difficulty, ind=self.ind)
            elif was_green:
                print(f"Missed a green :( ")
                self.failure(missed_green=True, clicked_red=False, difficulty=difficulty, ind=self.ind)
        return reset_states

    def clicked_cb(self, reaction_time, clicked, is_green, iter_data):
        reset_states = False
        is_red, difficulty = iter_data
        'Handle hit notes'
        if clicked:
            'turn off robot'
            if is_green:
                print(f"clicked a green :) reaction time: {np.round(reaction_time, 2)} seconds")
                self.just_clicked = True
                self.success(reaction_time, clicked_green=True, missed_red=False, difficulty=difficulty, ind=self.ind)
                reset_states = True
            elif is_red:
                print("clicked a red :(")
                self.just_clicked = True
                self.failure(missed_green=False, clicked_red=True, difficulty=difficulty, ind=self.ind)
                reset_states = True
            else:
                'Clicked off state'
                self.failure(missed_green=False, clicked_red=False, difficulty=difficulty, ind=self.ind)
            print("turning robot off")
            self.gameEngine.win.rosHandler.pubThread.lightup_robot(ind=self.ind, state=0)

            return reset_states

    def success(self, reaction_time, clicked_green, missed_red, difficulty, ind):
        'Avoided clicking red - update score and play woohoo'
        curr_t = time.time() - self.gameEngine.t0
        if missed_red or clicked_green:

            if missed_red:
                new_datum = (curr_t, round(reaction_time, 2), 2, difficulty, ind)
            else:
                new_datum = (curr_t, round(reaction_time, 2), 1, difficulty, ind)

            self.append_datum(new_datum)

        'Increase score and difficulty level, play woohoo'
        self.gameEngine.scoreHandler.increase_score(self.ind, reaction_time)
        score = self.gameEngine.scoreHandler.score
        consecutive_success_needed = self.gameEngine.scoreHandler.woohoo_modulus_thresh
        if score % consecutive_success_needed == 0:
            self.gameEngine.soundHandler.play_woohoo()

        incr_thresh = self.gameEngine.params["TL_diff_incr_thresh"] + difficulty
        if score % incr_thresh == 0:
            self.gameEngine.increase_difficulty()

    def failure(self, missed_green, clicked_red, difficulty, ind):
        'Missed a green - reset score and play boohoo'
        curr_t = time.time() - self.gameEngine.t0
        if clicked_red and not missed_green:
            new_datum = (curr_t, -1, -2, difficulty, ind)
        elif not clicked_red and missed_green:
            new_datum = (curr_t, -1, -1, difficulty, ind)
        else:
            new_datum = (curr_t, -1, 0, difficulty, ind)

        self.append_datum(new_datum)

        'Update score difficulty and woohoo'
        if self.gameEngine.scoreHandler.score == 0:
            print("decreasing difficulty - two or more consecutive fails")

            'two consecutive failures'
            self.gameEngine.decrease_difficulty()
            self.gameEngine.soundHandler.play_uh_oh()
        else:
            print("single fail - resetting score")
            self.gameEngine.scoreHandler.reset_score()

    def append_datum(self, datum):
        interaction_strings = {
            -2: "clicked red",
            -1: "missed green",
            0: 'hit off state',
            1: "hit green",
            2: "missed red",
        }
        self.gameEngine.scoreHandler.reaction_times[self.ind].append(datum)

        if self.gameEngine.params["debug_database"]:
            print(f"Appending reaction time for TL: \n\trobo {self.ind} {interaction_strings[datum[2]]} at t={round(datum[0], 1)} with reaction time {datum[1]} at difficulty {datum[3]}")

class CBHandlerSS(CBHandler):
    def __init__(self, gameEngine, ind):
        super(CBHandlerSS, self).__init__(gameEngine, ind)
        self.clicked_true_color = False

    def led_cb(self, t0, prev_led_state, prev_was_true, iter_data):
        'Handle missed notes'
        reset_states = False
        #print(f"Update to LED state. clicked: {clicked}")
        # print(f"clicked: {self.clicked_true_color}, prev was true: {prev_was_true}")
        curr_t = time.time() - self.gameEngine.t0
        if self.clicked_true_color:
            self.clicked_true_color = False
            print("Safe because just clicked true color!")
            return

        if prev_was_true and not self.clicked_true_color:
            if self.grace_iters == 0:
                'Missed your chance!'
                print("should have clicked. Missed your chance to prove to the king who's boss!")
                self.failure()
            else:
                print("Single miss (robot of true color turned off without you clicking) - grace saved you this time!")
                self.grace_iters -= 1

            true_col, seq, is_final_color_in_seq, iteration = iter_data
            new_datum = (curr_t, int(true_col), -1, list(seq), is_final_color_in_seq, iteration, -1, self.ind)
            self.append_datum(new_datum)
            reset_states = True
        else:
            'Dont care about wrong missed notes'

        return reset_states
    def clicked_cb(self, reaction_time, clicked, is_true, iter_data):
        'Handle clicked notes'
        reset_states = False
        curr_t = time.time() - self.gameEngine.t0
        if clicked:
            self.gameEngine.phase = SSPhase.RECALL_PHASE_CLICK_DISABLED

            print("Robot clicked, turning off all robots!")
            for i in range(self.gameEngine.win.robotHandler.num_robots):
                self.gameEngine.win.rosHandler.pubThread.lightup_robot(ind=i, state=0)

            true_col, clicked_col, seq, is_final_color_in_seq, iteration = iter_data

            if is_true:
                # print(
                #     f"clicked: {clicked}, should click: {is_true} is final in seq: {is_final_color_in_seq} | rt: {t0}")
                self.grace_iters = self.gameEngine.params["SS_fail_grace"]
                self.clicked_true_color = True

                if is_final_color_in_seq:
                    print("last color in sequence. success!")
                    self.success(reaction_time)

                else:
                    print("Another step in the color sequence for now")
                    self.success(reaction_time, last_color=False)
            else:
                'clicked wrong color'
                print("Failed - clicked wrong color")
                self.failure()

            new_datum = (curr_t, int(true_col), clicked_col, list(seq), is_final_color_in_seq, iteration, round(reaction_time, 2), self.ind)
            self.append_datum(new_datum)
            reset_states = True
            self.gameEngine.win.rosHandler.pubThread.move_to_next_color = True

        return reset_states

    def success(self, reaction_time, last_color=True):
        """
        Update score in database.
        If difficulty level increased, woohoo
        :param reaction_time:
        :return:
        """
        self.gameEngine.soundHandler.play_woohoo()

        if last_color:
            self.gameEngine.scoreHandler.increase_score(self.ind)

    def failure(self):
        """
        Reset score

        :return:
        """
        self.gameEngine.soundHandler.play_uh_oh()
        self.gameEngine.scoreHandler.reset_score()
        self.gameEngine.win.rosHandler.pubThread.start_new_sequence = True

    def append_datum(self, datum):
        self.gameEngine.scoreHandler.reaction_times[self.ind].append(datum)

        if self.gameEngine.params["debug_database"]:
            print(f"Appending reaction time for SS: \n\trobo {self.ind}  seq={datum[3]}, true {datum[1]} clicked {datum[2]} at t={round(datum[0], 2)}.reaction time={datum[6]}")

class QT2RosPublisher(QThread):
    '''
    Publish LED commands to ROS topic
    '''

    dt = 0.1  # sec
    LED_msg = QtCore.pyqtSignal(Int16)
    traj_msg = QtCore.pyqtSignal(int, float, float)

    def __init__(self, params, ind):
        QThread.__init__(self)
        self.curr_LED_cmd = 0
        self.curr_nav_cmd = 0
        self.publisher = publisherNode(params, ind)

    def run_led_publisher(self, cmd):
        self.publisher.publish_led_command(cmd)
        self.curr_LED_cmd = cmd

    def run_traj_publisher(self, type, length, vel):
        # self.curr_nav_cmd = [x, 0.0, 0.0, 0.0, 0.0, rz]
        self.publisher.publish_traj_command(type, length, vel)

    def stop(self):
        self.run_led_publisher(0)
        self.run_traj_publisher(0, 0.0, 0.0)
        time.sleep(0.1)
        self.publisher.stop()
        self.quit()


class publisherNode(Node):
    def __init__(self, params, ind):
        super().__init__(f"LEDMasterNode_{ind}")
        self.params = params
        freq = self.params["publisher_frequency"]
        self.led_msg = Int16()
        self.LED_cmd_publisher_ = self.create_publisher(Int16, f"turtlebot_{ind}/LED_cmd", freq)

        self.traj_msg = Float32MultiArray()
        self.traj_pub = self.create_publisher(Float32MultiArray, f"turtlebot_{ind}/cmd_traj", freq)

    def publish_led_command(self, led_state_on):
        self.led_msg.data = led_state_on

        if rclpy.ok():
            self.LED_cmd_publisher_.publish(self.led_msg)

    def publish_traj_command(self, movement_type, movement_length, movement_vel):
        data = [float(movement_type), float(movement_length), float(movement_vel)]

        self.traj_msg.data = data

        if rclpy.ok():
            self.traj_pub.publish(self.traj_msg)

    def stop(self):
        self.LED_cmd_publisher_.destroy()
        self.traj_pub.destroy()
        self.destroy_node()


