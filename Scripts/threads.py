from PyQt5 import QtCore
from PyQt5.QtCore import QThread
import time
import numpy as np
import pyautogui


class LEDNavPubThread(QThread):
    '''
        publisher engine (real time and simulation).
        Sends light commands to LED publisher based on notes
        Sends cmd_vel commands to nav publisher based on movements
    '''
    LED_msg = QtCore.pyqtSignal(int)
    nav_msg = QtCore.pyqtSignal(float, float)  # x, rz

    def __init__(self, win, clicker):
        QThread.__init__(self)
        self.win = win
        self.clicker = clicker
        self.ind = self.clicker.ind

        if self.win.DEBUG:
            print("creating blink thread and starting")

        if not self.win.simActive:
            self.ros_publisher = QT2RosPublisher(self.ind) # ROS node
            self.LED_msg.connect(self.ros_publisher.run_led_publisher)
            self.nav_msg.connect(self.ros_publisher.run_nav_publisher)
        else:
            self.LED_msg.connect(self.start)

            'Move like ants instead. No point in simulating real life commands.'
            # self.nav_msg.connect(self.LED_publisher.run_nav_publisher)
        self.difficulty = 1
        self.threadactive = False
        self.last_note_time = -1

    def set_difficulty(self, dif):
        if 1 <= dif <= 10:
            self.difficulty = dif
        else:
            raise Exception("invalid difficulty level")

    def increase_difficulty(self):
        if self.difficulty < 10:
            self.difficulty += 1
            print(f"updated clicker dif to {self.difficulty}")

    def decrease_difficulty(self):
        if self.difficulty > 1:
            self.difficulty -= 1
            print(f"updated clicker dif to {self.difficulty}")

    def run(self):
        self.threadactive = True
        try:
            if self.win.active_game == 'normal game':
                self.run_normal_game()
            elif self.win.active_game == 'combo game':
                self.run_combo_game()

        except Exception as e:
            print(e)
            exit(4)

    def run_normal_game(self):
        # region Initialize clicker moves and notes for song
        t0 = time.time() - self.win.song_engine.start_from_second
        self.win.song_t0 = t0

        note_times = self.win.song_engine.note_times
        notes = self.win.song_engine.notes[self.ind]
        moves_x, moves_rz = self.win.song_engine.moves[self.ind]

        'Get first note index'
        first_note_ind = next(i for i in range(len(note_times)) if note_times[i] > time.time() - t0)
        # note_times = [note_times[i] - self.win.song_engine.start_from_second for i in range(len(note_times))
        #               if i >= first_note_ind]
        notes = [nt for nt in notes if nt > first_note_ind]
        next_note_ind = notes.pop(0)

        iteration = 1 + first_note_ind
        print(f"t0: {self.win.song_engine.start_from_second}")
        print(f"note_times: {note_times[first_note_ind:]}")
        print(f"iteration: {iteration}")
        print(f"first_note_ind: {first_note_ind}")
        print(f"notes: {notes}")
        print(f"next_note_ind: {next_note_ind}")
        # endregion

        while self.win.running and self.threadactive:
            'Sleep till note time'
            time_to_sleep = t0 - time.time() + note_times[iteration]
            time.sleep(time_to_sleep)
            self.play_notes(next_note_ind, iteration)

            if iteration == next_note_ind:
                'Played note'
                next_note_ind = notes.pop(0) if notes else -1  # get note from list if list isn't empty

            self.move_robot(moves_x[iteration], moves_rz[iteration])
            iteration += 1

    def run_combo_game(self):
        t0 = time.time()
        self.win.song_t0 = t0
        while self.win.running and self.threadactive:
            notes, moves = self.win.song_engine.generate_commands()
            for (sign, TOn, TOff) in notes:
                'Sleep till note time'
                self.cipher_and_play(1, self.difficulty, sign=sign)
                time.sleep(TOn)

                self.cipher_and_play(0, self.difficulty)
                time.sleep(TOff)

    def play_notes(self, next_note, iteration):
        'Play notes'
        played_note = False
        if next_note == iteration:
            played_note = True
            # print(f"Playing note {next_note} on time {time.time() - t0}")
            # if self.win.DEBUG:

            if self.win.simActive:
                activate_LED_thread = Thread(target=self.clicker.animation.activateLEDs)
                activate_LED_thread.start()  # blink in separate background thread
            else:
                self.cipher_and_play(1, self.difficulty)
        else:
            if self.win.simActive:
                pass
            else:
                self.cipher_and_play(0, self.difficulty)

        return played_note

    def cipher_and_play(self, state, difficulty, sign=1):
        'sign: 1 = green, -1 = red'
        led = difficulty*10 + state*1
        self.LED_msg.emit(sign * led)  # emits to LED publisher
        self.win.time_note_played[self.ind] = time.time() - self.win.song_t0

    def move_robot(self, x_cmd, rz_cmd):
        if not self.win.DEBUG:
            if (x_cmd != 0.0) or (rz_cmd != 0.0):
                print(f"Sending commands (x, rz) = {x_cmd, rz_cmd}")

        self.nav_msg.emit(x_cmd, rz_cmd)
        # endregion

    def stop(self):
        'Finished playing'
        self.threadactive = False
        self.nav_msg.disconnect(self.ros_publisher.run_nav_publisher)
        self.LED_msg.disconnect(self.ros_publisher.run_led_publisher)
        self.ros_publisher.stop()
        self.quit()


class ACMoveThread(QThread):
    '''
    Movement simulator engine.
    Visually moves clicker animations - based on move engine in simulation and on camera tf coords in real life.
    Emitted coordinates are handled by the AutoClicker self.move method.
    '''

    clicker_pos = QtCore.pyqtSignal(int, int, int)
    dt = 0.05
    initialOffsets = [(0, 0), (-200, 200), (200, 200)]
    noise = 100

    def __init__(self, win, clicker):
        QThread.__init__(self)
        self.win = win
        self.clicker = clicker
        self.ind = self.clicker.ind

        # Initialize constants
        self.screen_w, self.screen_h = pyautogui.size()
        self.clicker_radius = self.clicker.animation.helper.radius
        self.min_y, self.max_y = 510, self.screen_h
        self.min_x, self.max_x = self.clicker_radius, self.screen_w
        self.min_x, self.max_x = self.clicker_radius, 1300
        self.curr_x = 0
        self.curr_y = 0
        self.curr_ax = 0
        self.curr_ay = 0
        self.threadactive = False

    def run(self):
        try:
            self.threadactive = True
            if self.win.simActive:
                self.move_ants_simulator()
            else:
                # move_based_on_inputs is invoked when new messages arrive
                pass

        except Exception as e:
            print(e)
            exit(4)

    def move_ants_simulator(self):
        def in_bounds_x(a_x):
            touched_wall = False
            curr_x = self.curr_x
            new_x = curr_x + 0.5 * a_x * (self.dt ** 2)
            if new_x > self.max_x:
                new_x = self.max_x
                touched_wall = True
            elif new_x < self.min_x:
                new_x = self.min_x
                touched_wall = True

            return new_x, touched_wall

        def in_bounds_y(a_y):
            touched_wall = False
            curr_y = self.curr_y
            new_y = curr_y + 0.5 * a_y * (self.dt ** 2)
            if new_y > self.max_y:
                new_y = self.max_y
                touched_wall = True
            elif new_y < self.min_y:
                new_y = self.min_y
                touched_wall = True
            return new_y, touched_wall

        def sense_collision(ax, ay, i):
            # check if clicker hit the walls
            new_x, collided_wall_x = in_bounds_x(ax)
            new_y, collided_wall_y = in_bounds_y(ay)

            collided_clicker = False
            num_of_clickers = len(self.win.screens["game"].autoclickers)
            if num_of_clickers > 1:
                'Only check collisions if there are other robots around'
                for j in range(num_of_clickers):
                    if i != j:
                        other_clicker = self.win.screens["game"].autoclickers[j]
                        other_click_x, other_click_y = other_clicker.x, other_clicker.y
                        if ((new_x - other_click_x)**2) + ((new_y - other_click_y)**2) <= (self.clicker_radius ** 2)+self.clicker_radius/1000:
                            collided_clicker = True
                            if self.win.DEBUG:
                                print(f"clicker {i} collided when moving into {j}")
            return new_x, new_y, collided_wall_x, collided_wall_y, collided_clicker

        x = int(self.win.width / 2) + self.initialOffsets[self.ind][0]
        y = int(self.win.height / 2) + self.initialOffsets[self.ind][1]
        self.move_animation(x, y)

        x0, y0 = self.curr_x, self.curr_y
        if self.win.DEBUG:
            print(f"clicker starting position: {x0, y0}")

        self.curr_ax = 0
        self.curr_ay = 0

        while self.win.running:
            if not self.win.paused:
                rand = np.random.randint(-self.noise, self.noise, 2)
                # rand = [30, 20]

                new_ax = self.curr_ax + rand[0]
                new_ay = self.curr_ay + rand[1]

                new_x, new_y, col_x, col_y, col_clicker = sense_collision(new_ax, new_ay, self.ind)
                if col_x:
                    new_ax = 0
                if col_y:
                    new_ay = 0
                if col_clicker:
                    new_ax = 0
                    new_ay = 0
                else:
                    'No collision'
                    self.curr_ax = new_ax
                    self.curr_x = new_x
                    self.curr_ay = new_ay
                    self.curr_y = new_y

                self.move_animation(self.curr_x, self.curr_y)
                time.sleep(self.dt)

            # print(f"dx12={np.abs(self.curr_x[0]-self.curr_x[1])}")
            # print(f"dx23={np.abs(self.curr_x[1]-self.curr_x[2])}")
            # print(f"dx13={np.abs(self.curr_x[2]-self.curr_x[0])}")

    def move_animation(self, x, y):
        self.curr_x = x
        self.curr_y = y

        self.clicker_pos.emit(x, y, self.ind)

    def move_based_on_real_inputs(self, x, y, theta):
        if not self.win.simActive:
            center_screen_bias_x = 900
            center_screen_bias_y = 750
            MEF = 1 # make camera movements larger
            ac_social_dist = self.clicker_radius
            self.clicker_pos.emit(center_screen_bias_y + MEF*y + ac_social_dist, center_screen_bias_x + MEF*x, 0)
            self.clicker_pos.emit(center_screen_bias_y + MEF*y, center_screen_bias_x + MEF*x, 1)
            self.clicker_pos.emit(center_screen_bias_y + MEF*y - ac_social_dist, center_screen_bias_x + MEF*x, 2)



    def stop(self):
        self.threadactive = False
        self.quit()


class ProgressBarThread(QThread):
    dt = 0.2  # sec
    progress = QtCore.pyqtSignal(int)

    def __init__(self, win, music):
        QThread.__init__(self)
        self.win = win
        self.music = music

        self.threadactive = False

    def run(self):
        self.threadactive = True
        song_length = self.win.song_engine.tot_song_len
        while self.win.running:
            if self.music.get_pos() == -1:
                # song finished running
                # self.win.btn_stop_game()
                return

            curr_progress = self.music.get_pos() / 1000  # in seconds
            progress_percentage = curr_progress / song_length * 100
            self.progress.emit(int(progress_percentage))

            time.sleep(self.dt)


    def stop(self):
        self.threadactive = False
        self.quit()

#region ROS2

import rclpy
from rclpy.node import Node
from threading import Thread
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage


class SubscriberNode(Node):
    def __init__(self, win, ind):
        super().__init__('SubscriberNode')
        self.win = win
        self.ind = ind
        freq = 10

        tf_node_name = f'tf'
        self.x = 0
        self.y = 0
        self.theta = 0
        self.tf_subscriber = self.create_subscription(TFMessage, tf_node_name, self.tf_listener_callback, freq)

        clicked_node_name = f'led_state_{ind}'
        self.clicked_state = 0
        self.clicked_subscriber = self.create_subscription(Int16, clicked_node_name, self.clicked_listener_callback, freq)

        # self.subscription  # prevent unused variable warning

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

        # rotationX = msg.transforms.transform.rotation.x
        # tf2_msgs.msg.TFMessage(
        #     transforms=[geometry_msgs.msg.TransformStamped(
        #         header=std_msgs.msg.Header(
        #             stamp=builtin_interfaces.msg.Time(
        #                 sec=1639263839,
        #                 nanosec=522887424),
        #                 frame_id='odom_frame'),
        #         child_frame_id='camera_pose_frame',
        #         transform=geometry_msgs.msg.Transform(
        #             translation=geometry_msgs.msg.Vector3(
        #                 x=-3.236397969885729e-05,
        #                 y=3.9944672607816756e-05,
        #                 z=-0.00021711552108172327),
        #             rotation=geometry_msgs.msg.Quaternion(
        #                 x=0.004904894158244133,
        #                 y=-0.7111507654190063,
        #                 z=-0.004923465196043253,
        #                 w=0.7030051946640015)))])

    def clicked_listener_callback(self, msg):
        prev_clicked_state = self.clicked_state

        # print(f"my led state msg: {msg.data}")
        self.clicked_state = msg.data
        if self.clicked_state == 2 and prev_clicked_state != 2:
            'Clicked when LED was lit up'
            curr_time = time.time() - self.win.song_t0
            self.win.update_score(self.ind, curr_time)
        if self.clicked_state == -1:
            # wrong click in inhibition game - reset combo
            self.win.decrease_difficulty()

    def stop(self):
        self.tf_subscriber.destroy()
        self.clicked_subscriber.destroy()
        self.destroy_node()


class Ros2QTSubscriber(QThread):
    camera_msg = QtCore.pyqtSignal(float, float, float)
    clicked_msg = QtCore.pyqtSignal(int)

    def __init__(self, win, ind):
        QThread.__init__(self)
        self.threadactive = False
        self.win = win
        self.ind = ind

        self.rosSubscriber = SubscriberNode(self.win, self.ind)

        self.prev_num_of_clicks = 0
        self.wrong_clicks = 0
        self.dt = 0.05

    def run(self):
       # in a different thread
        self.threadactive = True
        listener_thread = Thread(target=self.run_ros_listener) # listen to tf and click messages
        listener_thread.start() # starts the infinite loop. since we don't call thread.join() we don't wait for execute

        while self.win.running and self.threadactive:
            x = self.rosSubscriber.x
            y = self.rosSubscriber.y
            theta = self.rosSubscriber.theta
            self.camera_msg.emit(x, y, theta)

            curr_num_of_clicks = self.rosSubscriber.clicked_state
            if self.prev_num_of_clicks != curr_num_of_clicks:
                self.prev_num_of_clicks = curr_num_of_clicks
                self.clicked_msg.emit(curr_num_of_clicks - self.wrong_clicks)
            else:
                'Wrong click'
                self.wrong_clicks += 1

            time.sleep(self.dt)

    def run_ros_listener(self):
        while self.threadactive:
            rclpy.spin_once(self.rosSubscriber)
            time.sleep(self.dt)  # 10Hz

        self.rosSubscriber.stop()
        self.quit()

    def stop(self):
        self.threadactive = False


class QT2RosPublisher(QThread):
    '''
    Publish LED commands to ROS topic
    '''

    dt = 0.1  # sec
    LED_msg = QtCore.pyqtSignal(Int16)
    nav_msg = QtCore.pyqtSignal(float, float)  # x, rz

    def __init__(self, ind):
        QThread.__init__(self)
        self.curr_LED_cmd = 0
        self.curr_nav_cmd = 0
        self.publisher = publisherNode(ind)

    def run_led_publisher(self, cmd):
        self.publisher.publish_led_command(cmd)
        print(f"publishing led command: {cmd}")
        self.curr_LED_cmd = cmd

    def run_nav_publisher(self, x, rz):
        # self.curr_nav_cmd = [x, 0.0, 0.0, 0.0, 0.0, rz]
        self.publisher.publish_nav_command(x, 0.0, 0.0, 0.0, 0.0, rz)

    def stop(self):
        self.run_led_publisher(0)
        self.run_nav_publisher(0.0, 0.0)
        self.quit()


# class QT2RosNavPublisher(QThread):
#     '''
#     publish navigation commands in a game-integrated thread
#     '''
#
#     # nav_msg = QtCore.pyqtSignal(float, float, float, float, float, float)
#
#     def __init__(self, win, ind):
#         QThread.__init__(self)
#         self.win = win
#         self.nav_master_node = NavMasterNode(self.win, ind)
#         # self.currCmd = 0
#
#     def run_nav_publisher(self, x, y, z, rx, ry, rz):
#         self.nav_master_node.publish_command(x, 0.0, 0.0, 0.0, 0.0, rz)
#         # self.currCmd = cmd
#
#     def stop(self):
#         self.threadactive = False
#         self.nav_master_node.stop()
#         self.quit()
class publisherNode(Node):
    def __init__(self, ind):
        super().__init__("LEDMasterNode")
        self.led_msg = Int16()
        self.LED_cmd_publisher_ = self.create_publisher(Int16, f"LED_cmd_{ind}", 10)

        self.nav_msg = Twist()
        self.nav_pub = self.create_publisher(Twist, f"cmd_vel_{ind}", 10)

    def publish_led_command(self, led_state_on):
        self.led_msg.data = led_state_on

        self.LED_cmd_publisher_.publish(self.led_msg)

    def publish_nav_command(self, x, y, z, rx, ry, rz):
        self.nav_msg.linear.x = x
        self.nav_msg.angular.z = rz

        self.nav_pub.publish(self.nav_msg)


