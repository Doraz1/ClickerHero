from PyQt5 import QtCore
from PyQt5.QtCore import QThread
import time
import numpy as np
import pyautogui
from PyQt5.QtGui import QRegion

class ClickerBlinkThread(QThread):
    def __init__(self, win, clicker):
        QThread.__init__(self)
        self.win = win
        self.clicker = clicker
        self.threadactive = False

    def run(self):
        try:
            self.threadactive = True
            self.clicker.activateLEDs()
            self.stop()
        except Exception as e:
            print(e)
            exit(2)


    def stop(self):
        self.threadactive = False
        # self.clicker.resetBlink()
        self.quit()

class ResetGuiThread(QThread):
    def __init__(self, win):
        QThread.__init__(self)
        self.win = win
        self.threadactive = False

    def run(self):
        try:
            print("entered gui reset thread")
            self.threadactive = True
            self.win.killAllThreads()
            for clicker in self.win.screens["game"].autoClickerAnimations:
                clicker.resetBlink()

            self.stop()
        except Exception as e:
            print(e)
            exit(3)


    def stop(self):
        self.threadactive = False
        self.quit()


class ACBlinkThread(QThread):
    easierLevel = 8

    def __init__(self, win, clicker_number, bpm, notes):
        QThread.__init__(self)
        self.win = win
        self.clicker = win.screens['game'].autoClickerAnimations[clicker_number - 1]
        self.bpm = bpm
        self.notes = notes
        self.blinkThread = ClickerBlinkThread(win, self.clicker)

        self.threadactive = False

    def run(self):
        try:
            self.threadactive = True

            start_time = time.time()
            iteration = 1
            note_ind = 0

            while self.win.running and self.threadactive:
                time_to_sleep = start_time - time.time() + self.win.note_times[iteration]
                time.sleep(time_to_sleep)
                next_note = self.notes[note_ind]
                if next_note == iteration:
                    prev_note = next_note
                    note_ind += 1
                    next_note = self.notes[note_ind]
                    self.blinkThread.start()
                iteration += 1
        except Exception as e:
            print(e)
            exit(4)


    def stop(self):
        self.threadactive = False
        self.quit()

class ACMoveThread(QThread):
    clicker_pos = QtCore.pyqtSignal(int, int, int)

    def __init__(self, win):
        QThread.__init__(self)
        self.win = win
        clicker1Anim = win.screens['game'].autoClickerAnimations[0]
        clicker2Anim = win.screens['game'].autoClickerAnimations[1]
        clicker3Anim = win.screens['game'].autoClickerAnimations[2]

        self.win = win
        self.clickers = [clicker1Anim, clicker2Anim, clicker3Anim]

        # Initialize constants
        self.noise = 100
        self.dt = 0.05
        self.screen_w, self.screen_h = pyautogui.size()
        self.clicker_radius = clicker1Anim.autoclicker.radius
        self.min_y, self.max_y = 510, self.screen_h
        self.min_x, self.max_x = self.clicker_radius, self.screen_w
        self.min_x, self.max_x = self.clicker_radius, 1300
        self.curr_x = []
        self.curr_y = []
        self.curr_ax = []
        self.curr_ay = []

        self.threadactive = False


    def run(self):
        try:
            self.threadactive = True
            if self.win.simulatorActive:
                self.move_ants_simulator()
            else:
                self.move_based_on_inputs()

        except Exception as e:
            print(e)
            exit(4)

    def move_ants_simulator(self):
        def in_bounds_x(a_x):
            touched_wall = False
            curr_x = self.curr_x[i]
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
            curr_y = self.curr_y[i]
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
            print(f"moving Clicker {i} which is in x={new_x}")
            for j in range(len(self.clickers)):
                if i != j:
                    # print(f"checking collision with {self.curr_x[j]}")
                    if ((new_x - self.curr_x[j])**2) + ((new_y - self.curr_y[j])**2) <= (self.clicker_radius ** 2):
                        collided_clicker = True
                        print(f"clicker {i} collided when moving into {j}")
            return new_x, new_y, collided_wall_x, collided_wall_y, collided_clicker

        for ac in self.clickers:
            geo = ac.geometry()
            x0, y0 = geo.x(), geo.y()
            self.curr_x.append(x0)
            self.curr_y.append(y0)
            self.curr_ax.append(0)
            self.curr_ay.append(0)

        while self.win.running:
            if not self.win.paused:
                for i, ac in enumerate(self.clickers):
                    rand = np.random.randint(-self.noise, self.noise, 2)
                    rand = [30, 20]

                    new_ax = self.curr_ax[i] + rand[0]
                    new_ay = self.curr_ay[i] + rand[1]

                    new_x, new_y, col_x, col_y, col_clicker = sense_collision(new_ax, new_ay, i)

                    if col_x:
                        new_ax = 0
                    if col_y:
                        new_ay = 0
                    if col_clicker:
                        new_ax = 0
                        new_ay = 0
                    else:
                        'No collision'
                        self.curr_ax[i] = new_ax
                        self.curr_x[i] = new_x
                        self.curr_ay[i] = new_ay
                        self.curr_y[i] = new_y

                    self.clicker_pos.emit(self.curr_x[i], self.curr_y[i], i)


                print(f"dx12={np.abs(self.curr_x[0]-self.curr_x[1])}")
                print(f"dx23={np.abs(self.curr_x[1]-self.curr_x[2])}")
                print(f"dx13={np.abs(self.curr_x[2]-self.curr_x[0])}")

                time.sleep(self.dt)

    def move_based_on_inputs(self, x, y, theta):
        center_screen_bias_x = 900
        center_screen_bias_y = 750
        MEF = 6 # make camera movements larger
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
        while self.win.running:
            if self.music.get_pos() == -1:
                # song finished running
                self.btn_stop_game()
                return

            curr_progress = self.music.get_pos() / 1000  # in seconds
            progress_percentage = curr_progress / self.win.total_song_length * 100
            self.progress.emit(int(progress_percentage))

            time.sleep(self.dt)


    def stop(self):
        self.threadactive = False
        self.quit()

#region ROS2

import rclpy
from rclpy.node import Node
from threading import Thread
from  tf2_msgs.msg import TFMessage
import math
import rospy2

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.freq = 10
        self.subscription = self.create_subscription(TFMessage,'tf',self.listener_callback, self.freq)
        self.msg = 15
        self.x = 0
        self.y = 0
        self.theta = 0
        print("entered")
        # self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
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



class Ros2QTSubscriber(QThread):
    dt = 0.2  # sec
    camera_msg = QtCore.pyqtSignal(float, float, float)

    def __init__(self, win):
        QThread.__init__(self)
        self.threadactive = False
        self.win = win
        # rclpy.init()
        self.minimal_subscriber = MinimalSubscriber()

    def run_ros_listener(self):
        dt = 1 / self.minimal_subscriber.freq
        while self.threadactive:
            rclpy.spin_once(self.minimal_subscriber)
            time.sleep(dt)  # 10Hz

        self.minimal_subscriber.destroy_node()

    def run(self):
       # in a different thread
        self.threadactive = True
        listener_thread = Thread(target=self.run_ros_listener)
        listener_thread.start() # starts the infinite loop. since we don't call thread.join() we don't wait for execute


        while self.win.running and self.threadactive:
            x = self.minimal_subscriber.x
            y = self.minimal_subscriber.y
            theta = self.minimal_subscriber.theta
            self.camera_msg.emit(x, y, theta)
            time.sleep(self.dt)


    def stop(self):
        self.threadactive = False
        self.quit()
#endregion