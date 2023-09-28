import os
import pyautogui
from Scripts.enums import Songs, Games

def get_params():
    params = {
    # Window parameters
    "window_title": "Clicker Hero",
    "sim_active": False,

    # Data parameters
    # "use_pickled_song_note_data": True,
    "use_pickled_song_note_data": False,

    # "use_pickled_song_move_data": True,
    "use_pickled_song_move_data": False,

    # Robot parameters
    # "num_robots": 1,
    "num_robots": 2,
    "robot_linear_speed": -0.06,  # forward
    "robot_turn_speed": -0.9,  # clockwise
    # "communication_delay": 10,  # centisec
    "communication_delay": 0,  # ms
    "robot_initial_reconnect_dt": 0,  # s
    "robot_reconnect_dt": 10,  # s

    "livelihood_thresh": 4,  # iterations before declaring communication error

    # Game parameters
    "fadeout_music_time": 1000,
    "check_communication": False,
    # "check_communication": True,

    # King of the Bongos
    # "move_robots": False,
    "move_robots": True,
    "KB_start_from_sec": 0.0,  # ms
    "KB_move_OL": True,
    # "KB_move_OL": False,
    "KB_woohoo_RT_thresh": 1.5,  # sec
    "KB_success_grace": 3,
    "KB_min_diff": 1,
    "KB_max_diff": 3,

    # Traffic Light
    "TL_max_score": 40,
    "TL_variance": 0.5,
    "TL_TOn": 3,
    "TL_fail_grace": 1,  # boo every this many
    "TL_success_grace": 2,  # woohoo every this many
    "TL_diff_incr_thresh": 4,
    "TL_dt_btween_cols": 0.5,
    "TL_min_diff": 1,
    "TL_max_diff": 7,
    "TL_gameover_score": 5,
    "TL_gameover_time": 150,

    # Simon Says
    "SS_host_index": 0,
    "TOn_host": 4,  # must match lightup time on physical robot?
    "TOff_host": 4,
    "T_change_phase": 3,
    "TOn_user": 11,  # This must be longer tan the on time of the '9' difficulty of robot
    "TOff_user": 1,  # 3?
    "SS_fail_grace": 0,
    "SS_min_diff": 1,
    "SS_max_diff": 4,
    "SS_time_between_color_clicks": 1.5,
    "SS_gameover_time": 150,

    # Database
    "user_reach": 60,  # cm radius

    # ROS
    "subscriber_frequency": 10,
    "publisher_frequency": 20,
    "dt_clicks": 2,  # minimal time between successive clicks


    # Files
    "king_of_the_bongos_notes_file": "data_files/King_of_the_Bongos_notes.pkl",
    "king_of_the_bongos_moves_file": "data_files/King_of_the_Bongos_moves.pkl",

    # Alarms
    "battery_alarm_thresh": 10,

    #debugs
    # "debug_movement": False,
    "debug_movement": True,
    # "debug_initialization": False,
    "debug_initialization": True,
    # "debug_database": False,
    "debug_database": True,

    }
    # Paths
    # params["ROOT_PATH"] = os.path.dirname(os.path.abspath(__file__))

    params["ROOT_PATH"] = os.getcwd()
    params["ASSET_PATH"] = os.path.join(params["ROOT_PATH"], 'Assets')
    params["SCRIPTS_PATH"] = os.path.join(params["ROOT_PATH"], 'Scripts')
    params["SONGS_PATH"] = os.path.join(params["ROOT_PATH"], "Songs")
    params["HEBREW_SONGS_PATH"] = os.path.join(params["SONGS_PATH"], "Hebrew_songs")
    params["ADDITIONAL_AUDIO_PATH"] = os.path.join(params["SONGS_PATH"], "Additional_audio")

    "Tutorials"
    params[f"{Games.KING_OF_THE_BONGOS.value} tutorial path"] = os.path.join(params["ASSET_PATH"], 'Videos/KB_tutorial.mp4')
    params[f"{Games.TRAFFIC_LIGHT.value} tutorial path"] = os.path.join(params["ASSET_PATH"], 'Videos/TL_tutorial.mp4')
    params[f"{Games.SIMON_SAYS.value} tutorial path"] = os.path.join(params["ASSET_PATH"], 'Videos/SS_tutorial.mp4')

    fullscreen = True
    if fullscreen:
        size = pyautogui.size()
        params["window_width"] = size[0]
        params["window_height"] = size[1]
    else:
        params["window_width"] = 1600
        params["window_height"] = 900

    return params
