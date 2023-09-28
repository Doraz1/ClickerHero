
import matplotlib
import sys

from Scripts.database import PlayerDataBase
from matplotlib import pyplot as plt
from Scripts.enums import Songs
import time
import numpy as np

songs_list = [
        Songs.KOL_HAKAVOD.value,
        Songs.PGISHA_BAMILUIM.value,
        Songs.KARNAVAL_BANACHAL.value,
        Songs.TRAFFIC_LIGHT_MUSIC.value,
        Songs.SIMON_SAYS_MUSIC.value,
    ]

def plot_song_data(player_data, song_data):
    def plot_TL():
        if song_dynamic_vals == '0.0':
            'Uninitialized'
            song_vals = [(0.0, 0.0, 0.0, 1, -1)]
        else:
            song_vals = db.str_to_list(song_dynamic_vals)

        green_cts = []
        green_vals = []
        green_rts = []

        red_cts = []
        red_vals = []

        gray_cts = []
        gray_vals = []
        for ct, val in enumerate(song_vals):
            time, rt, typeOfData, difficulty, robo_ind = val
            # print(f"time: {time} | Dif: {difficulty}")
            'Missed reds = 2'
            'Clicked greens = 1'
            'Clicked off state = 0'
            'Missed reds = -1'
            'Clicked reds = -2'
            if abs(typeOfData) == 1:
                green_cts.append(time)
                green_vals.append(typeOfData)
                green_rts.append(rt)
            elif abs(typeOfData) == 2:
                red_cts.append(time)
                red_vals.append(typeOfData)
            elif abs(typeOfData) == 0:
                gray_cts.append(time)
                gray_vals.append(typeOfData)

        green_avg_rt = np.mean(np.array(green_rts))
        if green_vals:
            ax.stem(green_cts, green_vals, markerfmt="Dg", basefmt='blue',
                    label=f"avg RT: {round(green_avg_rt, 1)} sec")
        if red_vals:
            ax.stem(red_cts, red_vals, markerfmt="Xr", basefmt='blue')
        if gray_vals:
            ax.stem(gray_cts, gray_vals, markerfmt="Pg", basefmt='blue')

        ax.set(ylim=(-2.5, 2.5), yticks=np.arange(-2, 3))
        ax.set_xlabel('Time[sec]')
        ax.legend()

    def plot_SS():
        if song_dynamic_vals == '0.0':
            'Uninitialized'
            song_vals = [(0, 0, 0, [0], False, 0, -1, -1)]
        else:
            song_vals = db.str_to_list(song_dynamic_vals)

        num_to_color_dict = {
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

        song_vals.sort(key=lambda x: x[0])
        reaction_times = []

        for ind, vals in enumerate(song_vals):
            t, true_col_num, clicked_col_num, seq, is_last, iter, reaction_time, robo_ind = vals
            true_col = num_to_color_dict[true_col_num]
            if clicked_col_num != -1:
                clicked_col = num_to_color_dict[clicked_col_num]
            else:
                'didnt click'
                clicked_col = 'black'
            reaction_times.append(reaction_time)

            'Plotting parameters'
            rx = 0.2
            ry = 6 * rx
            dy = 1.1 * ry

            y_val = reaction_time

            def get_loc_in_seq(seq_length, iteration):
                seq_middle = (seq_length - 1) / 2
                steps_from_middle = iteration - seq_middle
                curr_x = ind + steps_from_middle * rx
                return curr_x

            L = len(seq)
            'Plot sequence'
            for k, color_num in enumerate(seq):
                col = num_to_color_dict[color_num]
                curr_x = get_loc_in_seq(L, k)

                if col != "Off":
                    circle = matplotlib.patches.Ellipse(xy=(curr_x, y_val + dy), width=rx,
                                                        height=ry,
                                                        edgecolor=col, fc=col, lw=2)
                    ax.add_patch(circle)
            'Plot current clicked'
            if clicked_col != "Off":
                curr_x = get_loc_in_seq(L, iter)

                CC_circle = matplotlib.patches.Ellipse(xy=(curr_x, y_val), width=rx,
                                                       height=ry,
                                                       edgecolor=clicked_col, fc=clicked_col, lw=2)
                ax.add_patch(CC_circle)

        avg_reaction_time = np.mean(np.array(reaction_times))

        ax.set_facecolor("gray")
        ax.set_xlabel('iteration')
        ax.set_ylabel(f'RT [sec]\navg={round(avg_reaction_time, 1)}')
        ax.set_ylim(-2, 7)
        ax.set_xlim(-1, 13)

    def plot_KB():
        'King of the Bongos'
        if song_dynamic_vals == '0.0' or song_dynamic_vals == "":
            'Uninitialized'
            song_vals = []
        else:
            song_vals = db.str_to_list(song_dynamic_vals)

        song_vals.sort(key=lambda x: x[0])

        hit_t_list = []
        hit_rt_list = []
        wrong_hit_list = []
        wrong_hit_rt_list = []
        missed_t_list = []
        missed_rt_list = []

        for ct, val in enumerate(song_vals):
            t, type, num_notes_played, hitNote, robo_ind = val  # type is -1 if missed, 1 if hit and 0 if hit robot thats turned off

            if type == -1:
                'missed note'
                missed_t_list.append(t)
                missed_rt_list.append(0)
            elif type == 0:
                wrong_hit_list.append(t)
                wrong_hit_rt_list.append(0)
            elif type == 1:
                hit_t_list.append(t)
                hit_rt_list.append(hitNote)

        if hit_t_list:
            ax.stem(hit_t_list, hit_rt_list, markerfmt="Dg", basefmt='blue')
        if wrong_hit_list:
            ax.stem(wrong_hit_list, wrong_hit_rt_list, markerfmt="ob", basefmt='blue')
        if missed_t_list:
            ax.stem(missed_t_list, missed_rt_list, markerfmt="Xr", basefmt='blue')

        ax.set_xlabel('Time[sec]')
        ax.set_ylabel('RT\n [sec]')
        ax.set_ylim(-1, 180)
        ax.set_ylim(-1, 4)


    firstName, lastName, gender, age, difficulty = player_data
    fig, axs = plt.subplots(len(songs_list), N, figsize=(16, 8))
    plt.suptitle(f"{firstName} {lastName} [{gender}], age={age}")

    for j in range(N):
        for i, datum in enumerate(song_data):
            ax = axs[i][j]
            song_name = datum[0]
            full_title = song_name + f" {j+1}"
            ax.set_title(full_title)

            'Get song data'
            song_score, song_dynamic_vals = datum[1 + N*j: 1+N*j+2]
            print(f"Plotting {song_dynamic_vals} for {song_name} on axis {j}{i}")
            if song_name == Songs.TRAFFIC_LIGHT_MUSIC.value:
                plot_TL()
            elif song_name == Songs.SIMON_SAYS_MUSIC.value:
                plot_SS()
            else:
                plot_KB()

    # fig.tight_layout(pad=3)
    fig.tight_layout()
    plt.show()

    return

def visualize_player_info(row):
    player_data = row[:5]

    song_data = []
    for j, song in enumerate(songs_list):
            start_ind = 5 + j * 4
            score1, dynamic_vals1, score2, dynamic_vals2 = row[start_ind:start_ind+2*N]
            song_data.append([song, score1, dynamic_vals1, score2, dynamic_vals2])

    plot_song_data(player_data, song_data)


path = r"/home/clickerhero/Game/data_files/playersDB.db"
print(f"DB path: {path}")
db = PlayerDataBase(path=path)
N = db.num_RT_lists_saved

rows = db.load_all()
for row in rows:
    print(row)
    visualize_player_info(row)