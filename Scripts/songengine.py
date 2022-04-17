from mutagen.mp3 import MP3
import numpy as np
import os


class NormalSongEngine:
    def __init__(self, SONGS_PATH, song_name):
        """
        Creates the song's notes (LED commands) and moves (trajectory commands).

        :param song_name:
        :param song_path:
        :param bpm:
        """
        self.song_name = song_name
        self.song_dict = {
            "one_kiss": {"path": os.path.join(SONGS_PATH, "One Kiss_cropped.mp3")},
            "cant_help_falling_in_love": {"path": os.path.join(SONGS_PATH, "Can't Help Falling In Love.mp3")}
                          }
        self.song_path = self.song_dict[song_name]["path"]
        self.audio = MP3(self.song_path)
        self.tot_song_len = self.audio.info.length

        self.note_times = self.get_note_times()

        self.notes = self.get_autoclicker_notes(self.note_times)
        self.moves = self.get_autoclicker_moves(self.note_times)
        self.start_from_second = 0  # start of song
        self.difficulty = None

    def get_note_times(self):
        'Take a list of (phase_start, phase_bpm) and generate time vec'
        if self.song_name == 'one_kiss':
            phase_list = [(0, 123.85)]
        elif self.song_name == 'cant_help_falling_in_love':
            phase_list = [(0.17, 67.1), (65, 67.1), (79, 67.1)]
        else:
            phase_list = [(0, 123)]

        note_times = []
        for i, (phase_start, phase_bpm) in enumerate(phase_list):
            bps = phase_bpm / 60
            dt = 1 / bps  # every this many seconds

            if i+1 == len(phase_list):
                'End of phase list'
                phase_end = self.tot_song_len
            else:
                'end of this phase is the start of the next one'
                phase_end = phase_list[i+1][0]

            phase_times = np.arange(phase_start, phase_end, dt)  # 1.9sec, 3.8sec, etc.
            note_times.append(phase_times.tolist())

        flat_note_list = [item for sublist in note_times for item in sublist]
        return flat_note_list

    def get_autoclicker_notes(self, note_times):
        def silence(len):
            notes = [0 for iteration in range(len)]
            return notes

        def single_beat(reps):
            struct = [1]
            return repeat(struct, reps)

        def on_beat(reps):
            '''
            beat followed by 4 rests
            :param reps:
            :return:
            '''
            struct = [1, 0, 0, 0]
            return repeat(struct, reps)

        def repeat(note_list, reps):
            notes = []
            for _ in range(reps):
                notes += note_list
            return notes

        def get_node_indices(note_list):
            return [ind for ind, el in enumerate(note_list) if el == 1]

        'list of basic intervals in song'
        iters = range(len(note_times))
        notes = []

        'on_beat(k) - hit beat for k intervals'
        if self.song_name == 'one_kiss':
            init_note = 1

            start = silence(init_note)
            ac1_ph1 = start + on_beat(16)
            ac2_ph1 = start + silence(2) + on_beat(16)
            ac3_ph1 = start + silence(4) + on_beat(16)

            ac1_notes = get_node_indices(ac1_ph1)
            ac2_notes = get_node_indices(ac2_ph1)
            ac3_notes = get_node_indices(ac3_ph1)

        elif self.song_name == 'cant_help_falling_in_love':
            # iterations to play on, assuming each iteration is the shortest beat
            # first phase is 9 + 8*8 = 73 beats long
            'Phase 1'
            ph1_len = 8
            start = silence(9)
            ph1_struct = on_beat(1) + silence(4)

            ac1_ph1 = start + repeat(ph1_struct, ph1_len)
            ac2_ph1 = start + silence(2) + repeat(ph1_struct, ph1_len)
            ac3_ph1 = start + silence(4) + repeat(ph1_struct, ph1_len)

            'Phase 2'
            start_2 = silence(1)

            ac1_ph2 = start_2 + on_beat(5)
            ac2_ph2 = start_2 + silence(4) + on_beat(1) + silence(3) + single_beat(1)
            ac3_ph2 = start_2 + silence(8) + on_beat(1) + silence(3) + single_beat(1)

            'Phase 3'
            ph3_len = 8
            start_3 = silence(1)

            ac1_ph3 = start_3 + repeat(ph1_struct, ph3_len)
            ac2_ph3 = start_3 + silence(2) + repeat(ph1_struct, ph3_len)
            ac3_ph3 = start_3 + silence(4) + repeat(ph1_struct, ph3_len)

            'Construct full note list'
            ac1_notes = get_node_indices(ac1_ph1 + ac1_ph2 + ac1_ph3)
            ac2_notes = get_node_indices(ac2_ph1 + ac2_ph2 + ac2_ph3)
            ac3_notes = get_node_indices(ac3_ph1 + ac3_ph2 + ac3_ph3)
        else:
            print("no such song!")
            tmp = on_beat(1) + silence(4)
            ac1_notes = get_node_indices(silence(2) + tmp)
            ac2_notes = get_node_indices(silence(1) + tmp)
            ac3_notes = get_node_indices(tmp)

        notes.append(ac1_notes)
        notes.append(ac2_notes)
        notes.append(ac3_notes)

        return notes

    def get_autoclicker_moves(self, note_times):
        turn_speed = 0.92
        march_speed = 0.06
        def cmd(command, reps):
            if reps == 0:
                raise Exception("Didn't request valid length of repeat")

            out = []
            for _ in range(reps):
                out += [command]
            return out

        def stop(reps):
            possible_cmds_x = cmd(0.0, reps)
            possible_cmds_rz = cmd(0.0, reps)

            return repeat(possible_cmds_x, possible_cmds_rz, 1)

        def rotate(degs, reps):
            x_cmds = [0.0]
            rz_cmds = [degs]
            return repeat(x_cmds, rz_cmds, reps)

        def march(dist, reps):
            x_cmds = [dist]
            rz_cmds = [0.0]
            return repeat(x_cmds, rz_cmds, reps)

        def half_line_forward():
            tmp = march(-march_speed, 4) + rotate(-turn_speed, 4)
            tmp_dec = decipher(tmp)
            return repeat(tmp_dec[0], tmp_dec[1], 1)

        def half_line_back():
            tmp = march(-march_speed, 4) + rotate(turn_speed, 4)
            tmp_dec = decipher(tmp)
            return repeat(tmp_dec[0], tmp_dec[1], 1)
        def line(reps):
            tmp = march(-march_speed, 4) + rotate(-turn_speed, 4) + stop(4) + march(-march_speed, 4) + rotate(turn_speed, 4) + stop(4)
            tmp_dec = decipher(tmp)
            return repeat(tmp_dec[0], tmp_dec[1], reps)

        def half_circle(reps):
            x_cmds = [-0.7*march_speed]
            rz_cmds = [-0.14*turn_speed]
            return repeat(x_cmds, rz_cmds, reps)

        def repeat(x_cmds, rz_cmds, reps):
            out_x_cmds = []
            out_rz_cmds = []

            for _ in range(reps):
                out_x_cmds += x_cmds
                out_rz_cmds += rz_cmds
            return [out_x_cmds, out_rz_cmds]

        def decipher(cmds):
            x_cmds = []
            rz_cmds = []
            for i, cmd in enumerate(cmds):
                if i % 2 == 0:
                    'x command'
                    x_cmds += cmd
                else:
                    rz_cmds += cmd

            return x_cmds, rz_cmds
        'list of basic intervals in song'
        iters = range(len(note_times))
        moves = [] # both for x and rz

        'repeat returns a list of '
        if self.song_name == 'one_kiss':
            start = stop(1)

            ac1_ph1 = start + line(4)
            ac2_ph1 = start + line(4)
            ac3_ph1 = start + line(4)

            ac1_moves = decipher(ac1_ph1)
            ac2_moves = decipher(ac2_ph1)
            ac3_moves = decipher(ac3_ph1)
        elif self.song_name == 'cant_help_falling_in_love':
            # iterations to play on, assuming each iteration is the shortest beat
            # stop returns [ [x_cmds], [rz_cmds] ]
            # line returns [ [x_cmds], [rz_cmds] ]
            # so i can keep adding stops and lines as long as I decipher at the end
            start = stop(9)
            ph1_struct = decipher(half_line_forward() + stop(8) + half_line_back() + stop(8))  # turn it into a list of 2 elements - x command and rz command
            # each struct is 32 beats long
            # first phase is 73 beats long
            ac1_ph1 = start + repeat(ph1_struct[0], ph1_struct[1], 2)
            ac2_ph1 = start + repeat(ph1_struct[0], ph1_struct[1], 23)
            ac3_ph1 = start + repeat(ph1_struct[0], ph1_struct[1], 23)

            start_ph2 = stop(1)
            ac1_ph2 = start_ph2 + half_circle(16) + stop(2)

            ac1_moves = decipher(ac1_ph1 + ac1_ph2 + ac1_ph1)
            ac2_moves = decipher(ac2_ph1 + ac1_ph2 + ac1_ph1)
            ac3_moves = decipher(ac3_ph1 + ac1_ph2 + ac1_ph1)
        else:
            print("no such song!")
            start = stop(1)

            ac1_ph1 = start + line(4)
            ac2_ph1 = start + line(4)
            ac3_ph1 = start + line(4)

            ac1_moves = decipher(ac1_ph1)
            ac2_moves = decipher(ac2_ph1)
            ac3_moves = decipher(ac3_ph1)

        moves.append(ac1_moves)
        moves.append(ac2_moves)
        moves.append(ac3_moves)

        return moves

    def set_difficulty(self, dif):
        pass


class ComboSongEngine:
    def __init__(self, SONGS_PATH, song_name):
        self.song_name = song_name
        self.song_dict = {
            "upbeat_trans_music": {"path": os.path.join(SONGS_PATH, "Upbeat trans music.mp3")}
        }
        self.song_path = self.song_dict[song_name]["path"]
        self.audio = MP3(self.song_path)
        self.tot_song_len = self.audio.info.length
        self.difficulty = 1
        self.max_combo = 0
        self.difficulty_dict = {
            1: {"avg_TOff": 5, "prob_green": 1},
            2: {"avg_TOff": 4, "prob_green": 1},
            3: {"avg_TOff": 3.5, "prob_green": 0.9},
            4: {"avg_TOff": 3.5, "prob_green": 0.85},
            5: {"avg_TOff": 3, "prob_green": 0.85},
            6: {"avg_TOff": 2.5, "prob_green": 0.8},
            8: {"avg_TOff": 2.5, "prob_green": 0.7},
            9: {"avg_TOff": 2, "prob_green": 0.7},
            10: {"avg_TOff": 1.8, "prob_green": 0.7},
        }
        self.steps_per_lvl = 10
        self.start_from_second = 0

    def generate_commands(self, difficulty=-1):
        if difficulty == -1:
            difficulty = self.difficulty

        self.set_difficulty(difficulty)

        notes = self.generate_notes()
        moves = self.generate_moves()
        return notes, moves

    def set_difficulty(self, dif):
        self.difficulty = dif

    def generate_notes(self):
        dif_dict = self.difficulty_dict[self.difficulty]
        avg_TOff = dif_dict["avg_TOff"]
        p = dif_dict["prob_green"]

        'Generate notes'
        variance = 0.2

        signs = np.random.choice([1, -1], self.steps_per_lvl, p=[p, 1 - p])  # choose color - green or red
        preset_on_time = 2
        TOn = preset_on_time*np.ones(self.steps_per_lvl)
        TOff = np.random.uniform(avg_TOff-variance, avg_TOff + variance, self.steps_per_lvl)
        notes = zip(signs, TOn, TOff)

        return notes

    def generate_moves(self):
        moves = []
        return moves



