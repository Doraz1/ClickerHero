import os
import sqlite3 as sql
import sys
import json

import numpy as np
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
)
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QWidget, QPushButton, QVBoxLayout, QInputDialog, QLineEdit, QListWidget, QListWidgetItem
from PyQt5.QtGui import QFont
from Scripts.enums import Songs


class PlayerDataBase:
    topic = 'players'
    database_name = f'data_files/{topic}DB.db'
    list_to_str_char = "|"
    multiple_inputs_per_data_item_char = "_"

    def __init__(self, path=""):
        if path:
            self.database_name=path

        self.firstName = ""
        self.lastName = ""
        self.age = -1
        self.gender = ""
        self.scores = []
        self.difficulty = 1
        self.playerLoaded = False
        self.num_RT_lists_saved = 2
        round_err_digits = 3
        self.rounding_coeff = 10 ** round_err_digits

        self.songs_list = [
            Songs.KOL_HAKAVOD.value,
            Songs.PGISHA_BAMILUIM.value,
            Songs.KARNAVAL_BANACHAL.value,
            Songs.TRAFFIC_LIGHT_MUSIC.value,
            Songs.SIMON_SAYS_MUSIC.value,
                          ]

        self.sql_keys = ['first_name TEXT',
                    'last_name TEXT',
                    'gender TEXT',
                    'age INTEGER',
                    'KB_difficulty INTEGER']

        for song in self.songs_list:
            for i in range(self.num_RT_lists_saved):
                self.sql_keys.append(f"{song}_score_{i} FLOAT")
                self.sql_keys.append(f"{song}_reaction_times_{i} TEXT")

        self.sql_dict = {i: key for i,key in enumerate(self.sql_keys)}
        self.sql_key_names = [key.split(' ')[0] for key in self.sql_keys]

        query = f"CREATE TABLE IF NOT EXISTS {self.topic} ("
        for i in range(len(self.sql_dict)):
            if i == len(self.sql_dict)-1:
                query += self.sql_dict[i] + ")"
            else:
                query += self.sql_dict[i] + ", "

        self.__execute_query(query)

    def load_all(self):
        query = f"""SELECT * FROM {self.topic}"""
        rows = self.__execute_query(query, return_rows=True)

        return rows

    def get_player_row(self, first_name, last_name):
        query = f"""select * from {self.topic}
                            where first_name = '{first_name}' and last_name = '{last_name}'"""
        row = self.__execute_query(query, return_rows=True)
        return row

    def get_ind_of_key(self, key):
        for ind, k in enumerate(self.sql_key_names):
            if key.lower() == k.lower():
                return ind
        return None
    def load(self, first_name, last_name):

        rows = self.get_player_row(first_name, last_name)

        if rows:
            user = rows[0] # get first user found
            firstNameInd = self.get_ind_of_key('first_name')
            self.firstName = user[0]
            self.lastName = user[1]
            self.gender = user[2]
            self.age = user[3]
            self.difficulty = user[4]

            self.scores = []
            for song_score in user[5:]:
                self.scores.append(song_score)
            print(f"Loaded play profile of {self.firstName} {self.lastName}")
            self.playerLoaded = True
        else:
            # empty list
            self.firstName = ""
            self.lastName = ""
            self.age = -1
            self.gender = ""
            self.scores = []
            self.playerLoaded = False

    def insert(self, firstName, lastName, gender, age):
        self.load(firstName, lastName)
        if self.firstName != "":
            # player exists
            print("---------------------------------\n"
                  "Failed to insert new player:\n"
                  "Player already exists in database!\n"
                  "---------------------------------")
        else:
            print("---------------------------------\n"
                  "Creating new player!\n"
                  "---------------------------------")
            query = f"""INSERT INTO  {self.topic}  VALUES
                        ('{firstName}', '{lastName}', '{gender}', {age}, 1, """
            num_entries = 2*self.num_RT_lists_saved*len(self.songs_list)
            for i in range(num_entries):  # each song has N scores and reaction time lists
                if i != num_entries - 1:
                    query += "0.0, "
                else:
                    query += "0.0)"

            self.__execute_query(query)

    def remove(self, first_name, last_name):
        query = f"""delete from {self.topic}
                    where first_name = '{first_name}' and last_name = '{last_name}'"""

        self.__execute_query(query)
    def get_kb_scores(self):
        out = []
        for song in self.songs_list:
            scores_list = self.get_all_vals_of_field(song, self.firstName, self.lastName, field='score')
            out.append(max(scores_list))
        return out
    def update_db_score_and_RT(self, song_name, new_score, reaction_time=[], convert=True):
        first_name = self.firstName
        last_name = self.lastName

        # if song name contains number, update this iteration number
        ind = None
        for s in song_name:
            if s.isdigit():
                song_name = song_name.split('_score')[0]
                ind = int(s)

        self.update_score(song_name, first_name, last_name, new_score, ind=ind)
        self.update_reaction_time(song_name, first_name, last_name, reaction_time, convert, ind=ind)

    def get_all_vals_of_field(self, song_name, first_name, last_name, field='score'):
        list_of_vals = []
        for n in range(self.num_RT_lists_saved):
            query = f"""SELECT {song_name}_{field}_{n} FROM {self.topic}
                                   WHERE first_name = '{first_name}' AND last_name = '{last_name}'"""

            val = self.__execute_query(query, return_rows=True)[0][0]
            list_of_vals.append(val)

        return list_of_vals

    def update_score(self, song_name, first_name, last_name, new_score, ind=None):
        'Get current scores'
        scores_list = self.get_all_vals_of_field(song_name, first_name, last_name, field='score')
        # scores_list_float = [self.str_to_list(el) for el in scores_list]

        if ind is None:
            'Check for the first empty list'
            for ind, el in enumerate(scores_list):
                if el == 0.0:
                    'Non-updated'
                    break

        curr_max = max(scores_list)
        if float(curr_max) < float(new_score):
            print("---------------------------------\n"
                  "Congratulations! New record score!\n"
                  f"Player high score: {new_score} exceeds current performance: {curr_max}!\n"
                  "---------------------------------")

        else:
            print("\n---------------------------------\n"
                  f"Didn't beat top score of {curr_max} with {new_score}. Better luck next time!\n"
                  "---------------------------------")

        'Update score in database'
        query = f"""Update {self.topic}
                                    set {song_name}_score_{ind} = {new_score}
                                    where first_name = '{first_name}' and last_name = '{last_name}'"""
        self.__execute_query(query)

        'update scores in main window'
        query = f"""select * from {self.topic}
                                        where first_name = '{first_name}' and last_name = '{last_name}'"""
        rows = self.__execute_query(query, return_rows=True)
        if rows:
            'first user found'
            user = rows[0]
            self.scores = []
            for song_score in user[3:]:
                self.scores.append(song_score)

    def update_reaction_time(self, song_name, first_name, last_name, reaction_time_list, convert, ind=None):
        'Get current reaction times'
        RTs_list = self.get_all_vals_of_field(song_name, first_name, last_name, field='reaction_times')
        RT_list = [self.str_to_list(el) if el else -1 for el in RTs_list]

        if ind is None:
            'Check for the first empty list '
            for ind, el in enumerate(RT_list):
                if el == -1 or el == 0.0 or el == [0.0] or el == [(0.0,)]:
                    'Non-updated'
                    break

        'Write RT list to empty list / last if none are empty'
        if convert:
            reaction_times = self.list_to_str(reaction_time_list)
        else:
            reaction_times = reaction_time_list
        print(f"gonna insert RT: {reaction_times}")
        query = f"""Update {self.topic}
                                    set {song_name}_reaction_times_{ind} = '{reaction_times}'
                                    where first_name = '{first_name}' and last_name = '{last_name}'"""
        self.__execute_query(query)

    def str_to_list(self, str):
        split_string = str.split(self.list_to_str_char)
        res = []
        for datum in split_string:
            datum_list = datum.split(self.multiple_inputs_per_data_item_char)
            tmp = []
            for el in datum_list:
                if el == "False":
                    tmp.append(False)
                elif el == "True":
                    tmp.append(True)
                elif el and el[0] == '[':
                    'list'
                    json_list = json.loads(el)
                    tmp.append(json_list)
                else:
                    try:
                        tmp.append(float(el) / self.rounding_coeff)
                    except Exception as e:
                        print(f"Coulnd't append resulting float from db string: {e}")
            res.append(tuple(tmp))
        return res

    def list_to_str(self, lst):
        'Could be a list of floats or a list of sublists/tuples'
        res = ""

        L = len(lst)
        try:
            sub_L = len(lst[0])

            for i, el in enumerate(lst):
                for j, sub_el in enumerate(el):
                    if type(sub_el) == float or type(sub_el) == int:
                        res += f"{int(sub_el*self.rounding_coeff)}"
                    elif type(sub_el) == bool:
                        res += f"{sub_el}"
                    elif isinstance(sub_el, list):
                        res += f"{sub_el}"

                    if j != sub_L - 1:
                        res += self.multiple_inputs_per_data_item_char

                'differentiate between different inputs'
                if i != L - 1:
                    res += self.list_to_str_char


            print(f"db converted {lst}\n to {res}")
        except Exception as e:
            print(f"List {lst} has no element 0: {e}")
        return res

    def update_parameter(self, param, new_val):
        'Update parameter in database'
        query = f"""Update {self.topic}
                    set {param} = {new_val}
                    where first_name = '{self.firstName}' and last_name = '{self.lastName}'"""
        print(f"Updated {param} to {new_val}")

        self.__execute_query(query)

    def get_parameter(self, keyName):
        'Update parameter in database'
        query = f"""select * from {self.topic}
                            where first_name = '{self.firstName}' and last_name = '{self.lastName}'"""
        rows = self.__execute_query(query, return_rows=True)

        if rows:
            user = rows[0]  # get first user found
            param = user[self.get_ind_of_key(keyName)]
            return param
        else:
            return None

    def __execute_query(self, query, return_rows = False):
        self.connection = sql.connect(self.database_name)
        self.cursor = self.connection.cursor()
        try:
            self.cursor.execute(query)
            self.connection.commit()
            rows = self.cursor.fetchall()

            self.connection.close()
            if return_rows:
                return rows
        except Exception as e:
            print(f"Exception in SQL: {e}")

    def delete_whole_database(self):
        query = f"""delete from {self.topic}"""
        self.__execute_query(query)


class ListGenerator(QWidget):

    font = QFont('Times', 45, QFont.Bold)

    def __init__(self, win, db):
        super(ListGenerator, self).__init__()
        self.win = win
        self.params_dict = {"KB_difficulty": {i for i in range(1, self.win.params["KB_max_diff"]+1)},"saver": {0, 1}}
        self.db = db_facade(self, db)
        self.setFont(self.font)
        self.setGeometry(1000, 500, 600, 300)
        self.move(700, 400)
        self.layout = None
        self.listWidget = None

    def make_list(self, title):
        self.layout = QVBoxLayout()

        self.setWindowTitle(f"{title}")
        vals = self.params_dict[title]

        self.listWidget = self.__create_list_widget(vals)
        self.layout.addWidget(self.listWidget)
        self.setLayout(self.layout)

    def __create_list_widget(self, val_list):
        listWidget = QListWidget()
        listWidget.itemClicked.connect(self.__changeParamVal)

        for val in val_list:
            item = QListWidgetItem(f"{val}")
            listWidget.addItem(item)

        # setting selection mode property
        listWidget.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        return listWidget

    def __changeParamVal(self):
        paramName = self.windowTitle()  # get the parameter name from the window title
        val = self.listWidget.currentItem().text()
        self.db.updateParam(paramName, val)
        self.close()

    def make_change_user_list(self):
        self.layout = QVBoxLayout()

        self.setWindowTitle("Change user")

        list_of_widgets = self.__create_change_user_list_widget()
        for widget in list_of_widgets:
            self.layout.addWidget(widget)

        self.setLayout(self.layout)

    def __create_change_user_list_widget(self):
        widget_list = []
        self.listWidget = QListWidget()
        # fetch player names and populate list
        rows = self.db.load_all()
        for player in rows:
            firstName = player[0]
            lastName = player[1]
            item = QListWidgetItem(f"{firstName} {lastName}")
            self.listWidget.addItem(item)

        # on player choice - load player
        self.listWidget.itemClicked.connect(self.db.loadPlayerFromList)
        self.listWidget.itemClicked.connect(self.close)
        self.listWidget.itemClicked.connect(self.win.screenHandler.refresh_screen)
        widget_list.append(self.listWidget)

        button_new_player = QPushButton()
        button_new_player.setText("Add new")
        button_new_player.clicked.connect(self.db.addPlayer)
        widget_list.append(button_new_player)

        return widget_list

class db_facade:
    def __init__(self, listGenScreen, db):
        self.listGenScreen = listGenScreen
        self.db = db
    def load_all(self):
        return self.db.load_all()

    def addPlayer(self, playerFirstName="", playerLastName="", playerGender="", playerAge=-1):
        print("Adding player")

        def getAge():
            i, okPressed = QInputDialog.getInt(self.listGenScreen, "Player age", "Your age:", 40, 18, 100, 1)
            if okPressed:
                return i

        def getName(type="first"):
            text, okPressed = QInputDialog.getText(self.listGenScreen, "Player name", f"Your {type} name:", QLineEdit.Normal, "")
            if okPressed and text != '':
                return text

        def getGender():
            genders_list = ['Male', 'Female']
            gender, okPressed = QInputDialog.getItem(self.listGenScreen, "Player gender", f"Your gender:", genders_list)
            if okPressed and gender != '':
                return gender

        if not playerFirstName:
            playerFirstName = getName("first")
        if not playerLastName:
            playerLastName = getName("last")
        if not playerGender:
            playerGender = getGender()
        if playerAge == -1:
            playerAge = getAge()

        self.db.insert(playerFirstName, playerLastName, playerGender, playerAge)
        print("inserted player successfully")
        self.__loadPlayer(playerFirstName, playerLastName)
        print("loaded player successfully")
        self.listGenScreen.win.screenHandler.refresh_screen()

        self.listGenScreen.close()

    def loadPlayerFromList(self):
        chosen_players_str = self.listGenScreen.listWidget.currentItem().text()

        name = chosen_players_str.split(" ")
        firstName = name[0]
        lastName = name[1]
        self.__loadPlayer(firstName, lastName)

    def __loadPlayer(self, firstName, lastName):
        self.db.load(firstName, lastName)

        # close player menu
        # self.win.active_screen.show()
        # self.close()

    def updateParam(self, paramName, val):
        self.db.update_parameter(paramName, val)





class Window(QMainWindow):

    def __init__(self):
        super().__init__()
        db = PlayerDataBase()

        # setting title
        self.setWindowTitle("Python ")

        self.listCreator = ListGenerator(db)
        # self.listCreator.make_list("difficulty")
        # self.listCreator.make_list("saver")
        self.listCreator.make_change_user_list()
        # showing all the widgets
        self.listCreator.show()


def testCustomList():
    # create pyqt5 app
    App = QApplication(sys.argv)

    # create the instance of our Window
    window = Window()

    # start the app
    sys.exit(App.exec())



def testDB():
    'Create player database handler'

    path = r"/home/clickerhero/Game/data_files/playersDB.db"
    db = PlayerDataBase(path=path)

    # db.insert("Guy", "Pit", 'Male', 38)

    # db.update_db_score_and_RT("Guy", "Pit", f'Songs.KOL_HAKAVOD', 490.0, [])

    # data = [(1.2, 2.3, False), (3.0, 4.0, True)]  # TL
    data = [(1, 2, [1, 2, 3], False), (3, 4, [3, 2, 4, 6, 5], True)]  # SS (true_col, clicked_col, seq, last_in_seq
    data = [(4, 4, [4, 5], False), (5, 5, [4, 5], True), (3, 4, [3, 5, 2], False)]
    print(data)
    data_string = db.list_to_str(data)
    print(data_string)
    back_to_lst = db.str_to_list(data_string)
    print(back_to_lst)

    # db.update_db_score_and_RT(f'{Songs.KOL_HAKAVOD.value}', 490.0, [0.2, 0.25, 0.3, 0.35, 0.4])


    rows = db.load_all()
    for row in rows:
        print(f"db row: {row}")

    # db.delete_whole_database()

    return db


if __name__ == '__main__':

    testDB()
    # testCustomList()