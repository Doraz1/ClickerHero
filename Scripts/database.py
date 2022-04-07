import sqlite3 as sql
import sys

from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
)
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QWidget, QPushButton, QVBoxLayout, QInputDialog, QLineEdit, QListWidget, QListWidgetItem
from PyQt5.QtGui import QFont


class PlayerDataBase():
    topic = 'players'
    database_name = f'{topic}DB.db'

    def __init__(self):
        self.firstName = ""
        self.lastName = ""
        self.age = -1
        self.scores = []
        self.difficulty = 1
        self.playerLoaded = False
        self.sql_dict = {0: 'first_name TEXT',
                         1: 'last_name TEXT',
                         2: 'age INTEGER',
                         3: 'difficulty INTEGER',
                         4: 'one_kiss_score FLOAT',
                         5: 'cant_help_falling_in_love_score FLOAT',
                         }
        query = f"""CREATE TABLE IF NOT EXISTS {self.topic} 
                    (
                    {self.sql_dict[0]},
                    {self.sql_dict[1]},
                    {self.sql_dict[2]},
                    {self.sql_dict[3]},
                    {self.sql_dict[4]},
                    {self.sql_dict[5]}
                    )"""

        self.__execute_query(query)

    def load_all(self):
        query = f"""SELECT * FROM {self.topic}"""
        rows = self.__execute_query(query, return_rows=True)

        return rows

    def load(self, first_name, last_name):
        def searchDict(dict, substring):
            for k in dict:
                for v in dict[k]:
                    if substring in v:
                        return k
            return None
        query = f"""select * from {self.topic}
                    where first_name = '{first_name}' and last_name = '{last_name}'"""
        rows = self.__execute_query(query, return_rows=True)
        if rows:
            user = rows[0] # get first user found
            firstNameInd = searchDict(self.sql_dict, 'first_name')
            print(f"first nam ind: {firstNameInd}")
            self.firstName = user[0]
            self.lastName = user[1]
            self.age = user[2]
            self.scores = []
            for song_score in user[4:]:
                self.scores.append(song_score)
            print(f"Loaded play profile of {self.firstName} {self.lastName}")
            self.playerLoaded = True
        else:
            # empty list
            self.firstName = ""
            self.lastName = ""
            self.age = -1
            self.scores = []
            self.playerLoaded = False

    def insert(self, firstName, lastName, age):
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
                        ('{firstName}', '{lastName}', {age}, 1, 0.0, 0.0 )"""

            self.__execute_query(query)

    def remove(self, first_name, last_name):
        query = f"""delete from {self.topic}
                    where first_name = '{first_name}' and last_name = '{last_name}'"""

        self.__execute_query(query)

    def update_score(self, song_name, new_score):
        first_name = self.firstName
        last_name = self.lastName

        'Get current score'
        query = f"""SELECT {song_name}_score FROM {self.topic}
               WHERE first_name = '{first_name}' AND last_name = '{last_name}'"""

        song_score = self.__execute_query(query, return_rows=True)[0][0]
        if song_score < new_score:
            print("---------------------------------\n"
                  "Congratulations! New record score!\n"
                  f"Player high score: {new_score} exceeds current performance: {song_score}!\n"
                  "---------------------------------")

            'Update score in database'
            query = f"""Update {self.topic}
                            set {song_name}_score = {new_score}
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

        else:
            print("---------------------------------\n"
                  "Didn't beat top score. Better luck next time!"
                  "---------------------------------")

    def update_parameter(self, param, new_val):
        'Update parameter in database'
        query = f"""Update {self.topic}
                    set {param} = {new_val}
                    where first_name = '{self.firstName}' and last_name = '{self.lastName}'"""
        print(f"Updated {param} to {new_val}")

        self.__execute_query(query)

    def get_parameter(self, paramName):
        'Update parameter in database'
        query = f"""select * from {self.topic}
                            where first_name = '{self.firstName}' and last_name = '{self.lastName}'"""
        rows = self.__execute_query(query, return_rows=True)

        if rows:
            user = rows[0]  # get first user found
            difficulty = user[3]
            return difficulty
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
    params_dict = {"difficulty": {1, 2, 3},
                   "saver": {0, 1}}
    font = QFont('Times', 45, QFont.Bold)

    def __init__(self, win, db):
        super(ListGenerator, self).__init__()
        self.win = win
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
        self.listWidget.itemClicked.connect(self.win.refresh_screen)
        widget_list.append(self.listWidget)

        button_new_player = QPushButton()
        button_new_player.setText("Add new")
        button_new_player.clicked.connect(self.db.addPlayer)
        widget_list.append(button_new_player)

        return widget_list

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


class db_facade:
    def __init__(self, listGenScreen, db):
        self.listGenScreen = listGenScreen
        self.db = db
    def load_all(self):
        return self.db.load_all()

    def addPlayer(self):
        print("Adding player")

        def getAge():
            i, okPressed = QInputDialog.getInt(self.listGenScreen, "Player age", "Your age:", 40, 18, 100, 1)
            if okPressed:
                return i

        def getName(type="first"):
            text, okPressed = QInputDialog.getText(self.listGenScreen, "Player name", f"Your {type} name:", QLineEdit.Normal, "")
            if okPressed and text != '':
                return text

        playerFirstName = getName("first")
        playerLastName = getName("last")
        playerAge = getAge()

        self.db.insert(playerFirstName, playerLastName, playerAge)
        self.__loadPlayer(playerFirstName, playerLastName)

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



def testDB():
    db = PlayerDataBase()
    # db.insert("Guy", "Pit", 38)
    rows = db.load_all()
    for row in rows:
        print(row)
    db.update_score("Guy", "Pit", "song2", 490.0)
    # db.delete_whole_database()
    rows = db.load_all()


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


if __name__ == '__main__':
    # testDB()
    testCustomList()