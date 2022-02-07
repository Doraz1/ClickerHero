import sqlite3 as sql

class PlayerDataBase():
    topic = 'players'
    database_name = f'{topic}DB.db'

    def __init__(self):
        self.firstName = ""
        self.lastName = ""
        self.age = -1
        self.scores = []

        query = f"""CREATE TABLE IF NOT EXISTS {self.topic} 
                    (first_name TEXT, 
                    last_name TEXT, 
                    age INTEGER, 
                    song1_score FLOAT, 
                    song2_score FLOAT)"""

        self.execute_query(query)

    def load(self, first_name, last_name):
        query = f"""select * from {self.topic}
                    where first_name = '{first_name}' and last_name = '{last_name}'"""
        rows = self.execute_query(query, return_rows=True)
        if rows:
            user = rows[0] # first user found
            self.firstName = user[0]
            self.lastName = user[1]
            self.age = user[2]
            self.scores = []
            for song_score in user[3:]:
                self.scores.append(song_score)
            print(f"Loaded play profile of {self.firstName} {self.lastName}")
        else:
            # empty list
            self.firstName = ""
            self.lastName = ""
            self.age = -1
            self.scores = []

    def load_all(self):
        query = f"""SELECT * FROM {self.topic}"""
        rows = self.execute_query(query, return_rows=True)

        return rows

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
                        ('{firstName}', '{lastName}', {age}, 0.0, 0.0 )"""

            self.execute_query(query)

    def remove(self, first_name, last_name):
        query = f"""delete from {self.topic}
                    where first_name = '{first_name}' and last_name = '{last_name}'"""

        self.execute_query(query)

    def update_score(self, song_name, new_score):
        first_name = self.firstName
        last_name = self.lastName

        query = f"""SELECT {song_name}_score FROM {self.topic}
               WHERE first_name = '{first_name}' AND last_name = '{last_name}'"""

        song_score = self.execute_query(query, return_rows=True)[0][0]
        if song_score < new_score:
            query = f"""Update {self.topic}
                            set {song_name}_score = {new_score}
                            where first_name = '{first_name}' and last_name = '{last_name}'"""

            self.execute_query(query)

            # update scores
            query = f"""select * from {self.topic}
                                where first_name = '{first_name}' and last_name = '{last_name}'"""
            rows = self.execute_query(query, return_rows=True)
            if rows:
                user = rows[0]  # first user found
                self.scores = []
                for song_score in user[3:]:
                    self.scores.append(song_score)

            print("---------------------------------\n"
                  "Congratulations! New record score!\n"
                  f"Player high score: {new_score} exceeds current performance: {song_score}!\n"
                  "---------------------------------")
        else:
            print("---------------------------------\n"
                  "Didn't beat top score. Better luck next time!"
                  "---------------------------------")



    def execute_query(self, query, return_rows = False):
        self.connection = sql.connect(self.database_name)
        self.cursor = self.connection.cursor()

        self.cursor.execute(query)
        self.connection.commit()
        rows = self.cursor.fetchall()

        self.connection.close()
        if return_rows:
            return rows

    def delete_whole_database(self):
        query = f"""delete from {self.topic}"""
        self.execute_query(query)




if __name__ == '__main__':
    db = PlayerDataBase()
    # db.insert("Guy", "Pit", 38)
    rows = db.load_all()
    for row in rows:
        print(row)
    db.update_score("Guy", "Pit", "song2", 490.0)
    # db.delete_whole_database()
    rows = db.load_all()
