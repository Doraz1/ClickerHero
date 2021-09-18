import sqlite3 as sql


class PlayerDataBase():
    topic = 'players'
    database_name = f'{topic}DB.db'

    def __init__(self):
        self.firstName = ""
        self.lastName = ""
        self.age = -1
        self.score = -1

        self.connection = sql.connect(self.database_name)
        self.cursor = self.connection.cursor()

        self.cursor.execute(f"""
                               CREATE TABLE IF NOT EXISTS {self.topic} (
                                first_name TEXT,
                                last_name TEXT,
                                age INTEGER,
                                score FLOAT
                                )""")

        self.connection.commit()
        self.connection.close()

    def load_person(self, first_name, last_name):
        self.connection = sql.connect(self.database_name)
        self.cursor = self.connection.cursor()

        self.cursor.execute(f"""
               SELECT * FROM {self.topic}
               WHERE first_name = '{first_name}' AND last_name = '{last_name}'
                """)
        rows = self.cursor.fetchone()

        if rows is not None:
            self.firstName = rows[0]
            self.lastName = rows[1]
            self.age = rows[2]
            self.score = rows[3]

        else:
            self.firstName = ""
            self.lastName = ""
            self.age = -1
            self.score = -1

        print(f"Loaded play profile of {self.firstName} {self.lastName}")

        self.connection.commit()
        self.connection.close()

    def load_all_players(self):
        self.connection = sql.connect(self.database_name)
        self.cursor = self.connection.cursor()

        self.cursor.execute(f"""
                            SELECT * FROM {self.topic}
                             """)

        rows = self.cursor.fetchall()

        self.connection.commit()
        self.connection.close()

        return rows

    def insert_person(self, firstName, lastName, age):
        self.connection = sql.connect(self.database_name)
        self.cursor = self.connection.cursor()

        self.cursor.execute(f"""
                            INSERT INTO {self.topic} VALUES
                             ('{firstName}', '{lastName}', {age}, 0.0 )
                             """)

        self.connection.commit()
        self.connection.close()

    def remove_person(self, first_name, last_name):
        self.connection = sql.connect(self.database_name)
        self.cursor = self.connection.cursor()

        self.cursor.execute(f"""
                            DELETE FROM {self.topic}
                            WHERE first_name = '{first_name}' AND last_name = '{last_name}'
                            """)

        self.connection.commit()
        self.connection.close()


class SongDataBase():
    topic = 'songs'
    database_name = f'{topic}DB.db'

    def __init__(self):
        self.song_name = ""
        self.song_file_name = ""
        self.song_BPM = -1

        self.connection = sql.connect(self.database_name)
        self.cursor = self.connection.cursor()

        self.cursor.execute(f"""CREATE TABLE IF NOT EXISTS {self.topic} 
                            (song_name TEXT, song_file_name TEXT, song_BPM INTEGER, special_beats LIST)""")

        self.connection.commit()
        self.connection.close()

    def load_song(self, song_name):
        self.connection = sql.connect(self.database_name)
        self.cursor = self.connection.cursor()

        self.cursor.execute(f"""SELECT * FROM {self.topic} WHERE song_name = '{song_name}'""")
        rows = self.cursor.fetchone()

        if rows is not None:
            self.song_name = rows[0]
            self.song_file_name = rows[1]
            self.song_BPM = rows[2]
        else:
            self.song_name = ""
            self.song_file_name = ""
            self.song_BPM = -1

        print(f"Loaded song data of {self.song_name}")

        self.connection.commit()
        self.connection.close()

    def load_all_songs(self):
        self.connection = sql.connect(self.database_name)
        self.cursor = self.connection.cursor()

        self.cursor.execute(f"""SELECT * FROM {self.topic}""")

        rows = self.cursor.fetchall()

        self.connection.commit()
        self.connection.close()

        return rows

    def insert_song(self, song_name, song_file_name, song_BPM, song_special_beats_list):
        self.connection = sql.connect(self.database_name)
        self.cursor = self.connection.cursor()

        self.cursor.execute(f"""INSERT INTO {self.topic} VALUES('{song_name}', '{song_file_name}', {song_BPM}, 0.0 )""")

        self.connection.commit()
        self.connection.close()

    def remove_song(self, song_name):
        self.connection = sql.connect(self.database_name)
        self.cursor = self.connection.cursor()

        self.cursor.execute(f"""
                            DELETE FROM {self.topic}
                            WHERE song_name = '{song_name}'
                            """)

        self.connection.commit()
        self.connection.close()


if __name__ == '__main__':
    db = PlayerDataBase()
    db.insert_person("Guy", "Zeidner", 38)
    db.insert_person("Guy", "Pitner", 48)
    db.load_person("*", "*")
    # db.remove_person("Guy", "Zeidner)

    # db = SongDataBase()
    # db.insert_song("Guy song", "path_to_guy", 138, )
    # db.load_song("Guy song")
    # print(db.song_BPM)


