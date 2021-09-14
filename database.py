import sqlite3 as sql


class DataBase():
    database_name = 'playersDB.db'

    def __init__(self, firstName="", lastName="", age=-1, score=0.0):
        self.firstName = firstName
        self.lastName = lastName
        self.age = age
        self.score = score
        self.connection = sql.connect(self.database_name)
        self.cursor = self.connection.cursor()

        self.cursor.execute("""
                               CREATE TABLE IF NOT EXISTS players (
                                first_name TEXT,
                                last_name TEXT,
                                age INTEGER,
                                score FLOAT
                                )""")

        self.connection.commit()
        self.connection.close()

    def load_person(self, first_name):
        self.connection = sql.connect(self.database_name)
        self.cursor = self.connection.cursor()

        self.cursor.execute(f"""
               SELECT * FROM players
               WHERE first_name = '{first_name}'
                """)
        rows = self.cursor.fetchone()

        if rows is not None:
            self.firstName = rows[0]
            self.lastName = rows[1]
            self.age = rows[2]
            self.score = rows[3]

        else:
            self.firstName = None
            self.lastName = None
            self.age = None
            self.score = None

        self.connection.commit()
        self.connection.close()

    def insert_person(self, firstName, lastName, age):
        self.connection = sql.connect(self.database_name)
        self.cursor = self.connection.cursor()

        self.cursor.execute(f"""
                INSERT INTO players VALUES
                ('{firstName}', '{lastName}', {age}, 0.0 )
                """)

        self.connection.commit()
        self.connection.close()

    def remove_person(self, first_name):
        self.connection = sql.connect(self.database_name)
        self.cursor = self.connection.cursor()

        self.cursor.execute(f"""
                       DELETE FROM players
                       WHERE first_name = '{first_name}'
                        """)

        self.connection.commit()
        self.connection.close()


if __name__ == '__main__':
    db = DataBase()
    db.insert_person("Guy", "Zeidner", 38)
    db.load_person("Guy")
    print(db.firstName)
    print(db.lastName)
    print(db.age)
    print(db.score)
    # db.remove_person("Guy")


