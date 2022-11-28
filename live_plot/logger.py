import sqlite3

DB = "log.db"


class Writer:
    def __init__(self, table, columns):
        # initialize parameters
        self.db = DB
        self.table = table
        self.columns = columns

        # create table
        conn = sqlite3.connect(self.db)
        cursor = conn.cursor()
        columns_text = ', '.join((f'{column_name} {datatype}' for column_name, datatype in self.columns.items()))
        cursor.execute(f"DROP TABLE IF EXISTS {self.table}")
        cursor.execute(f"CREATE TABLE {self.table} ({columns_text}, "
                       f"time TIMESTAMP DEFAULT ((julianday('now') - 2440587.5) * 86400.0), PRIMARY KEY (time))")
        conn.close()

    def write(self, row):
        conn = sqlite3.connect(self.db)
        cursor = conn.cursor()
        cursor.execute(f"INSERT INTO {self.table} ({', '.join(self.columns.keys())}) "
                       f"VALUES ({','.join(['?'] * len(self.columns))})", row)
        conn.commit()
        conn.close()


class Reader:
    def __init__(self, table):
        # initialize parameters
        self.db = DB
        self.table = table

    def read(self):
        conn = sqlite3.connect(self.db)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()
        print(f"SELECT * FROM {self.table} ORDER BY time DESC LIMIT 1")
        cursor.execute(f"SELECT * FROM {self.table} ORDER BY time DESC LIMIT 1")
        out = cursor.fetchone()
        conn.close()

        return out

class Parameter:
    def __init__(self, base_value, table, column):
        self.base_value = base_value
        self.column = column

        self.reader = Reader(table)

    def __call__(self, *args, **kwargs):
        try:
            return self.reader.read()[self.column]
        except sqlite3.OperationalError as e:
            print(e)
            return self.base_value

    def __mul__(self, other):
        return self() * other