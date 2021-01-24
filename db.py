#!/usr/bin/python
import sys
import sqlite3
import settings

dbcon = None


def getDBcon():
    return dbcon


def openDB():
    global dbcon

    try:
        dbcon = sqlite3.connect(settings.DB_FILENAME)
        cur = dbcon.cursor()
        return cur

    except Exception as e:
        print("Error %s:" % str(e))
        sys.exit(1)
