#!/usr/bin/python3

# rtl_433 >> rtl_433.log &
# tail -f rtl_433.log | ./voda.py

volume_offset = 0.0

import time
import mysql.connector
import sys
import select
import re
import subprocess

def insert_into_database(timestamp, datetime, sensor_id, channel_id, counter, volume):
  cnx = mysql.connector.connect(user='voda', password="vodenjak", database='voda')
  cursor = cnx.cursor()
  add_reading = ("INSERT INTO voda "
                 "(timestamp, datetime, sensor_id, channel_id, counter, volume) "
                 "VALUES (%s, %s, %s, %s, %s, %s)")
  data_reading = (timestamp, datetime, sensor_id, channel_id, counter, volume)
  cursor.execute(add_reading, data_reading)
  data_no = cursor.lastrowid
  cnx.commit()
  cursor.close()
  cnx.close()

# read standard input
# when sensor reading is detected (String "Honeywell")
# parse the counter and sensor id
# update to database and notify user about water event

pattern = re.compile("^"
                     "([-0-9]+ [0-9:]+) :"
                     "[ \t]*Honeywell Door/Window Sensor[ \t]*:"
                     "[ \t]*([0-9a-fA-F]+)[ \t]*:"
                     "[ \t]*([0-9a-fA-F]+)[ \t]*:"
                     "[ \t]*([0-9a-fA-F]+)[ \t]*:"
                     "[ \t]*[0-9a-zA-Z]+[ \t]*:"
                     "[ \t]*[0-9a-zA-Z]+[ \t]*"
                     "$")
old_event_counter = 0
# timeout = 15.0 # s
# while sys.stdin in select.select([sys.stdin], [], [], timeout)[0]:
while sys.stdin in select.select([sys.stdin], [], [])[0]:
  line = sys.stdin.readline()
  if line:
    match = pattern.match(line)
    if match:
      event_counter = int(match.group(2), 16)
      if event_counter != old_event_counter:
        old_event_counter = event_counter
        event_time = match.group(1)
        event_channel = int(match.group(3), 16)
        event_sensorid = int(match.group(4), 16)
        volume = volume_offset + 0.05 * event_counter
        print("%s sensor:%d ch:%d counter:%d -> %9.1f m^3"
             % (event_time, event_sensorid, event_channel, event_counter, volume) )
        timestamp = int(time.time())
        insert_into_database(timestamp, event_time, event_sensorid, event_channel, event_counter, volume)
        subprocess.call(["/home/user/bin/ringtone-voda-test.sh", "0"]) # audible notification
  else: # an empty line means stdin has been closed
    print('*** EOF ***')
    exit(0)
else:
  print("*** TIMEOUT ***")
