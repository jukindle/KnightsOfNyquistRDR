#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
import time
import json


WPTime = 0.5 # How long between two WPs?


# Internal variables
waypoints = []
gates = []
current_location = {"lon": 0, "lat": 0, "alt": 0}
stopped = False
lastWP = time.time()

time.sleep(5) # Wait for mavros to launch

# Callback of global position of rover - add waypoint to list if WPTime has passed
def cbPosition(msg):
    global lastWP,stopped,current_location,WPTime,waypoints
    if stopped: return
    current_location = {"lat": msg.latitude, "lon": msg.longitude, "alt": msg.altitude}
    if time.time() - lastWP > WPTime:
        waypoints.append(current_location)
        lastWP = time.time()


rospy.init_node('WPRecorder') # ROS setup

# Register callback
sub_position = rospy.Subscriber("/mavros/global_position/global", NavSatFix, cbPosition, queue_size=1)


print ("Enter [g] to add a gate at the current location")
print ("Enter [q] to stop the script and save the waypoints")
print()
inputCmd = raw_input()

# If input is 'g', we append a entry to gates array
while inputCmd != 'q':
    if inputCmd == 'g': gates.append(current_location)
    inputCmd = raw_input()

stopped = True

# Construct json and output it
jsonRaw = {"Waypoints": waypoints, "Gates": gates}
jsonString = json.dumps(jsonRaw, indent=4)

print(jsonString)


# Store json to file
text_file = open("/home/julien/RoboDroneRace/KnightsOfNyquistRDR/Path.json", "w")
text_file.write(jsonString)
text_file.close()
rospy.signal_shutdown('Quit')
