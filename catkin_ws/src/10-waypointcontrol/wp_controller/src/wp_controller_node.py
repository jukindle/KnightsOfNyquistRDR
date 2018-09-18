#!/usr/bin/env python
import rospy
from mavros_msgs.msg import State, HomePosition, GlobalPositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from math import sin, cos, sqrt, radians

import json

class WPControllerNode(object):
    def __init__(self):
        self.node_name = "Waypoints Controller Node"

        self.started = True

        # Load JSON file
        with open('/home/julien/RoboDroneRace/KnightsOfNyquistRDR/Path.json') as f:
            data = json.load(f)


        # Create array of waypoints
        self.waypoints = []
        for el in data["Waypoints"]:
            self.waypoints.append([el["lat"], el["lon"], el["alt"]])

        # Specify current and last waypoint idx
        self.currentWPidx = 0
        self.maxWPidx = len(self.waypoints)

        # Internal variables
        self.current_pos = [0,0,0]
        self.home_pos = [0,0,0]
        self.current_state = State()
        self.last_request = rospy.Time.now()



        # Subscribers
        self.sub_state = rospy.Subscriber("mavros/state", State, self.cbState, queue_size=10)
        self.sub_home = rospy.Subscriber("/mavros/home_position/home", HomePosition, self.cbHome, queue_size=10)
        self.sub_position = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.cbPosition, queue_size=1)

        # Publishers
        self.local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.global_pos_pub = rospy.Publisher('mavros/setpoint_position/global', GlobalPositionTarget, queue_size=10)



        # Services
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

        # Timer for main loop
        self.timer_pubcmd = rospy.Timer(rospy.Duration.from_sec(0.05), self.cbPubCmd)

    ### BEGIN callback functions ###
    # Callback for home location
    def cbHome(self, msg):
        lat, lon, alt = msg.geo.latitude, msg.geo.longitude, msg.geo.altitude
        self.home_pos = [lat, lon, alt]

    # Callback for global position
    def cbPosition(self, msg):
        self.current_pos = [msg.latitude, msg.longitude, msg.altitude]

    # Callback for state
    def cbState(self, msg):
        self.current_state = msg

    # Calculate distance between two GPS points TODO may be wrong approximation
    def getDistanceBetweenGPS(self, p1, p2):
        XYZ = lambda lat,lon,alt: ((6371000 + alt)*cos(lat)*cos(lon), (6371000 + alt)*cos(lat)*sin(lon), (6371000 + alt)*sin(lat))
        x1,y1,z1 = XYZ(radians(p1[0]), radians(p1[1]), radians(p1[2]))
        x2,y2,z2 = XYZ(radians(p2[0]), radians(p2[1]), radians(p2[2]))
        return sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)

    # Main loop
    def cbPubCmd(self, event):
        if not self.started: return
        if self.home_pos[2] < 1: return # Wait for home pos to be received


        # Check if distance to waypoint is small enough to count it as reached TODO may be wrong approximation and TODO change for RTK
        if self.getDistanceBetweenGPS(self.current_pos, self.waypoints[self.currentWPidx]) < 2:
            self.currentWPidx += 1
            if self.currentWPidx == self.maxWPidx:
                rospy.loginfo("MISSION DONE")
                self.started = False
                return

        # Log info about current waypoint
        rospy.loginfo("Current waypoint: " + str(self.currentWPidx) + " / " + str(self.maxWPidx))

        # Check if mode needs to be changed for OFFBOARD and ARM vehicle (this is a startup procedure)
        if str(self.current_state.mode) != "OFFBOARD" and rospy.Time.now() - self.last_request > rospy.Duration(5.0):
            resp = self.set_mode_client(0, "OFFBOARD")
            if resp.mode_sent:
                rospy.loginfo("Offboard enabled")

            self.last_request = rospy.Time.now()

        else:
            if not self.current_state.armed and rospy.Time.now() - self.last_request > rospy.Duration(5.0):
                resp= self.arming_client(True)
                if resp.success:
                    rospy.loginfo("Vehicle armed")
                self.last_request = rospy.Time.now()

        # Publish information - where should the drone fly next?
        pose = PoseStamped()
        latWP = self.waypoints[self.currentWPidx][0]
        lonWP = self.waypoints[self.currentWPidx][1]
        altWP = self.waypoints[self.currentWPidx][2]
        latHome = self.home_pos[0]
        lonHome = self.home_pos[1]
        altHome = self.home_pos[2]
        pose.pose.position.x = 6371000 * radians(lonWP - lonHome) * cos(radians(latHome))
        pose.pose.position.y = 6371000 * radians(latWP - latHome)
        pose.pose.position.z = altWP - altHome
        #self.local_pos_pub.publish(pose)


        msg = GlobalPositionTarget()

        msg.latitude = latWP
        msg.longitude = lonWP
        msg.altitude = altWP
        msg.header.stamp = rospy.Time.now()
        msg.coordinate_frame = 5
        msg.type_mask = 0b111111111000
        msg.yaw = 12.0
        self.global_pos_pub.publish(msg)
    ### END callback functions ###


    ### BEGIN internal functions ###

    # Initialize serial communication

    ### END internal functions ###


    def onShutdown(self):
        rospy.loginfo("[WPControllerNode] Shutdown.")
        self.R2serial.close()
        self.sub_cmds.unregister()


if __name__ == '__main__':
    rospy.init_node('wp_controller_node',anonymous=False)
    wp_controller_node = WPControllerNode()
    rospy.on_shutdown(wp_controller_node.onShutdown)
    rospy.spin()
