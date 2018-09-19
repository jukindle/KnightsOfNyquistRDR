#!/usr/bin/env python
import rospy
from mavros_msgs.msg import State, HomePosition, GlobalPositionTarget, ParamValue
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from math import sin, cos, sqrt, radians, atan, atan2, pi

import json

class WPControllerNode(object):
    def __init__(self):
        self.node_name = "Waypoints Controller Node"

        self.started = True
        self.ready = False

        # Load JSON file
        with open('/home/julien/RoboDroneRace/KnightsOfNyquistRDR/Path.json') as f:
            data = json.load(f)


        # Create array of waypoints
        self.waypoints = []
        for el in data["Waypoints"]:
            self.waypoints.append([el["lat"], el["lon"], el["alt"], el["vel"] if "vel" in el else 4.0])

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
        self.set_vel = rospy.ServiceProxy('/mavros/param/set', ParamSet)



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
        velWP = self.waypoints[self.currentWPidx][3]
        # latHome = self.home_pos[0]
        # lonHome = self.home_pos[1]
        # altHome = self.home_pos[2]
        # pose.pose.position.x = 6371000 * radians(lonWP - lonHome) * cos(radians(latHome))
        # pose.pose.position.y = 6371000 * radians(latWP - latHome)
        # pose.pose.position.z = altWP - altHome
        #self.local_pos_pub.publish(pose)

        if self.ready:
            msg = GlobalPositionTarget()
            msg.latitude = latWP
            msg.longitude = lonWP
            msg.altitude = altWP
            msg.header.stamp = rospy.Time.now()
            msg.coordinate_frame = 5
            msg.type_mask = 0b101111111000


            msg.yaw = self.getNextYaw()
            self.global_pos_pub.publish(msg)

            par = ParamValue()
            par.integer = 0
            par.real = velWP
            try:
                self.set_vel("MPC_XY_VEL_MAX", par)
            except Exception:
                print("e")




                
        else:
            pose = PoseStamped()
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = 2.0
            self.local_pos_pub.publish(pose)

            try:
                par = ParamValue()
                par.integer = 0
                par.real = velWP
                resp = self.set_vel("MPC_XY_VEL_MAX", par)
                if resp.success: self.ready = True
            except Exception as e:
                print(e)



    ### END callback functions ###


    ### BEGIN internal functions ###

    def getNextYaw(self):

        if self.currentWPidx >= self.maxWPidx -1: return 0.0
        latWP = self.waypoints[self.currentWPidx][0]
        lonWP = self.waypoints[self.currentWPidx][1]
        altWP = self.waypoints[self.currentWPidx][2]

        latWPn = self.waypoints[self.currentWPidx+1][0]
        lonWPn = self.waypoints[self.currentWPidx+1][1]
        altWPn = self.waypoints[self.currentWPidx+1][2]

        dx = 6371000 * radians(lonWP - lonWPn) * cos(radians(latWP))
        dy = 6371000 * radians(latWP - latWPn)

        return (((atan2(dy, dx) + pi) + pi) % (2*pi)) - pi

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
