#!/usr/bin/env python
import rospy
from mavros_msgs.msg import State, HomePosition, GlobalPositionTarget, ParamValue
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
from geometry_msgs.msg import PoseStamped, Vector3, TwistStamped
from sensor_msgs.msg import NavSatFix
import math
import json

class WPControllerNode(object):
    def __init__(self):
        self.node_name = "Waypoints Controller Node"

        self.started = True # Describes if we're ready to do the mission (we could implement a big red button xD)
        self.readyForMission = False # Describes if system has taken off and WP mission ready

        # Load JSON file
        with open('/home/julien/RoboDroneRace/KnightsOfNyquistRDR/Path.json') as f:
            data = json.load(f)

        # Create array of waypoints
        self.waypoints = [] # [latitude, longitude, altitude, velocity, exactness]
        for el in data["Waypoints"]:
            self.waypoints.append([el["lat"], el["lon"], el["alt"], el["vel"] if "vel" in el else 4.0, el["exa"] if "exa" in el else 1.0])

        # Specify current and last waypoint idx
        self.currentWPidx = 0
        self.maxWPidx = len(self.waypoints)

        # Internal variables
        self.current_pos = [0,0,0] # [lat, lon, alt]
        self.home_pos = [0,0,0] # [lat, lon, alt]
        self.current_state = State() # State of vehicle
        self.last_request = rospy.Time.now()


        # Subscribers
        self.sub_state = rospy.Subscriber("mavros/state", State, self.cbState, queue_size=10) # State information
        self.sub_home = rospy.Subscriber("/mavros/home_position/home", HomePosition, self.cbHome, queue_size=10) # Home position
        self.sub_position = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.cbPosition, queue_size=1) # Vehicle position


        # Publishers
        self.local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10) # Set position relative to home pos
        #self.global_pos_pub = rospy.Publisher('mavros/setpoint_position/global', GlobalPositionTarget, queue_size=10) # Set global pos
        self.pub_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10) # Set velocity in [x y z] coorinate system (x East, y North)


        # Services
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool) # Request the vehicle to start motors
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode) # Change mode (OFFBOARD, AUTO, MANUAL etc.)
        self.set_vel = rospy.ServiceProxy('/mavros/param/set', ParamSet) # Set a vehicle parameter like max. velocity


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

    # Calculate distance between two GPS points TODO may be wrong approximation NOTE seems to work fine
    def getDistanceBetweenGPS(self, p1, p2):
        XYZ = lambda lat,lon,alt: ((6371000 + alt)*math.cos(lat)*math.cos(lon), (6371000 + alt)*math.cos(lat)*math.sin(lon), (6371000 + alt)*math.sin(lat))
        x1,y1,z1 = XYZ(math.radians(p1[0]), math.radians(p1[1]), math.radians(p1[2]))
        x2,y2,z2 = XYZ(math.radians(p2[0]), math.radians(p2[1]), math.radians(p2[2]))
        return math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)


    ### MAIN LOOP ### Executed at 20Hz
    def cbPubCmd(self, event):
        if not self.started: return # Wait for start
        if self.home_pos[2] < 1: return # Wait for home pos to be received


        # Check if distance to waypoint is small enough to count it as reached - this is the exactness variable in the waypoints array
        if self.getDistanceBetweenGPS(self.current_pos, self.waypoints[self.currentWPidx]) < self.waypoints[self.currentWPidx][4]:
            self.currentWPidx += 1
            if self.currentWPidx == self.maxWPidx:
                rospy.loginfo("MISSION DONE")
                self.started = False
                return


        # Check if vehicle is in OFFBOARD mode and ARMED, if not apply changes
        self.checkIfStartedAndStart()

        pose = PoseStamped()

        # Obtain waypoint information
        latWP = self.waypoints[self.currentWPidx][0]
        lonWP = self.waypoints[self.currentWPidx][1]
        altWP = self.waypoints[self.currentWPidx][2]
        velWP = self.waypoints[self.currentWPidx][3]

        # Obtain current position
        latRov = self.current_pos[0]
        lonRov = self.current_pos[1]
        altRov = self.current_pos[2]

        # Check if we reached altitude of 2.0 meters above ground and then start
        if not self.readyForMission:
            success = self.takeOff()
            if success: self.readyForMission = True
            return

        # Log info about current waypoint
        rospy.loginfo("Current waypoint: " + str(self.currentWPidx) + " / " + str(self.maxWPidx))

        # Calculate x,y,z vector between current pos and target
        y = math.radians(latWP - latRov) * 6371000 * math.cos(math.radians(lonRov))
        x = math.radians(lonWP - lonRov) * 6371000
        z = altWP - altRov

        # Obtain unit vector and scale it according to WP velocity
        scal = math.sqrt(x**2 + y**2 + z**2)
        v = velWP
        vx = x / scal * v
        vy = y / scal * v
        vz = z / scal * v

        # Create message of velocity vector and publish it
        msg = TwistStamped()
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        self.pub_vel.publish(msg)


    ### END callback functions ###


    ### BEGIN internal functions ###

    # Check if mode needs to be changed for OFFBOARD and ARM vehicle (this is a startup procedure) and apply changes if necessary
    def checkIfStartedAndStart(self):
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

    # Take off to 2.0m above home position
    def takeOff(self):
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 2.0
        self.local_pos_pub.publish(pose)

        return self.current_pos[2] - self.home_pos[2] > 1.8

    # Function for flying to absolute WP
    def flyToAbsWP(self, lat, lon, alt, yaw):
        msg = GlobalPositionTarget()
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        msg.header.stamp = rospy.Time.now()
        msg.coordinate_frame = 5
        msg.type_mask = 0b101111111000
        msg.yaw = yaw
        self.global_pos_pub.publish(msg)

    # Calculate Yaw to next waypoint
    def getNextYaw(self):

        if self.currentWPidx >= self.maxWPidx -1: return 0.0
        latWP = self.waypoints[self.currentWPidx][0]
        lonWP = self.waypoints[self.currentWPidx][1]
        altWP = self.waypoints[self.currentWPidx][2]

        latWPn = self.waypoints[self.currentWPidx+1][0]
        lonWPn = self.waypoints[self.currentWPidx+1][1]
        altWPn = self.waypoints[self.currentWPidx+1][2]

        dx = 6371000 * math.radians(lonWP - lonWPn) * math.cos(math.radians(latWP))
        dy = 6371000 * math.radians(latWP - latWPn)

        return (((math.atan2(dy, dx) + math.pi) + math.pi) % (2*math.pi)) - math.pi


    ### END internal functions ###


    def onShutdown(self):
        rospy.loginfo("[WPControllerNode] Shutdown.")


if __name__ == '__main__':
    rospy.init_node('wp_controller_node',anonymous=False)
    wp_controller_node = WPControllerNode()
    rospy.on_shutdown(wp_controller_node.onShutdown)
    rospy.spin()
