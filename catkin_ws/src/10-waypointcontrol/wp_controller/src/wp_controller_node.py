#!/usr/bin/env python
import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped

class WPControllerNode(object):
    def __init__(self):
        self.node_name = "Waypoints Controller Node"

        self.current_state = State()
        self.last_request = rospy.Time.now()

        self.sub_state = rospy.Subscriber("mavros/state", State, self.cbState, queue_size=10)

        self.local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

        self.timer_pubcmd = rospy.Timer(rospy.Duration.from_sec(0.05), self.cbPubCmd)

    ### BEGIN callback functions ###
    def cbState(self, msg):
        self.current_state = msg

    def cbPubCmd(self, event):

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

        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 12.2
        pose.pose.position.z = 7.21
        self.local_pos_pub.publish(pose)
        return

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
