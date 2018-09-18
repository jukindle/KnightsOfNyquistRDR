
import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped


current_state = State()




def state_cb(msg):
    global current_state
    current_state = msg
    rospy.loginfo("State: " + str(msg.mode))


rospy.init_node('testiiiing')

rospy.Subscriber("mavros/state", State, state_cb, queue_size=10)
local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)


rate = rospy.Rate(20.0)

rospy.loginfo("1")


rospy.loginfo("2")


pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 2

for i in range(0,100):
    local_pos_pub.publish(pose)
    rate.sleep()


offb_set_mode = "OFFBOARD"

arm_cmd = True

last_request = rospy.Time.now()

rospy.loginfo("3")
while not rospy.is_shutdown():
    rospy.loginfo("4")
    rospy.loginfo(current_state.mode)
    if str(current_state.mode) != "OFFBOARD" and rospy.Time.now() - last_request > rospy.Duration(5.0):
        resp = set_mode_client(0, offb_set_mode)
        if resp.mode_sent:
            rospy.loginfo("Offboard enabled")

        last_request = rospy.Time.now()

    else:
        if not current_state.armed and rospy.Time.now() - last_request > rospy.Duration(5.0):
            resp= arming_client(arm_cmd)
            if resp.success:
                rospy.loginfo("Vehicle armed")
            last_request = rospy.Time.now()

    local_pos_pub.publish(pose)

    rate.sleep()
