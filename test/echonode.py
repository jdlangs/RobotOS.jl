#A simple ROS node in python that listens for and responds to communication
#from the Julia node

import rospy
from geometry_msgs.msg import PoseStamped, Vector3

from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse

class Echo(object):
    def __init__(self):
        self._pub = rospy.Publisher("poses", PoseStamped, queue_size=10)
        self._sub = rospy.Subscriber("vectors", Vector3, self.msg_cb, queue_size=10)
        self._nrecv = 0

        self._srvlisten = rospy.Service("callme", SetBool, self.srv_cb)
        self._srvcall = rospy.ServiceProxy("getplan", GetPlan)
        rospy.set_param("/received_service_call", False)

    #Translate a Vector3 message to a PoseStamped and republish
    def msg_cb(self, msg):
        pmsg = PoseStamped()
        pmsg.header.stamp = rospy.Time.now()
        pmsg.pose.position.x = msg.x
        pmsg.pose.position.y = msg.y
        pmsg.pose.position.z = msg.z
        self._pub.publish(pmsg)

        self._nrecv += 1
        rospy.set_param("/num_received_messages", self._nrecv)

    def srv_cb(self, req):
        if req.data:
            self._calltimer = rospy.Timer(rospy.Duration(2.0), self.callsrv, oneshot=True)
        rospy.set_param("/received_service_call", True)
        return SetBoolResponse(True, "")

    def callsrv(self, ev):
        req = GetPlanRequest()
        req.start.pose.position.x = 1.0
        req.goal.pose.position.y = 1.0

        rospy.wait_for_service("getplan")
        resp = self._srvcall(req)
        for pose in resp.plan.poses:
            self._pub.publish(pose)

def main():
    rospy.init_node("echo", anonymous=True)
    n = Echo()
    rospy.spin()

if __name__ == "__main__":
    main()
