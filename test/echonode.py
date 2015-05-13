#A simple ROS node in python that listens for and responds to communication
#from the Julia node

import rospy
from geometry_msgs.msg import PoseStamped, Vector3

from std_srvs.srv import Empty, EmptyResponse
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse

class Echo(object):
    def __init__(self):
        self._pub = rospy.Publisher("poses", PoseStamped, queue_size=10)
        self._sub = rospy.Subscriber("vectors", Vector3, self.msg_cb, queue_size=10)

        self._srvlisten = rospy.Service("callme", Empty, self.srv_cb)
        self._srvcall = rospy.ServiceProxy("getplan", GetPlan)

    #Translate a Vector3 message to a PoseStamped and republish
    def msg_cb(self, msg):
        pmsg = PoseStamped()
        pmsg.header.stamp = rospy.Time.now()
        pmsg.pose.position.x = msg.x
        pmsg.pose.position.y = msg.y
        pmsg.pose.position.z = msg.z
        self._pub.publish(pmsg)

    def srv_cb(self, req):
        self._calltimer = rospy.Timer(rospy.Duration(2.0), self.callsrv, oneshot=True)
        return EmptyResponse()

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
