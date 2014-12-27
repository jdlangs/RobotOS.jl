#Simple republishing python script

import rospy
from geometry_msgs.msg import PoseStamped, Vector3

class Rpub(object):
    def __init__(self):
        self._pub = rospy.Publisher("poses", PoseStamped, queue_size=10)
        self._sub = rospy.Subscriber("vectors", Vector3, self.cb, queue_size=10)
    def cb(self, msg):
        pmsg = PoseStamped()
        pmsg.header.stamp = rospy.Time.now()
        pmsg.pose.position.x = msg.x
        pmsg.pose.position.y = msg.y
        pmsg.pose.position.z = msg.z
        self._pub.publish(pmsg)

def main():
    rospy.init_node("rpub", anonymous=True)
    r = Rpub()
    rospy.spin()

if __name__ == "__main__":
    main()
