# A simple ROS node in python that publishes tf topic

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

def main():
    rospy.init_node("tf_pub", anonymous=True)
    pub = rospy.Publisher("/tf", TFMessage, queue_size=10)
    
    while not rospy.is_shutdown():
        tfstamped = TransformStamped()
        tfstamped.header = Header(0, rospy.get_rostime(), "parent_link")
        tfstamped.child_frame_id = "child_link"
        tfstamped.transform.rotation.w = 1.0

        tf_msg = TFMessage()
        tf_msg.transforms.append(tfstamped)

        pub.publish(tf_msg)

if __name__ == "__main__":
    main()
