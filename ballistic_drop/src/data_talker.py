#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Float64MultiArray

def data_talker():
    data = [[0, 0, 40, 0, 400, 5, 0, 0], [0, 0, 40, 0, 400, 0, 5, 0], [0, 0, 40, 90, 400, 5, 0, 0], [0, 0, 40, 90, 400, 0, 5, 0]]
    i = 0
    pub = rospy.Publisher('data_chatter', Float64MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    
    while not rospy.is_shutdown():
        msg = Float64MultiArray()
        msg.data = []
        msg.data = data[i]
        rospy.loginfo(msg)
        pub.publish(msg)

        # hello_array = data[i]
        # rospy.loginfo(hello_array)
        # pub.publish(hello_array)

        i = (i+1)%3

        rate.sleep()

if __name__ == '__main__':
    try:
        data_talker()
    except rospy.ROSInterruptException:
        pass