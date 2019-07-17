#!/usr/bin/env python
# -*- coding: utf-8 -*-

#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    count=1
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        count=count/0.9
        hello_str = "hello world %s" % count
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
