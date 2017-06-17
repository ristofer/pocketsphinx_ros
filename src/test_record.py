#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Empty

def talker():
    record_pub = rospy.Publisher('record', String, queue_size=10)
    stop_pub = rospy.Publisher('stop',Empty,queue_size=10)
    rospy.init_node('test_record', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        record_pub.publish(raw_input("Name of file: "))
        print "Press enter to stop"
        raw_input()
        stop_pub.publish()
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass