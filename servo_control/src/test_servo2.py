#!/usr/bin/env python3

import rospy
import sys
from servo_control.msg import ServoEsc, ServoSteer

def test_servo():
    pub = rospy.Publisher('servo', ServoSteer, queue_size=10)
    #rospy.init_node('test_servo', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    speeds = [5000,6000,6800]
    it = 0
    while not rospy.is_shutdown():
        msg = ServoSteer()
        i = int(it/10)%3
        msg.steer_servo = 9
        msg.steer_acc = 1
        msg.steer_speed = 1
        msg.steer_target = speeds[i]
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()
        it+=1

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('test_servo')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        test_servo()
    except rospy.ROSInterruptException: pass
