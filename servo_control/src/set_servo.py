#!/usr/bin/env python

import rospy
import maestro
roslib.load_manifest('servo')
from servo_control.msg import ServoEsc, ServoSteer

servo = maestro.Controller()

def callback(data):
    #servo = maestro.Controller()
    servo.setAccel(data.steer_servo,data.steer_acc)
    servo.setTarget(data.steer_servo,data.steer_target)
    servo.setSpeed(data.steer_servo,data.steer_speed)
    #servo.close()

def set_servo():
    rospy.Subscriber("servo", ServoSteer, callback)
    rospy.spin()

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('set_servo', anonymous = True)
    # Go to the main loop.
    set_servo()
