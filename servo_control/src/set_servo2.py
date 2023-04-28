#!/usr/bin/env python3

import rospy
import maestro
#roslib.load_manifest('servo')
from state_machine.msg import MotorCommand

servo = maestro.Controller()

def callback(data):
    #servo = Controller()i
    print("check")
    if data.joint_name == "servo":
        channel = 8
    else:
        channel = 10
    target = int(6000+data.position*1000)
    servo.setAccel(channel,int(data.acceleration))
    servo.setTarget(channel,target)
    servo.setSpeed(channel,int(data.speed))
    print("channel,target ",channel,target)
    #servo.close()

def set_servo():
    rospy.Subscriber("/pololu/command", MotorCommand, callback)
    rospy.spin()

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('set_servo', anonymous = True)
    # Go to the main loop.
    set_servo()
