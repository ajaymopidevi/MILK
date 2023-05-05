#!/usr/bin/env python3

import rospy
import time

from std_msgs.msg import *
from polulu_command import PoluluCommand, polulu
from topics import DEPTH_TOPIC, BALL_TOPIC, STOP_SIGNAL_TOPIC

STOP_MOTOR_VALUE = -0.2
STOP_SERVO_VALUE = 0
#Slowest speed
PID_MOTOR_VALUE = 0.3

class StateMachine:
    def __init__(self) -> None:
        rospy.Subscriber(DEPTH_TOPIC, Float32MultiArray, callback=self.depth_callback)
        rospy.Subscriber(PID_TOPIC, Float32, callback=self.pid_callback)
        rospy.Subscriber(STOP_SIGNAL_TOPIC, Bool, callback=self.stop_signal_callback)


    def depth_callback(self, data):
        self.center_depth = data.data[1]
        self.left_depth = data.data[0]
        self.right_depth = data.data[2]
        # Consider only Emergency STOP
        # Rest should be handles in PID
        if self.center_depth < 1000:
            polulu.send_motor_command(pos=STOP_MOTOR_VALUE, speed=0)
            polulu.send_servo_command(pos=STOP_SERVO_VALUE, speed=0)



    def pid_callback(self, data):
        self.pid = data.data


    def stop_signal_callback(self, data):
        self.recieved_stop_signal = data.data
        polulu.send_motor_command(pos=STOP_MOTOR_VALUE, speed=0)
        polulu.send_servo_command(pos=STOP_SERVO_VALUE, speed=0)

    def run(self):
        rate = rospy.Rate(10) #10Hz
        while not rospy.is_shutdown():
            #self.pid_pub.publish(self.output)
            polulu.send_motor_command(pos=PID_MOTOR_VALUE, speed=0)
            # Right most turn -> 0.75, SERVO VALUE -> 6750
            # Left most turn -> -0.75, SERVO VALUE -> 5250
            polulu.send_servo_command(pos=self.pid, speed=0)
            rate.sleep()



if __name__ == "__main__":
    try:
        #rospy.init_node('depth_pid')
        # Set the Kp value based on how much pid values will be
        # (in-turn servo values should be, part from max and min)
        state = StateMachine()
        state.run()
    except rospy.ROSInterruptException:
        pass
