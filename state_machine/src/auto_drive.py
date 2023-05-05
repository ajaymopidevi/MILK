import time

import rospy

from std_msgs.msg import *
from state_machine.msg import *

from topics import DEPTH_TOPIC, STOP_SIGNAL_TOPIC, STOP_POLOLU
from motion import straight, turn_right, stop
from pid_controller import pid_controller

class State:
    STRAIGHT_FAST = 0
    STRAIGHT_SLOW = 1
    STOP = 2
    TURN_RIGHT = 3

class AutonomousDriving:
    def __init__(self) -> None:
        rospy.Subscriber(DEPTH_TOPIC, Float32MultiArray, callback=self.sub_depth_callback)
        rospy.Subscriber(STOP_SIGNAL_TOPIC, Bool, callback=self.sub_stop_sign_callback)
        rospy.Subscriber(STOP_POLOLU, Bool, callback=self.sub_stop_pololu_callback)
        self.pid = pid_controller
        self.start_time = time.time()
        self.stop_polulu = False

    def sub_depth_callback(self, data):
        self.center_depth = data.center_depth
        self.left_depth = data.left_depth
        self.right_depth = data.right_depth

    def sub_stop_sign_callback(self, data):
        self.is_stop_sign = data.data

    def sub_stop_polulu_callback(self, data): # TODO
        self.stop_polulu = data.data
        stop.go()
    
    def time_passed(self):
        return time.time() - self.start_time

    def determine_state(self): # TODO
        if self.time_passed() <= 2:
            return State.STRAIGHT_FAST
        
    
    def run(self):
        while not self.stop_polulu:
            pid = self.pid.update(self.right_depth, self.left_depth, self.center_depth)
            state = self.determine_state()
            if state == State.STRAIGHT_FAST:
                straight.go(fast=True, pid=pid)
            elif state == State.STRAIGHT_SLOW:
                straight.go(fast=False, pid=pid)
            elif state == State.TURN_RIGHT:
                turn_right.go(pid=pid)
            elif state == State.STOP:
                stop.go()
        stop.go()

if __name__ == "__main__":
    rospy.init_node('auto_drive')
    auto_driving = AutonomousDriving()
    auto_driving.run()