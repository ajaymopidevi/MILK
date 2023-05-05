
import rospy

from std_msgs.msg import *
from state_machine.msg import *

from polulu_command import polulu

class _Straight():
    def __init__(self) -> None:
        rospy.Subscriber('/DepthInfo', Float32MultiArray, callback=self.sub_depth_callback)

    def sub_depth_callback(self, data):
        self.center_depth = data.center_depth
        self.left_depth = data.left_depth
        self.right_depth = data.right_depth

    def get_polulu_servo_pos(self, control):
        angle = 0 # TODO
        if control > 0: # We are going more towards left so we should turn a bit right
            angle = 0.75
        elif control <=0 : # We are going more towards right so we should turn a bit right
            angle = -0.75
        return angle
    
    def get_polulu_motor_pos(self, fast):
        if fast: # TODO
            return 0.8
        return 0.7

    def go(self, pid, fast=True):
        motor_pos, servo_pos = self.get_polulu_motor_pos(fast), self.get_polulu_servo_pos(pid)
        polulu.send_motor_command(pos=motor_pos, speed=0)
        polulu.send_servo_command(pos=servo_pos, speed=0)

straight = _Straight()