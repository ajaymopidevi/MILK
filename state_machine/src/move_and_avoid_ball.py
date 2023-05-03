#!/usr/bin/env python3

import rospy
import time

from std_msgs.msg import *
from polulu_command import PoluluCommand, polulu
from topics import DEPTH_TOPIC, BALL_TOPIC, STOP_SIGNAL_TOPIC

class MoveAndAvoidBall:
    def __init__(self) -> None:
        rospy.Subscriber(DEPTH_TOPIC, Float32MultiArray, callback=self.depth_callback)
        rospy.Subscriber(BALL_TOPIC, Bool, callback=self.ball_detection_callback)
        rospy.Subscriber(STOP_SIGNAL_TOPIC, Bool, callback=self.stop_signal_callback)

    def depth_callback(self, data):
        self.center_depth = data.data[1]
        self.left_depth = data.data[0]
        self.right_depth = data.data[2]


    def ball_detection_callback(self, data):
        self.ball_detected = data.data


    def stop_signal_callback(self, data):
        self.recieved_stop_signal = data.data


    def is_in_safe_distance_from_wall(self):
        return self.center_depth > 0.5


    def move_forward(self):
        polulu.send_motor_command(pos=0.1, speed=0)
        polulu.send_servo_command(pos=0, speed=0)


    def get_steering_angle_to_steer_away_from_ball(self):
        if self.left_depth > self.right_depth:
            angle = 0.2
        elif self.left_depth <= self.right_depth:
            angle = -0.2
        return angle


    def steer_away_from_ball(self):
        angle = self.get_steering_angle_to_steer_away_from_ball()
        start_time = time.time()
        while self.ball_detected and self.is_in_safe_distance_from_wall() or not self.recieved_stop_signal:
            polulu.send_servo_command(pos=angle, speed=0.1)
            polulu.send_motor_command(pos=angle, speed=0.1)
        end_time = time.time()
        steer_time = end_time - start_time
        return steer_time


    def steer_back_to_center(self, steer_time):
        angle = -1 * self.get_steering_angle_to_steer_away_from_ball()
        end_time = time.time() + steer_time
        while time.time() < end_time and self.is_in_safe_distance_from_wall() or not self.recieved_stop_signal:
            polulu.send_servo_command(pos=angle, speed=0.1)
            polulu.send_motor_command(pos=angle, speed=0.1)


    def avoid_ball(self):
        steer_time = self.steer_away_from_ball()
        self.steer_back_to_center(steer_time)


    def run(self):
        while not rospy.is_shutdown() or not self.recieved_stop_signal:
            while not self.ball_detected or not self.recieved_stop_signal:
                self.move_forward()
            self.avoid_ball()


if __name__ == "__main__":
    rospy.init_node('move_and_avoid_ball')
    move_and_avoid_ball = MoveAndAvoidBall()
    move_and_avoid_ball.run()
