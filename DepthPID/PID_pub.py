#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import *


class DepthPID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        #self.Ki = Ki
        #self.Kd = kd
        self.current_time = time.time()
        self.last_time = self.current_time
        self.output = 0
        self.pid_pub = rospy.Publisher('/pid',Float32, queue_size = 10)
        rospy.Subscriber("/DepthInfo", Float32MultiArray ,self.depth_callback)
        self.error = 0
        self.turn_thresh = 1500
        # When in straight line, and in center, left and right calues re close to 1800
        self.center_value = 1800

    def update(self, error):
        delta_time = time.time() - self.last_time
        if delta_time>0.05:
            P = self.Kp * error
            self.output = P
            self.last_time = time.time()
        #return self.output

    def depth_callback(self, data):
        # While giving the right turn when you see a hallway passage on right,
        # even if left values are higher
        # 2000 is the max right turn possible (Multiply with Kp)
        if data.data[2]>3600:
            self.error = 2000
            return
        # When left values are closer, depth values might be 0, Take right most turn
        if data.data[0] == 0:
            self.error = 2000
            return


        self.error = data.data[2] - data[0]

        #Close to left side of the wall, Take a right turn
        self.error = data.data[2] - self.center_value

        #Close to right side of the wall, Take a left turn
        self.error = self.center_value - data.data[0]

        # Open hallway - need to take a turn
        if data.data[1] < self.turn_thresh + 500:
            # When taking right-turn in the hallway, you see a new hallway on the right
            # in that case , left values might be higher than right values prompting for left turn
            # Should also cover when taking right turn
            # Close to wall , take faster right turn -> Hogher right pid

            if self.error < 1000:
                self.error = 1000
            # FAILURE CASE: In the miidle of the hallway, car somehow faces the wall,
            # then it takes opposite turn and in next hallway it goes right and hits the wall
            # If it can Right is not hardcoded, this might not be a failure




        # Slightly in center no-change
        if abs(self.error)<200:
            self.error = 0
        """
        # If both values are greater than the threshold, adjust to the center based on difference
        if (data.data[2] >= turn_thresh and data.data[0] >= turn_thresh) or (data.data[0]<turn_thresh and data.data[1] <turn_thresh):
            error = data.data[2] - data.data[0]
            print("Check",data.data)
        elif data.data[0] > turn_thresh:
            # Right values less ---> right side close to obstacle
            # Take left turn ---> Error negative
            print('Using right depth: ',data.data[0],data.data[1],data.data[2])
            error = -(data.data[2]-1800)
        elif data.data[2] > turn_thresh:
            print('Using left depth: ',data.data[0],data.data[1],data.data[2])
            error = data.data[0]-1800
        else:
            print('Else condition in error')
            error = data.data[2] - data.data[0]
        if abs(error) <= 200:
            error = 0
        print("Check",error)
        """

    def run(self):
        rate = rospy.Rate(10) #10Hz
        while not rospy.is_shutdown():
            self.update(self.error)
            # Right most servo value
            if self.output > 0.75:
                self.output = 0.75
            elif self.output < 0.75:
                self.output = -0.75
            self.pid_pub.publish(self.output)
            rate.sleep()





if __name__ == "__main__":
    try:
        rospy.init_node('depth_pid')
        # Set the Kp value based on how much pid values will be
        # (in-turn servo values should be, part from max and min)
        pid_node = DepthPID(0.18/1500,0,0)
        pid_node.run()
    except rospy.ROSInterruptException:
        pass
