import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import numpy as np

class DepthSubscriber(object):
    self.depth_height = 480
    self.depth_width = 640
    self.width_dim = 40
    self.height_dim = 50
    self.width_dim_LR = 50
    self.height_dim_LR = 50
    self.cx = depth_width/2
    self.cy = depth_height/2

    #self.corners = getCorners(width_dim, height_dim, cx, cy, depth_width)
    self.corners = getCorners()
    rospy.init_node('DepthDistance')
    self.depthPub = rospy.Publisher('DepthInfo', Flaot32MultiArray, queue_size=1) 
    rospy.Subscriber("/camera/depth", Image, depth_callback)

    def getCorners(self):
        """
        For a 640x480 image, we consider corners
        Left: 20:20+50, 215:265
        Center: 300:340, 215:265
        Right: 580:620, 215:265
        """
        corners = []
        corners.push_back(20) # Left width i.e -20 to +20 around center
        corners.push_back(self.cy - (self.height/2)) 
        corners.push_back(self.cx - (self.width/2))
        corners.push_back(self.cy - (height/2))
        corners.push_back(self.depth_width - self.width - 20)
        corners.push_back(self.cy - (self.height/2))
        return corners

    def getDepth(self, depthImage, width, height, corners, indices):
        """
        For a 640x480 image, we consider pixels only around
        Left: 20:20+50, 215:265
        Center: 300:340, 215:265
        Right: 580:620, 215:265
        """
        Depthsum = 0
        counter = 0
        for y in range(corners[indices[1]], corners[indices[1]] + height):
            for x in range(corners[indices[0]], corners[indices[0]]+width):
                depth_val = depthImage[x,y]
                if(depth_val>0):
                    counter += 1
                    Depthsum += depth_val
        if counter==0:
            return 0
        return 1000*DepthSum/counter

    def getMedianDepth(self, depthImage, width, height):
        """ 
        For each row, find the median depth around the row center
        And compare from each row and find maximum 
        TODO: Shouldn't it be minimum
        """
        cy = (height/2)
        medianDepth = 0
        for x in range(width):
            depthRow = []
            for y in range(-5, 5):
                depthVal = depthImage[x,cy+y]
                depthRow.append(depthVal)
            depthRow.sort()
            if(medianDepth<depthRow[5]):
                medianDepth = depthRow[5]

        return medianDepth





    def depth_callback(self, data):
        depth_left = getLeftDepth(data.data, self.width_dim_LR, self.height_dim_LR, self.corners, [0,1])
        depth_center = getCenterDepth(data.data, self.width_dim, self.height_dim, self.corners, [2,3]) 
        depth_right = getRightDepth(data.data, self.width_dim_LR, self.height_dim_LR, self.corners,[4,5])
        depth_median = getMedianDepth(data.data, self.depth_with, self.depth_height)
        depthInfo = Float32MultiArray(data=[depth_left, depth_center, depth_right, depth_median])
        self.depthPub.publish(depthInfo)

    def loop(self):
        rospy.logwarn("Starting Loop...")
        rospy.spin()

if __name__=='__main__':
    """
    depth_height = 480
    depth_width = 640
    width_dim = 40
    height_dim = 50
    width_dim_LR = 50
    height_dim_LR = 50
    cx = depth_width/2
    cy = depth_height/2

    corners = getCorners(width_dim, height_dim, cx, cy, depth_width)
    rospy.init_node('DepthDistance')
    rospy.Subscriber("/camera/depth", Image, depth_callback)
    rospy.spin()
    """
    depthinfo = DepthSubscriber()
    depthinfo.loop()
