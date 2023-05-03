import rospy

from cv_bridge import CvBridge, CvBridgeError
import cv2
import imutils
from sensor_msgs.msg import Image, Bool
from state_machine.src.topics import RAW_IMAGE_TOPIC, BALL_TOPIC

class BallDetector:
    def __init__(self) -> None:
        rospy.Subscriber(RAW_IMAGE_TOPIC, Image, self.ball_detector_call_back)
        self.pub = rospy.Publisher(BALL_TOPIC, Bool, queue_size = 50)
        self.is_ball = False


    def ball_detector_call_back(self, data):
        is_ball_updated = self.ball_detected(data)
        if self.is_ball != is_ball_updated:
            self.pub.publish(is_ball_updated)
            self.is_ball = is_ball_updated


    def ball_detected(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            frame = cv2.resize(frame, (320, 240))
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            cyan_lower = (80, 110, 120)
            cyan_upper = (180, 255, 255)

            mask = cv2.inRange(hsv, cyan_lower, cyan_upper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            
            if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                if radius > 10:
                    return True
                else:
                    return False

        except CvBridgeError as e:
            rospy.logerr(str(e))
            return False


if __name__ == '__main__':
    rospy.init_node('ball_detection', log_level = rospy.INFO, anonymous=True)
    BallDetector()
    rospy.spin()