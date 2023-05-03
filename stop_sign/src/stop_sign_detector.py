import rospy

from std_msgs.msg import *
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

from state_machine.src.topics import RAW_IMAGE_TOPIC

class StopSignDetector:
    def __init__(self) -> None:
        rospy.Subscriber(RAW_IMAGE_TOPIC, Image, self.stop_sign_callback)
        self.pub = rospy.Publisher('is_stop_sign', Bool, queue_size=30)

        classifier_file = "/home/odroid/Cruiser/src/stop_sign/src/stopsign_classifier.xml"
        self.classifier = cv2.CascadeClassifier(classifier_file)
        self.bridge = CvBridge()
    
    def stop_sign_callback(self, data):
        is_stop_sign = self.stop_sign_detected(data)
        self.pub.publish(is_stop_sign)
    
    def stop_sign_detected(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
            cv_image = cv2.resize(cv_image, (253,200))
        except CvBridgeError as e:
            rospy.logerr(e)
        stop_signs = self.classifier.detectMultiScale(cv_image, 1.02, 10)
        
        if len(stop_signs) != 0:
            return True
        return False

if __name__ == "__main__":
    rospy.init_node('stop_sign_detector')
    stop_sign_detector = StopSignDetector()
    stop_sign_detector.run()
