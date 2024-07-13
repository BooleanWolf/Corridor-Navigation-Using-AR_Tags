#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def image_publisher():
    # Initialize the ROS node
    rospy.init_node('webcam_image_publisher', anonymous=True)
    # Create a publisher object
    image_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
    # Create a CvBridge object
    bridge = CvBridge()
    
    # Open a connection to the webcam (0 is the default camera)
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        rospy.logerr("Failed to open webcam.")
        return
    
    rate = rospy.Rate(10)  # Set the publishing rate to 10 Hz
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        
        if not ret:
            rospy.logerr("Failed to capture image.")
            continue
        
        try:
            # Convert the OpenCV image to a ROS Image message
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            # Publish the image message
            image_pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        
        rate.sleep()
    
    # Release the webcam when done
    cap.release()

if __name__ == '__main__':
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass
