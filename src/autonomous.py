#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist
import cv2.aruco as aruco

ROTATING_SPEED = 1
LINEAR_SPEED = 0.1
THRESHOLD_AREA_MIN = 7000 #3800 
THRESHOLD_AREA_MAX = 60000 #4000 

class CorridotNav(object):
    def __init__(self, target: int):
        rospy.init_node('aruco_follow', anonymous=True)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_callback)
        self.bridge_object = CvBridge()
        self.target = target
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)


        self.left_grid = 640/3 
        self.right_grid = 2*self.left_grid
        self.active = True 


    def camera_callback(self,data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            
            # aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
            # parameters = aruco.DetectorParameters_create()
            # corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)

            marker_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
            param_markers = aruco.DetectorParameters()
            detector = aruco.ArucoDetector(marker_dict, param_markers)
            corners, ids, reject = detector.detectMarkers(cv_image)


            # print(corners, ids)


            if ids is not None: 
                for id, corner in zip(ids, corners):
                    aruco.drawDetectedMarkers(cv_image, corners, ids)

                    if id == self.target: 
                        left_top, right_top, right_bottom, left_bottom  = corner[0]
                        center = (left_top + right_top + right_bottom + left_bottom)/4
                        center_x = center[0]
                        center_y = center[1]


                        area = cv2.contourArea(corner[0])
                        print("AREA: ", area)
                        
                        print(center_x, center_y)
                        print(self.left_grid)

                        if center_x <= self.left_grid:
                            print("STATE LEFT")
                            self.move_left()
                        elif center_x >= self.right_grid:
                            print("STATE RIGHT")
                            self.move_right()
                        else:
                            if not self.reached(area): 
                                self.move_forward()
                            else:
                                rospy.loginfo(f"LAB REACHED")
                                self.stop()
                                self.active = False 
                    else:
                        if self.active:
                            self.move_forward()
            else:
                if self.active:
                    self.move_forward()
                        
        except CvBridgeError as e:
            print(e)
        
        cv2.imshow('image',cv_image)
        cv2.waitKey(1)
    
    def reached(self, area):
        if area >= THRESHOLD_AREA_MIN and area <= THRESHOLD_AREA_MAX:
            return True 
        else:
            return False 
    
    def stop(self):
        rospy.loginfo("CMD_VEL: ROVER STOPPED")
        data = Twist()
        data.angular.z = 0
        data.linear.x = 0 
        self.pub.publish(data)

    def move_forward(self):
        rospy.loginfo("CMD_VEL: ROVER MOVING FORWARD")
        data = Twist()
        data.linear.x = LINEAR_SPEED
        self.pub.publish(data)

    def move_right(self):
        rospy.loginfo("CMD_VEL: ROVER MOVING RIGHT")
        data = Twist()
        data.angular.z = -ROTATING_SPEED
        self.pub.publish(data)
    
    def move_left(self):
        rospy.loginfo("CMD_VEL: ROVER MOVING LEFT")
        data = Twist()
        data.angular.z =  ROTATING_SPEED
        self.pub.publish(data)
    

    

if __name__ == '__main__':
    try:
        showing_image_object = CorridotNav(5)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()

