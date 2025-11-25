#!/usr/bin/env python3
"""
Node đọc dữ liệu từ sensors (IMU, Camera)
"""

import rospy
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge
import numpy as np

class SensorReader:
    def __init__(self):
        rospy.init_node('sensor_reader', anonymous=True)
        
        # IMU data
        self.imu_data = None
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        
        # Pose data
        self.pose_data = None
        rospy.Subscriber('/ground_truth/state', Odometry, self.pose_callback)
        
        # Camera data
        self.bridge = CvBridge()
        self.latest_image = None
        rospy.Subscriber('/quadrotor/camera/image_raw', Image, self.image_callback)
        
        rospy.loginfo("Sensor Reader initialized")
        
        # Timer để in dữ liệu
        rospy.Timer(rospy.Duration(1.0), self.print_data)
    
    def imu_callback(self, msg):
        """Callback cho IMU data"""
        self.imu_data = msg
    
    def pose_callback(self, msg):
        """Callback cho Pose data"""
        self.pose_data = msg
    
    def image_callback(self, msg):
        """Callback cho Camera image"""
        try:
            # Convert ROS Image sang OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
            
            # Hiển thị image (optional)
            cv2.imshow("Drone Camera", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
    
    def print_data(self, event):
        """In dữ liệu sensor định kỳ"""
        rospy.loginfo("=" * 50)
        
        # IMU data
        if self.imu_data:
            rospy.loginfo("IMU Data:")
            rospy.loginfo(f"  Linear Accel: x={self.imu_data.linear_acceleration.x:.2f}, "
                         f"y={self.imu_data.linear_acceleration.y:.2f}, "
                         f"z={self.imu_data.linear_acceleration.z:.2f}")
            rospy.loginfo(f"  Angular Vel: x={self.imu_data.angular_velocity.x:.2f}, "
                         f"y={self.imu_data.angular_velocity.y:.2f}, "
                         f"z={self.imu_data.angular_velocity.z:.2f}")
        
        # Pose data
        if self.pose_data:
            pos = self.pose_data.pose.pose.position
            rospy.loginfo("Pose Data:")
            rospy.loginfo(f"  Position: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}")
        
        # Image data
        if self.latest_image is not None:
            rospy.loginfo(f"Camera: Image size {self.latest_image.shape}")
    
    def spin(self):
        """Keep node running"""
        rospy.spin()
        cv2.destroyAllWindows()

def main():
    try:
        reader = SensorReader()
        reader.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
