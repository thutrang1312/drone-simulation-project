#!/usr/bin/env python3
"""
Node điều khiển drone đơn giản
Publish velocity commands đến drone
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class DroneController:
    def __init__(self):
        # Khởi tạo node
        rospy.init_node('drone_controller', anonymous=True)
        
        # Publisher cho velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Publisher cho takeoff/land (nếu dùng)
        self.takeoff_pub = rospy.Publisher('/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/land', Empty, queue_size=1)
        
        # Rate
        self.rate = rospy.Rate(10)  # 10Hz
        
        rospy.loginfo("Drone Controller initialized")
    
    def takeoff(self):
        """Cất cánh drone"""
        rospy.loginfo("Taking off...")
        msg = Twist()
        msg.linear.z = 0.5  # Bay lên với vận tốc 0.5 m/s
        
        # Publish trong 3 giây
        for i in range(30):
            self.cmd_vel_pub.publish(msg)
            self.rate.sleep()
        
        rospy.loginfo("Takeoff complete")
    
    def hover(self, duration=5):
        """Bay lơ lửng tại chỗ"""
        rospy.loginfo(f"Hovering for {duration} seconds...")
        msg = Twist()
        # Tất cả velocities = 0
        
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time:
            self.cmd_vel_pub.publish(msg)
            self.rate.sleep()
        
        rospy.loginfo("Hover complete")
    
    def move_forward(self, speed=0.3, duration=3):
        """Di chuyển về phía trước"""
        rospy.loginfo(f"Moving forward at {speed} m/s...")
        msg = Twist()
        msg.linear.x = speed
        
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time:
            self.cmd_vel_pub.publish(msg)
            self.rate.sleep()
        
        rospy.loginfo("Forward movement complete")
    
    def rotate(self, angular_speed=0.5, duration=3):
        """Xoay tại chỗ"""
        rospy.loginfo(f"Rotating at {angular_speed} rad/s...")
        msg = Twist()
        msg.angular.z = angular_speed
        
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time:
            self.cmd_vel_pub.publish(msg)
            self.rate.sleep()
        
        rospy.loginfo("Rotation complete")
    
    def land(self):
        """Hạ cánh drone"""
        rospy.loginfo("Landing...")
        msg = Twist()
        msg.linear.z = -0.3  # Hạ xuống với vận tốc -0.3 m/s
        
        # Publish trong 5 giây
        for i in range(50):
            self.cmd_vel_pub.publish(msg)
            self.rate.sleep()
        
        rospy.loginfo("Landing complete")
    
    def execute_mission(self):
        """Thực hiện nhiệm vụ bay tự động"""
        rospy.sleep(2)  # Đợi hệ thống sẵn sàng
        
        # Cất cánh
        self.takeoff()
        
        # Bay lơ lửng
        self.hover(duration=2)
        
        # Di chuyển về phía trước
        self.move_forward(speed=0.5, duration=4)
        
        # Bay lơ lửng
        self.hover(duration=2)
        
        # Xoay 180 độ
        self.rotate(angular_speed=0.5, duration=6)
        
        # Bay lơ lửng
        self.hover(duration=2)
        
        # Quay về
        self.move_forward(speed=0.5, duration=4)
        
        # Hạ cánh
        self.land()
        
        rospy.loginfo("Mission complete!")

def main():
    try:
        controller = DroneController()
        
        # Chờ user input
        rospy.loginfo("Press Enter to start mission...")
        input()
        
        # Thực hiện nhiệm vụ
        controller.execute_mission()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
