#!/usr/bin/env python3
"""
Node điều khiển drone bằng keyboard
"""

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class KeyboardControl:
    def __init__(self):
        rospy.init_node('keyboard_control', anonymous=True)
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10)
        
        # Thông số điều khiển
        self.linear_speed = 0.5
        self.angular_speed = 0.5
        self.vertical_speed = 0.3
        
        self.msg = Twist()
        
        rospy.loginfo("Keyboard Control initialized")
        self.print_instructions()
    
    def print_instructions(self):
        """In hướng dẫn sử dụng"""
        print("\n" + "="*50)
        print("KEYBOARD CONTROL")
        print("="*50)
        print("W/S: Forward/Backward")
        print("A/D: Left/Right")
        print("Q/E: Rotate Left/Right")
        print("R/F: Up/Down")
        print("SPACE: Stop")
        print("X: Exit")
        print("="*50 + "\n")
    
    def get_key(self):
        """Đọc key từ keyboard"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def run(self):
        """Main loop"""
        while not rospy.is_shutdown():
            key = self.get_key()
            
            # Reset velocities
            self.msg = Twist()
            
            # Process key
            if key == 'w':
                self.msg.linear.x = self.linear_speed
                rospy.loginfo("Moving forward")
            elif key == 's':
                self.msg.linear.x = -self.linear_speed
                rospy.loginfo("Moving backward")
            elif key == 'a':
                self.msg.linear.y = self.linear_speed
                rospy.loginfo("Moving left")
            elif key == 'd':
                self.msg.linear.y = -self.linear_speed
                rospy.loginfo("Moving right")
            elif key == 'r':
                self.msg.linear.z = self.vertical_speed
                rospy.loginfo("Moving up")
            elif key == 'f':
                self.msg.linear.z = -self.vertical_speed
                rospy.loginfo("Moving down")
            elif key == 'q':
                self.msg.angular.z = self.angular_speed
                rospy.loginfo("Rotating left")
            elif key == 'e':
                self.msg.angular.z = -self.angular_speed
                rospy.loginfo("Rotating right")
            elif key == ' ':
                rospy.loginfo("Stop")
            elif key == 'x':
                rospy.loginfo("Exiting...")
                break
            else:
                continue
            
            # Publish command
            self.cmd_vel_pub.publish(self.msg)
            self.rate.sleep()

def main():
    try:
        controller = KeyboardControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {e}")

if __name__ == '__main__':
    main()
