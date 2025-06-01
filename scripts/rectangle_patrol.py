#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import time

def move_robot():
    rospy.init_node('rectangle_patrol', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    move_cmd = Twist()

    # 矩形参数
    side_length = 1.0  # 米
    turn_angle_rad = 1.5708 # 90度，约等于 PI/2
    
    linear_speed = 0.2  # 米/秒
    angular_speed = 0.5 # 弧度/秒

    rospy.loginfo("Starting rectangular patrol...")

    for i in range(4): # 四条边
        # --- 前进 ---
        rospy.loginfo("Moving forward...")
        move_cmd.linear.x = linear_speed
        move_cmd.angular.z = 0.0
        
        start_time = rospy.get_time()
        duration_forward = side_length / linear_speed
        
        while (rospy.get_time() - start_time) < duration_forward:
            if rospy.is_shutdown():
                break
            pub.publish(move_cmd)
            rate.sleep()
        
        # --- 停止 ---
        move_cmd.linear.x = 0.0
        pub.publish(move_cmd)
        rospy.loginfo("Stopped after forward movement.")
        time.sleep(1) # 短暂暂停

        if i < 3: # 最后一次前进后不需要转弯
            # --- 左转90度 ---
            rospy.loginfo("Turning left...")
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = angular_speed # 左转为正角速度
            
            start_time = rospy.get_time()
            duration_turn = turn_angle_rad / angular_speed
            
            while (rospy.get_time() - start_time) < duration_turn:
                if rospy.is_shutdown():
                    break
                pub.publish(move_cmd)
                rate.sleep()

            # --- 停止旋转 ---
            move_cmd.angular.z = 0.0
            pub.publish(move_cmd)
            rospy.loginfo("Stopped after turning.")
            time.sleep(1) # 短暂暂停
    
    rospy.loginfo("Rectangular patrol finished.")

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Patrol interrupted.")
    except Exception as e:
        rospy.logerr("An error occurred: {}".format(e))