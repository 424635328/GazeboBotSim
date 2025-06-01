#!/usr/bin/env python
# -*- coding: utf-8 -*- # 声明UTF-8编码以支持中文注释

import rospy
from geometry_msgs.msg import Twist, Point # Twist用于速度指令, Point用于坐标点
from nav_msgs.msg import Odometry          # Odometry用于里程计数据
from tf.transformations import euler_from_quaternion # 用于从四元数转换到欧拉角
import math
import time

class RectanglePatrolClosedLoop:
    def __init__(self):
        rospy.init_node('rectangle_patrol_closed_loop', anonymous=False) # 初始化ROS节点

        # ROS 发布者和订阅者
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # 发布速度指令到 /cmd_vel
        rospy.Subscriber('/odom', Odometry, self.odom_callback) # 订阅里程计数据从 /odom

        # 机器人状态变量
        self.current_pose = Point() # 当前位置 (x, y, z), z通常为0
        self.current_yaw = 0.0      # 当前朝向 (偏航角, 弧度)
        self.odom_received = False  # 里程计数据是否已接收的标志

        # 控制参数
        self.linear_speed = 0.15  # 直线速度 (米/秒)
        self.angular_speed = 0.4  # 角速度 (弧度/秒)
        self.goal_tolerance_linear = 0.05  # 到达目标的线性容忍度 (米)
        self.goal_tolerance_angular = math.radians(5) # 到达目标的角度容忍度 (5度转为弧度)

        # 比例控制器增益 (简单的P控制器)
        self.kp_angular = 0.8 # 移动时调整朝向的角度误差增益
        self.kp_turn = 1.0    # 原地转弯时的角度误差增益

        # 矩形参数
        self.side_length = 1.0 # 矩形边长 (米)
        self.vertices = []     # 存储矩形顶点的列表
        self.current_vertex_index = 0 # 当前目标顶点的索引

        # 状态机的状态定义
        self.STATE_IDLE = 0               # 空闲状态
        self.STATE_MOVING_TO_VERTEX = 1   # 向顶点移动状态
        self.STATE_TURNING_AT_VERTEX = 2  # 在顶点转弯状态
        self.STATE_FINISHED = 3           # 完成状态
        self.current_state = self.STATE_IDLE # 初始化当前状态为空闲

        self.rate = rospy.Rate(20) # 控制循环的频率 (20 Hz)

    def odom_callback(self, msg):
        """里程计回调函数，当收到/odom消息时被调用"""
        self.current_pose = msg.pose.pose.position # 更新当前位置
        orientation_q = msg.pose.pose.orientation # 获取姿态的四元数
        # 将四元数转换为欧拉角 (roll, pitch, yaw)
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.current_yaw) = euler_from_quaternion(orientation_list)
        self.odom_received = True # 标记已收到里程计数据
        # rospy.loginfo_throttle(1.0, "里程计: x=%.2f, y=%.2f, yaw=%.2f度" % (self.current_pose.x, self.current_pose.y, math.degrees(self.current_yaw)))


    def calculate_vertices(self):
        """计算矩形的顶点坐标"""
        # 假设机器人从 (0,0) 开始，朝向X轴正方向 (yaw=0)
        # 顶点 0: (0,0) - 起始点, 不是最初移动的目标
        # 顶点 1: (边长, 0)
        # 顶点 2: (边长, 边长)
        # 顶点 3: (0, 边长)
        # 顶点 4 (回到原点): (0,0)

        self.vertices = [
            Point(0.0, 0.0, 0.0), # 起始姿态参考点
            Point(self.side_length, 0.0, 0.0),
            Point(self.side_length, self.side_length, 0.0),
            Point(0.0, self.side_length, 0.0),
            Point(0.0, 0.0, 0.0) # 回到原点
        ]
        rospy.loginfo("计算出的矩形顶点:")
        for i, v in enumerate(self.vertices):
            rospy.loginfo("顶点 %d: (%.2f, %.2f)" % (i, v.x, v.y))


    def get_target_yaw(self, target_point):
        """计算从当前位置指向目标点的绝对角度 (偏航角)"""
        return math.atan2(target_point.y - self.current_pose.y, target_point.x - self.current_pose.x)

    def normalize_angle(self, angle):
        """将角度归一化到 [-pi, pi] 范围内"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stop_robot(self):
        """发送停止指令给机器人"""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

    def run_patrol(self):
        """主控制循环，执行巡逻逻辑"""
        rospy.loginfo("等待里程计数据...")
        while not self.odom_received and not rospy.is_shutdown(): # 等待直到收到第一帧里程计数据
            self.rate.sleep()
        
        if rospy.is_shutdown(): return # 如果ROS关闭，则退出

        self.calculate_vertices() # 计算矩形顶点
        self.current_state = self.STATE_MOVING_TO_VERTEX # 初始状态：向第一个顶点移动
        self.current_vertex_index = 1 # 目标是顶点列表中的第一个实际目标点 (vertices[1])

        rospy.loginfo("开始基于闭环控制的矩形巡逻。")

        while not rospy.is_shutdown() and self.current_state != self.STATE_FINISHED:
            if not self.odom_received: # 安全检查：如果中途丢失里程计数据
                rospy.logwarn_throttle(1.0, "未收到里程计数据，暂停运动。")
                self.stop_robot()
                self.rate.sleep()
                continue

            target_vertex = self.vertices[self.current_vertex_index] # 获取当前目标顶点
            twist_msg = Twist() # 创建速度指令消息

            if self.current_state == self.STATE_MOVING_TO_VERTEX: # 如果当前是“向顶点移动”状态
                rospy.loginfo_throttle(1.0, "状态: 向顶点 %d (%.2f, %.2f) 移动" % (self.current_vertex_index, target_vertex.x, target_vertex.y))
                
                # 计算到目标顶点的距离
                distance_to_target = math.sqrt((target_vertex.x - self.current_pose.x)**2 + (target_vertex.y - self.current_pose.y)**2)
                
                # 如果已到达目标顶点 (在容忍度范围内)
                if distance_to_target < self.goal_tolerance_linear:
                    rospy.loginfo("已到达顶点 %d." % self.current_vertex_index)
                    self.stop_robot()
                    time.sleep(0.5) # 在顶点处短暂停留
                    self.current_state = self.STATE_TURNING_AT_VERTEX # 切换到“在顶点转弯”状态
                    if self.current_vertex_index >= len(self.vertices) -1: # 如果已到达最终目标 (回到原点)
                        self.current_state = self.STATE_FINISHED # 切换到“完成”状态
                    continue # 进入下一次循环处理新状态

                # --- 使用P控制器调整朝向 ---
                desired_yaw = self.get_target_yaw(target_vertex) # 计算期望的朝向角
                error_yaw = self.normalize_angle(desired_yaw - self.current_yaw) # 计算角度误差
                
                # 如果方向偏差较大，则优先调整方向
                if abs(error_yaw) > self.goal_tolerance_angular * 0.5 : 
                    twist_msg.linear.x = 0.0 # 停止直线运动以进行旋转
                    twist_msg.angular.z = self.kp_angular * error_yaw # P控制计算角速度
                    # 限制最大角速度
                    if abs(twist_msg.angular.z) > self.angular_speed: 
                        twist_msg.angular.z = math.copysign(self.angular_speed, twist_msg.angular.z)
                    # 保证最小角速度以克服静摩擦 (如果误差仍然存在)
                    elif abs(twist_msg.angular.z) < 0.05 and abs(error_yaw) > self.goal_tolerance_angular * 0.1: 
                        twist_msg.angular.z = math.copysign(0.05, twist_msg.angular.z)
                else: # 如果方向基本对准，则向前移动
                    twist_msg.linear.x = self.linear_speed
                    # 前进时进行轻微的角度校正
                    twist_msg.angular.z = self.kp_angular * error_yaw * 0.3 # 使用较小的增益进行校正
                    # 限制前进时的校正角速度
                    if abs(twist_msg.angular.z) > self.angular_speed * 0.3: 
                        twist_msg.angular.z = math.copysign(self.angular_speed*0.3, twist_msg.angular.z)

                # 如果接近目标，则减速 (防止过冲)
                if distance_to_target < self.linear_speed * 1.0: # 如果距离小于1秒的行程
                    twist_msg.linear.x = max(0.05, distance_to_target / 2.0) # 速度与距离成正比，最小速度0.05

                self.cmd_vel_pub.publish(twist_msg) # 发布速度指令

            elif self.current_state == self.STATE_TURNING_AT_VERTEX: # 如果当前是“在顶点转弯”状态
                # 计算下一个移动段的期望朝向
                # 我们当前在 self.vertices[self.current_vertex_index]，需要转向以朝向 self.vertices[self.current_vertex_index + 1]
                
                next_target_point_for_yaw_calc_index = self.current_vertex_index + 1
                # 检查是否已经是最后一个顶点，理论上STATE_FINISHED会处理，但作为安全检查
                if next_target_point_for_yaw_calc_index >= len(self.vertices): 
                    rospy.loginfo("转弯逻辑错误或已完成巡逻。")
                    self.current_state = self.STATE_FINISHED
                    self.stop_robot()
                    continue
                
                next_target_point_for_yaw_calc = self.vertices[next_target_point_for_yaw_calc_index]
                
                # 期望的朝向是当前位置指向下一个目标顶点
                desired_yaw_for_next_segment = self.get_target_yaw(next_target_point_for_yaw_calc)

                rospy.loginfo_throttle(1.0, "状态: 在顶点 %d 转弯. 当前Yaw: %.2f度, 下一段期望Yaw: %.2f度" % \
                    (self.current_vertex_index, math.degrees(self.current_yaw), math.degrees(desired_yaw_for_next_segment)))

                error_yaw = self.normalize_angle(desired_yaw_for_next_segment - self.current_yaw) # 计算角度误差

                # 如果已转到目标朝向 (在容忍度范围内)
                if abs(error_yaw) < self.goal_tolerance_angular:
                    rospy.loginfo("在顶点 %d 完成转弯." % self.current_vertex_index)
                    self.stop_robot()
                    time.sleep(0.5) # 短暂停留
                    # 准备移动到 *下一个* 顶点
                    self.current_vertex_index += 1 # 更新目标顶点索引
                    if self.current_vertex_index >= len(self.vertices): # 如果所有顶点都已访问
                        self.current_state = self.STATE_FINISHED # 切换到“完成”状态
                    else:
                        self.current_state = self.STATE_MOVING_TO_VERTEX # 否则，切换到“向顶点移动”状态
                    continue # 进入下一次循环处理新状态
                
                twist_msg.linear.x = 0.0 # 原地转弯，直线速度为0
                twist_msg.angular.z = self.kp_turn * error_yaw # P控制计算角速度
                # 限制最大角速度
                if abs(twist_msg.angular.z) > self.angular_speed:
                    twist_msg.angular.z = math.copysign(self.angular_speed, twist_msg.angular.z)
                # 保证最小角速度以克服静摩擦 (如果误差仍然存在)
                elif abs(twist_msg.angular.z) < 0.1 and abs(error_yaw) > self.goal_tolerance_angular * 0.2: 
                    twist_msg.angular.z = math.copysign(0.1, twist_msg.angular.z)

                self.cmd_vel_pub.publish(twist_msg) # 发布速度指令

            self.rate.sleep() # 按设定频率暂停，控制循环速率

        rospy.loginfo("矩形巡逻完成。")
        self.stop_robot() # 确保机器人最终停止

if __name__ == '__main__':
    try:
        patroller = RectanglePatrolClosedLoop() # 创建巡逻对象
        patroller.run_patrol() # 开始巡逻
    except rospy.ROSInterruptException: # 处理Ctrl+C中断
        rospy.loginfo("巡逻被中断。")
    except Exception as e: # 处理其他异常
        rospy.logerr("发生错误: {}".format(e))
        import traceback
        traceback.print_exc() # 打印详细的错误堆栈信息
    finally:
        # 确保在脚本异常退出时机器人也能停止
        if 'patroller' in locals() and patroller: # 检查patroller对象是否存在
            patroller.stop_robot()