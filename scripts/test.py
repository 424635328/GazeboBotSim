#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import time
import numpy as np # 用于插值

# ==============================================================================
# 用户可配置参数区域
# ==============================================================================
TARGET_WAYPOINTS = [
    (5.0, 0.0), (5.0, 2.0), (2.0, 5.0), (0.0, 5.0), (0.0, 0.0)
]

# --- 速度与容忍度 ---
DEFAULT_LINEAR_SPEED_MAX = 1.0
DEFAULT_ANGULAR_SPEED_MAX = 1.2
GOAL_TOLERANCE_LINEAR = 0.03
GOAL_TOLERANCE_ANGULAR_DEG = 1.5

# --- PID 控制器基础增益 (将作为增益调度的基准或一部分) ---
# 移动时的PID
BASE_PID_ANGULAR_MOVE_KP = 1.5 # 基础P值
BASE_PID_ANGULAR_MOVE_KD = 0.2  # 基础D值
BASE_PID_ANGULAR_MOVE_KI = 0.005
# 原地转向时的PID
BASE_PID_ANGULAR_TURN_KP = 3.0
BASE_PID_ANGULAR_TURN_KD = 0.4
BASE_PID_ANGULAR_TURN_KI = 0.01

# --- 增益调度参数 ---
# 角度误差较大时，P增益的乘数 (使其更快速响应大误差)
LARGE_ERROR_KP_MULTIPLIER = 1.5
LARGE_ERROR_THRESHOLD_DEG = 30.0 # 超过此角度误差，应用乘数
# 线速度较高时，角度P增益的除数 (降低P以防高速震荡)
HIGH_SPEED_KP_DIVISOR = 1.8
HIGH_SPEED_THRESHOLD = 0.5 # 线速度超过此值，应用除数
# 线速度较高时，角度D增益的乘数 (增加D以抑制高速震荡)
HIGH_SPEED_KD_MULTIPLIER = 1.3


# PID积分项抗饱和参数
INTEGRAL_MAX_ANGULAR_CONTRIBUTION = 0.2
INTEGRAL_RESET_ERROR_THRESHOLD_DEG = 10

# --- 减速与末端控制参数 ---
DECELERATION_START_TIME_FACTOR = 1.0
K_DIST_DECELERATION = 3.0 # S型减速陡峭度，值越大末端减速越急

APPROACHING_DISTANCE_THRESHOLD = 0.35
FINAL_APPROACH_LINEAR_SPEED_CAP = 0.05
FINAL_APPROACH_ANGULAR_SPEED_CAP = 0.30

FINAL_STOP_LINEAR_VEL_THRESHOLD = 0.015
FINAL_STOP_ANGULAR_VEL_THRESHOLD_DEG = 1.0

# --- 其他行为参数 ---
PAUSE_AT_WAYPOINT_DURATION = 0.2
FINAL_TURN_TO_INITIAL_YAW = True
INITIAL_YAW_TARGET_DEG = 0.0

ROS_RATE_HZ = 20
DT = 1.0 / ROS_RATE_HZ
# ==============================================================================

class AdvancedWaypointNavigation:
    def __init__(self):
        rospy.init_node('advanced_waypoint_navigation_node', anonymous=False)

        # --- 加载基础参数 ---
        self.target_waypoints_coords = TARGET_WAYPOINTS
        self.linear_speed_max = rospy.get_param("~linear_speed_max", DEFAULT_LINEAR_SPEED_MAX)
        self.angular_speed_max = rospy.get_param("~angular_speed_max", DEFAULT_ANGULAR_SPEED_MAX)
        self.goal_tolerance_linear = rospy.get_param("~goal_tolerance_linear", GOAL_TOLERANCE_LINEAR)
        self.goal_tolerance_angular = math.radians(rospy.get_param("~goal_tolerance_angular_deg", GOAL_TOLERANCE_ANGULAR_DEG))
        
        # 基础PID增益
        self.base_pid_move_kp = rospy.get_param("~base_pid_angular_move_kp", BASE_PID_ANGULAR_MOVE_KP)
        self.base_pid_move_kd = rospy.get_param("~base_pid_angular_move_kd", BASE_PID_ANGULAR_MOVE_KD)
        self.base_pid_move_ki = rospy.get_param("~base_pid_angular_move_ki", BASE_PID_ANGULAR_MOVE_KI)
        self.base_pid_turn_kp = rospy.get_param("~base_pid_angular_turn_kp", BASE_PID_ANGULAR_TURN_KP)
        self.base_pid_turn_kd = rospy.get_param("~base_pid_angular_turn_kd", BASE_PID_ANGULAR_TURN_KD)
        self.base_pid_turn_ki = rospy.get_param("~base_pid_angular_turn_ki", BASE_PID_ANGULAR_TURN_KI)

        # 增益调度参数
        self.large_error_kp_multiplier = rospy.get_param("~large_error_kp_multiplier", LARGE_ERROR_KP_MULTIPLIER)
        self.large_error_threshold_rad = math.radians(rospy.get_param("~large_error_threshold_deg", LARGE_ERROR_THRESHOLD_DEG))
        self.high_speed_kp_divisor = rospy.get_param("~high_speed_kp_divisor", HIGH_SPEED_KP_DIVISOR)
        self.high_speed_threshold = rospy.get_param("~high_speed_threshold", HIGH_SPEED_THRESHOLD)
        self.high_speed_kd_multiplier = rospy.get_param("~high_speed_kd_multiplier", HIGH_SPEED_KD_MULTIPLIER)


        self.integral_max_angular_contribution = rospy.get_param("~integral_max_angular_contribution", INTEGRAL_MAX_ANGULAR_CONTRIBUTION)
        self.integral_reset_error_threshold_rad = math.radians(rospy.get_param("~integral_reset_error_threshold_deg", INTEGRAL_RESET_ERROR_THRESHOLD_DEG))

        self.deceleration_start_time_factor = rospy.get_param("~deceleration_start_time_factor", DECELERATION_START_TIME_FACTOR)
        self.k_dist_deceleration = rospy.get_param("~k_dist_deceleration", K_DIST_DECELERATION)
        self.approaching_distance_threshold = rospy.get_param("~approaching_distance_threshold", APPROACHING_DISTANCE_THRESHOLD)
        self.final_approach_linear_speed_cap = rospy.get_param("~final_approach_linear_speed_cap", FINAL_APPROACH_LINEAR_SPEED_CAP)
        self.final_approach_angular_speed_cap = rospy.get_param("~final_approach_angular_speed_cap", FINAL_APPROACH_ANGULAR_SPEED_CAP)
        self.final_stop_linear_vel_threshold = rospy.get_param("~final_stop_linear_vel_threshold", FINAL_STOP_LINEAR_VEL_THRESHOLD)
        self.final_stop_angular_vel_threshold = math.radians(rospy.get_param("~final_stop_angular_vel_threshold_deg", FINAL_STOP_ANGULAR_VEL_THRESHOLD_DEG))

        self.pause_duration = rospy.get_param("~pause_at_waypoint_duration", PAUSE_AT_WAYPOINT_DURATION)
        self.final_turn = rospy.get_param("~final_turn_to_initial_yaw", FINAL_TURN_TO_INITIAL_YAW)
        self.initial_yaw_target_rad = math.radians(rospy.get_param("~initial_yaw_target_deg", INITIAL_YAW_TARGET_DEG))

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.current_pose = Point()
        self.current_yaw = 0.0
        self.current_linear_vel = 0.0 # 从odom获取当前线速度
        self.current_angular_vel = 0.0 # 从odom获取当前角速度
        self.odom_received = False
        self.waypoints = []
        self.current_waypoint_index = 0
        self.prev_angular_error_move = 0.0
        self.integral_angular_error_move = 0.0
        self.prev_angular_error_turn = 0.0
        self.integral_angular_error_turn = 0.0

        self.STATE_IDLE = 0
        self.STATE_MOVING_TO_WAYPOINT = 1
        self.STATE_TURNING_FOR_NEXT_WAYPOINT = 2
        self.STATE_PAUSING_AT_WAYPOINT = 3
        self.STATE_FINAL_TURNING = 4
        self.STATE_FINISHED = 5
        self.current_state = self.STATE_IDLE
        self.rate = rospy.Rate(ROS_RATE_HZ)
        self._prepare_waypoints()

    def _prepare_waypoints(self): # Same
        self.waypoints = []
        if not self.target_waypoints_coords: rospy.logwarn("目标航点列表为空！"); return
        for i, coord_tuple in enumerate(self.target_waypoints_coords):
            if len(coord_tuple) >= 2:
                wp = Point(); wp.x = coord_tuple[0]; wp.y = coord_tuple[1]; wp.z = 0
                self.waypoints.append(wp)
                rospy.loginfo("已加载航点 %d: (%.2f, %.2f)" % (i, wp.x, wp.y))
            else: rospy.logwarn("航点 %s 格式不正确，已忽略。" % str(coord_tuple))
        if not self.waypoints: rospy.logwarn("没有有效的航点被加载！")

    def odom_callback(self, msg): # Modified to get current velocities
        self.current_pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.current_yaw) = euler_from_quaternion(orientation_list)
        
        # 获取当前线速度和角速度 (从twist部分)
        self.current_linear_vel = msg.twist.twist.linear.x
        self.current_angular_vel = msg.twist.twist.angular.z
        
        self.odom_received = True

    def get_target_yaw(self, target_point): # Same
        return math.atan2(target_point.y - self.current_pose.y, target_point.x - self.current_pose.x)

    def normalize_angle(self, angle): # Same
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def stop_robot(self): # Same
        twist_msg = Twist()
        self.cmd_vel_pub.publish(twist_msg)

    def get_scheduled_pid_gains(self, base_kp, base_kd, base_ki, current_error_abs, current_linear_velocity_abs):
        """根据当前误差和速度动态调整PID增益"""
        kp_sched = base_kp
        kd_sched = base_kd
        ki_sched = base_ki # KI通常不参与基于速度的调度，除非有特殊需求

        # 1. 基于误差大小调整P增益
        if current_error_abs > self.large_error_threshold_rad:
            kp_sched *= self.large_error_kp_multiplier
            # D项也可以相应调整，例如也乘以一个小于P乘数的因子，或者保持不变
            # kd_sched *= (1 + (self.large_error_kp_multiplier - 1) * 0.5)


        # 2. 基于当前线速度调整P和D增益 (主要用于移动时的角度控制)
        if current_linear_velocity_abs > self.high_speed_threshold:
            kp_sched /= self.high_speed_kp_divisor
            kd_sched *= self.high_speed_kd_multiplier
            # 在高速时，可以考虑减小或禁用I项，防止其导致不稳定
            # ki_sched *= 0.5 # 或者 ki_sched = 0

        # 确保增益不为负或过小
        kp_sched = max(0.1, kp_sched) # P最小为0.1
        kd_sched = max(0.01, kd_sched) # D最小为0.01
        ki_sched = max(0, ki_sched)   # KI最小为0
        
        return kp_sched, kd_sched, ki_sched


    def calculate_angular_pid_output(self, current_error, kp, kd, ki, prev_error_list, integral_error_list, mode="turn"): # Same logic, uses scheduled gains
        p_term = kp * current_error
        derivative_error = (current_error - prev_error_list[0]) / DT 
        d_term = kd * derivative_error
        current_integral_error = integral_error_list[0]
        if ki > 1e-7:
            if abs(current_error) > self.integral_reset_error_threshold_rad or \
               (current_error * prev_error_list[0] < 0 and abs(current_error) > self.goal_tolerance_angular * 0.3) :
                current_integral_error = 0.0 
            current_integral_error += current_error * DT
            max_integral_val = self.integral_max_angular_contribution / (ki + 1e-7)
            current_integral_error = max(min(current_integral_error, max_integral_val), -max_integral_val)
            integral_error_list[0] = current_integral_error
        i_term = ki * current_integral_error
        pid_output = p_term + d_term + i_term
        prev_error_list[0] = current_error
        return pid_output

    def reset_pid_states(self, mode="all"): # Same
        if mode == "all" or mode == "move":
            self.prev_angular_error_move = 0.0
            self.integral_angular_error_move = 0.0
        if mode == "all" or mode == "turn":
            self.prev_angular_error_turn = 0.0
            self.integral_angular_error_turn = 0.0
            
    def run_navigation(self): # Main loop
        rospy.loginfo("等待里程计数据...")
        while not self.odom_received and not rospy.is_shutdown(): self.rate.sleep()
        if rospy.is_shutdown() or not self.waypoints:
            rospy.logwarn("没有航点或ROS关闭，退出。"); return

        self.current_state = self.STATE_MOVING_TO_WAYPOINT
        self.current_waypoint_index = 0
        self.reset_pid_states("all")
        rospy.loginfo("开始高级PID定点巡视。总共 %d 个航点。" % len(self.waypoints))

        while not rospy.is_shutdown() and self.current_state != self.STATE_FINISHED:
            if not self.odom_received:
                rospy.logwarn_throttle(1.0, "未收到里程计数据，暂停。")
                self.stop_robot(); self.rate.sleep(); continue

            if self.current_waypoint_index < len(self.waypoints):
                target_waypoint = self.waypoints[self.current_waypoint_index]
            else:
                if self.final_turn and self.current_state != self.STATE_FINAL_TURNING:
                    rospy.loginfo("所有航点已到达。准备最终转向。")
                    self.current_state = self.STATE_FINAL_TURNING
                    self.reset_pid_states("turn")
                else:
                    self.current_state = self.STATE_FINISHED
                continue

            twist_msg = Twist()

            if self.current_state == self.STATE_MOVING_TO_WAYPOINT:
                distance_to_target = math.sqrt((target_waypoint.x - self.current_pose.x)**2 + \
                                               (target_waypoint.y - self.current_pose.y)**2)
                desired_yaw = self.get_target_yaw(target_waypoint)
                error_yaw = self.normalize_angle(desired_yaw - self.current_yaw)

                # --- 获取调度后的PID增益 (用于移动时的角度控制) ---
                # 使用当前从odom获取的线速度的绝对值
                kp_move_sched, kd_move_sched, ki_move_sched = self.get_scheduled_pid_gains(
                    self.base_pid_move_kp, self.base_pid_move_kd, self.base_pid_move_ki,
                    abs(error_yaw), abs(self.current_linear_vel) # 使用odom的当前速度
                )

                angular_velocity_pid_raw = self.calculate_angular_pid_output(error_yaw, 
                                                                         kp_move_sched, 
                                                                         kd_move_sched, 
                                                                         ki_move_sched,
                                                                         [self.prev_angular_error_move],
                                                                         [self.integral_angular_error_move],
                                                                         mode="move_sched")
                
                # --- 线速度控制逻辑 (更平滑的S型曲线) ---
                # 1. 初始目标线速度 (受最大速度和角度误差影响)
                #    角度误差越大，允许的初始目标线速度越小
                #    使用插值函数可以在不同误差下平滑过渡速度上限
                #    例如: error_yaw_deg = abs(math.degrees(error_yaw))
                #    max_speed_due_to_angle_error = np.interp(error_yaw_deg, [5, 20, 45], [self.linear_speed_max, self.linear_speed_max * 0.6, self.linear_speed_max * 0.1])
                #    current_target_linear_speed = max_speed_due_to_angle_error
                #    简化版：
                angle_error_factor = math.exp(-2.0 * (abs(error_yaw) / math.radians(30.0))**2) # 之前是-3.5/45度
                current_target_linear_speed = self.linear_speed_max * angle_error_factor


                # 2. 根据距离平滑减速 (S型曲线)
                deceleration_start_distance = self.linear_speed_max * self.deceleration_start_time_factor
                if distance_to_target < deceleration_start_distance:
                    # 使用更平滑的减速因子，例如 (1 - cos(pi * x))/2 for x in [0,1]
                    # 或者调整tanh的参数
                    # x_norm = (distance_to_target - self.goal_tolerance_linear * 0.2) / (deceleration_start_distance - self.goal_tolerance_linear * 0.2 + 1e-6)
                    # distance_factor = (1.0 - math.cos(math.pi * max(0, min(1, x_norm)))) / 2.0 # 这个减速非常剧烈在末端
                    # 保持tanh，但调整参数
                    effective_decel_range = deceleration_start_distance - self.goal_tolerance_linear * 0.2
                    normalized_dist_in_decel_range = max(0, (distance_to_target - self.goal_tolerance_linear * 0.2) / (effective_decel_range + 1e-6) )
                    # k_dist_deceleration 越大，在远距离时速度保持越高，近距离时下降越快
                    distance_factor = math.tanh(self.k_dist_deceleration * normalized_dist_in_decel_range**1.2) # **1.2使曲线更陡峭
                    current_target_linear_speed *= distance_factor
                
                final_linear_speed = max(0, current_target_linear_speed)

                # --- 末端精细控制逻辑 ---
                is_approaching_final = (distance_to_target < self.approaching_distance_threshold)
                current_angular_cap = self.angular_speed_max
                
                if is_approaching_final:
                    final_linear_speed = min(final_linear_speed, self.final_approach_linear_speed_cap)
                    current_angular_cap = self.final_approach_angular_speed_cap
                
                twist_msg.angular.z = max(min(angular_velocity_pid_raw, current_angular_cap), -current_angular_cap)
                twist_msg.linear.x = final_linear_speed

                # 4. 最小驱动速度
                min_move_linear_speed = 0.03
                min_move_angular_speed = 0.06
                if not is_approaching_final:
                    if twist_msg.linear.x > 1e-3 and twist_msg.linear.x < min_move_linear_speed and \
                       distance_to_target > self.goal_tolerance_linear * 3.0 and abs(error_yaw) < math.radians(10):
                        twist_msg.linear.x = min_move_linear_speed
                    if abs(error_yaw) > self.goal_tolerance_angular * 1.1 and \
                       abs(twist_msg.angular.z) < min_move_angular_speed and \
                       abs(twist_msg.angular.z) > 1e-4:
                        twist_msg.angular.z = math.copysign(min_move_angular_speed, error_yaw)
                else: 
                    if abs(twist_msg.linear.x) < (self.final_stop_linear_vel_threshold * 0.8): twist_msg.linear.x = 0.0
                    if abs(twist_msg.angular.z) < (self.final_stop_angular_vel_threshold * 0.8): twist_msg.angular.z = 0.0

                rospy.loginfo_throttle(0.2, "MOVE Wpt%d D:%.2fm Y:%.1f E:%.1f ActVx:%.2f OdoVx:%.2f Wz:%.2f OdoWz:%.2f Kp:%.1f Kd:%.1f Appr:%d" % \
                    (self.current_waypoint_index, distance_to_target, math.degrees(self.current_yaw), math.degrees(error_yaw), \
                     twist_msg.linear.x, self.current_linear_vel, twist_msg.angular.z, self.current_angular_vel, \
                     kp_move_sched, kd_move_sched, int(is_approaching_final)))


                if distance_to_target < self.goal_tolerance_linear:
                    # 使用 odom 的速度进行稳定判断会更可靠
                    is_stable_now = (abs(self.current_linear_vel) < self.final_stop_linear_vel_threshold and \
                                     abs(self.current_angular_vel) < self.final_stop_angular_vel_threshold and \
                                     abs(twist_msg.linear.x) < self.final_stop_linear_vel_threshold * 1.2 and # 指令也应该很小
                                     abs(twist_msg.angular.z) < self.final_stop_angular_vel_threshold * 1.2)

                    if is_stable_now:
                        rospy.loginfo("已精确稳定到达航点 %d (D:%.3fm, OdoVx:%.3f, OdoWz:%.3f)." % \
                                      (self.current_waypoint_index, distance_to_target, self.current_linear_vel, self.current_angular_vel))
                        self.stop_robot()
                        self.current_state = self.STATE_PAUSING_AT_WAYPOINT
                        continue
                    else:
                        rospy.loginfo_throttle(0.1, "接近航点 %d 但需稳定. D:%.3fm, CmdVx:%.2f OdoVx:%.2f, CmdWz:%.2f OdoWz:%.2f" % \
                                               (self.current_waypoint_index, distance_to_target, \
                                                twist_msg.linear.x, self.current_linear_vel, \
                                                twist_msg.angular.z, self.current_angular_vel))
            
            elif self.current_state == self.STATE_PAUSING_AT_WAYPOINT:
                self.stop_robot()
                if self.pause_duration > DT : time.sleep(self.pause_duration - DT)
                rospy.loginfo("在航点 %d 暂停完毕。" % self.current_waypoint_index)
                self.reset_pid_states("all")
                if self.current_waypoint_index + 1 < len(self.waypoints):
                    self.current_state = self.STATE_TURNING_FOR_NEXT_WAYPOINT
                else:
                    self.current_state = self.STATE_FINAL_TURNING if self.final_turn else self.STATE_FINISHED
                continue

            elif self.current_state == self.STATE_TURNING_FOR_NEXT_WAYPOINT or \
                 self.current_state == self.STATE_FINAL_TURNING:

                is_final_turn = (self.current_state == self.STATE_FINAL_TURNING)
                current_base_kp = self.base_pid_turn_kp
                current_base_kd = self.base_pid_turn_kd
                current_base_ki = self.base_pid_turn_ki
                
                if is_final_turn:
                    target_yaw_rad = self.initial_yaw_target_rad
                    log_prefix = "FINAL_TURN"
                    # 最终转向时，如果误差大，P可以不那么激进
                    # kp_turn_sched, kd_turn_sched, ki_turn_sched = self.get_scheduled_pid_gains(
                    #     current_base_kp * 0.8, current_base_kd, current_base_ki, # 略微降低基础P
                    #     abs(error_yaw), 0 # 速度为0
                    # )
                else:
                    next_waypoint_idx = self.current_waypoint_index + 1
                    if next_waypoint_idx >= len(self.waypoints): self.current_state = self.STATE_FINISHED; continue
                    next_target_wp = self.waypoints[next_waypoint_idx]
                    target_yaw_rad = self.get_target_yaw(next_target_wp)
                    log_prefix = "TURN Wpt%d" % next_waypoint_idx
                
                error_yaw = self.normalize_angle(target_yaw_rad - self.current_yaw)

                # 获取调度后的转向PID增益 (只基于误差大小，因为速度为0)
                kp_turn_sched, kd_turn_sched, ki_turn_sched = self.get_scheduled_pid_gains(
                    current_base_kp, current_base_kd, current_base_ki,
                    abs(error_yaw), 0 # 转向时线速度为0
                )

                angular_velocity_pid_raw = self.calculate_angular_pid_output(error_yaw,
                                                                         kp_turn_sched,
                                                                         kd_turn_sched,
                                                                         ki_turn_sched,
                                                                         [self.prev_angular_error_turn],
                                                                         [self.integral_angular_error_turn],
                                                                         mode="turn_sched" if not is_final_turn else "final_turn_sched")
                
                current_angular_speed_cap = self.angular_speed_max if not is_final_turn else self.angular_speed_max * 0.85 # 最终转向略慢
                twist_msg.angular.z = max(min(angular_velocity_pid_raw, current_angular_speed_cap), -current_angular_speed_cap)
                
                min_effective_angular_speed_for_turn = 0.08 # 减小，更精细
                if abs(error_yaw) > self.goal_tolerance_angular * 0.8 and \
                   abs(twist_msg.angular.z) < min_effective_angular_speed_for_turn and \
                   abs(twist_msg.angular.z) > 1e-3 :
                     twist_msg.angular.z = math.copysign(min_effective_angular_speed_for_turn, error_yaw)
                twist_msg.linear.x = 0.0

                rospy.loginfo_throttle(0.1, "%s TrgY:%.1f CurY:%.1f E:%.1f Wz:%.2f OdoWz:%.2f Kp:%.1f Kd:%.1f" % \
                    (log_prefix, math.degrees(target_yaw_rad), math.degrees(self.current_yaw), math.degrees(error_yaw),\
                     twist_msg.angular.z, self.current_angular_vel, kp_turn_sched, kd_turn_sched))

                if abs(error_yaw) < self.goal_tolerance_angular:
                    if abs(self.current_angular_vel) < self.final_stop_angular_vel_threshold and \
                       abs(twist_msg.angular.z) < self.final_stop_angular_vel_threshold * 1.2: # odom和指令都小
                        rospy.loginfo("%s 调整朝向/最终转向稳定完毕。" % log_prefix)
                        self.stop_robot(); time.sleep(0.05)
                        if is_final_turn:
                            self.current_state = self.STATE_FINISHED
                        else:
                            self.current_waypoint_index += 1
                            self.current_state = self.STATE_MOVING_TO_WAYPOINT
                            self.reset_pid_states("move")
                        continue
                    else:
                         rospy.loginfo_throttle(0.05, "%s 接近目标角度但角速度(Cmd:%.2f Odo:%.2f)仍较大，继续微调." % \
                                                (log_prefix, twist_msg.angular.z, self.current_angular_vel))
            
            if self.current_state != self.STATE_PAUSING_AT_WAYPOINT:
                 self.cmd_vel_pub.publish(twist_msg)

            self.rate.sleep()

        rospy.loginfo("所有航点巡视完毕。")
        self.stop_robot()

if __name__ == '__main__':
    try:
        navigator = AdvancedWaypointNavigation() # 使用新类名
        navigator.run_navigation()
    except rospy.ROSInterruptException: rospy.loginfo("导航被中断。")
    except Exception as e:
        rospy.logerr("发生错误: {}".format(e))
        import traceback; traceback.print_exc()
    finally:
        if 'navigator' in locals() and isinstance(navigator, AdvancedWaypointNavigation):
             navigator.stop_robot()
        else:
            rospy.logwarn("Navigator 对象未完全初始化，发送手动停止指令。")
            pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            stop_twist = Twist()
            for _ in range(5): 
                if pub.get_num_connections() > 0: pub.publish(stop_twist); rospy.loginfo("已发送最终手动停止指令。"); break
                rospy.loginfo_once("等待 /cmd_vel 发布者连接..."); rospy.sleep(0.02)
            if pub.get_num_connections() == 0: rospy.logerr("无法连接到 /cmd_vel 发布者！")