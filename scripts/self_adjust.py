#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import time
import numpy as np

# ==============================================================================
# 用户可配置参数区域 (这些现在是自适应调整的初始值和边界)
# ==============================================================================
TARGET_WAYPOINTS = [
    (5.0, 0.0), (5.0, 2.0), (2.0, 5.0), (0.0, 5.0), (0.0, 0.0)
]

# --- 速度与容忍度 ---
DEFAULT_LINEAR_SPEED_MAX = 1.0
DEFAULT_ANGULAR_SPEED_MAX = 1.2
GOAL_TOLERANCE_LINEAR = 0.03
GOAL_TOLERANCE_ANGULAR_DEG = 1.5

# --- PID 控制器基础增益 (自适应调整的起点和参考) ---
# 移动时的PID (暂时不对此进行复杂自适应，重点调转向)
BASE_PID_ANGULAR_MOVE_KP = 1.8 # 调整为更适合直线稳定的值
BASE_PID_ANGULAR_MOVE_KD = 0.35
BASE_PID_ANGULAR_MOVE_KI = 0.00 # 直线I项通常更容易出问题，先禁用

# 原地转向时的PID (这些将是自适应调整的目标)
INITIAL_PID_ANGULAR_TURN_KP = 3.0 # 初始P值
INITIAL_PID_ANGULAR_TURN_KD = 0.4  # 初始D值
INITIAL_PID_ANGULAR_TURN_KI = 0.01 # 初始I值

# --- 自适应PID参数调整边界和步长 ---
# 转向PID的KP调整参数
ADAPTIVE_TURN_KP_MIN = 1.5
ADAPTIVE_TURN_KP_MAX = 5.0
ADAPTIVE_TURN_KP_STEP_INCREASE = 0.1 # 响应慢时增加的步长
ADAPTIVE_TURN_KP_STEP_DECREASE = 0.2 # 震荡或超调时减小的步长 (减小幅度可以大一些)
# 转向PID的KD调整参数
ADAPTIVE_TURN_KD_MIN = 0.1
ADAPTIVE_TURN_KD_MAX = 1.0
ADAPTIVE_TURN_KD_STEP_INCREASE = 0.05 # 震荡或超调时增加的步长
ADAPTIVE_TURN_KD_STEP_DECREASE = 0.02 # 响应慢或P减小时D也可能需要减小
# KI 的自适应调整更复杂，暂时只在初始值上微调或手动调整

# --- 自适应性能监测参数 ---
OVERSHOOT_THRESHOLD_DEG = 5.0 # 超过目标角度多少度算一次显著超调
OSCILLATION_COUNT_THRESHOLD = 3 # 在目标附近，误差符号改变多少次认为在震荡
OSCILLATION_ERROR_MAGNITUDE_DEG = 3.0 # 震荡时误差的典型幅度（大于容忍度）
STAGNATION_TIME_THRESHOLD_SEC = 3.0 # 如果长时间误差没有明显减小，认为响应慢/停滞

# PID积分项抗饱和参数 (保持)
INTEGRAL_MAX_ANGULAR_CONTRIBUTION = 0.2
INTEGRAL_RESET_ERROR_THRESHOLD_DEG = 10

# --- 减速与末端控制参数 (保持) ---
DECELERATION_START_TIME_FACTOR = 1.0
K_DIST_DECELERATION = 3.0
APPROACHING_DISTANCE_THRESHOLD = 0.35
FINAL_APPROACH_LINEAR_SPEED_CAP = 0.05
FINAL_APPROACH_ANGULAR_SPEED_CAP = 0.30
FINAL_STOP_LINEAR_VEL_THRESHOLD = 0.015
FINAL_STOP_ANGULAR_VEL_THRESHOLD_DEG = 1.0

PAUSE_AT_WAYPOINT_DURATION = 0.2
FINAL_TURN_TO_INITIAL_YAW = True
INITIAL_YAW_TARGET_DEG = 0.0
ROS_RATE_HZ = 20
DT = 1.0 / ROS_RATE_HZ
# ==============================================================================

class AdaptiveWaypointNavigation:
    def __init__(self):
        rospy.init_node('adaptive_waypoint_navigation_node', anonymous=False)

        # --- 加载基础参数 (大部分与之前类似) ---
        self.target_waypoints_coords = TARGET_WAYPOINTS
        self.linear_speed_max = rospy.get_param("~linear_speed_max", DEFAULT_LINEAR_SPEED_MAX)
        self.angular_speed_max = rospy.get_param("~angular_speed_max", DEFAULT_ANGULAR_SPEED_MAX)
        self.goal_tolerance_linear = rospy.get_param("~goal_tolerance_linear", GOAL_TOLERANCE_LINEAR)
        self.goal_tolerance_angular = math.radians(rospy.get_param("~goal_tolerance_angular_deg", GOAL_TOLERANCE_ANGULAR_DEG))
        
        # 移动时的PID增益 (固定或简单调度)
        self.pid_angular_move_kp = rospy.get_param("~base_pid_angular_move_kp", BASE_PID_ANGULAR_MOVE_KP)
        self.pid_angular_move_kd = rospy.get_param("~base_pid_angular_move_kd", BASE_PID_ANGULAR_MOVE_KD)
        self.pid_angular_move_ki = rospy.get_param("~base_pid_angular_move_ki", BASE_PID_ANGULAR_MOVE_KI)

        # 转向时的PID增益 (这些是当前正在使用的，会被自适应调整)
        self.current_pid_turn_kp = rospy.get_param("~initial_pid_angular_turn_kp", INITIAL_PID_ANGULAR_TURN_KP)
        self.current_pid_turn_kd = rospy.get_param("~initial_pid_angular_turn_kd", INITIAL_PID_ANGULAR_TURN_KD)
        self.current_pid_turn_ki = rospy.get_param("~initial_pid_angular_turn_ki", INITIAL_PID_ANGULAR_TURN_KI)

        # 自适应调整的边界和步长
        self.kp_turn_min = rospy.get_param("~adaptive_turn_kp_min", ADAPTIVE_TURN_KP_MIN)
        self.kp_turn_max = rospy.get_param("~adaptive_turn_kp_max", ADAPTIVE_TURN_KP_MAX)
        self.kp_turn_step_inc = rospy.get_param("~adaptive_turn_kp_step_increase", ADAPTIVE_TURN_KP_STEP_INCREASE)
        self.kp_turn_step_dec = rospy.get_param("~adaptive_turn_kp_step_decrease", ADAPTIVE_TURN_KP_STEP_DECREASE)
        self.kd_turn_min = rospy.get_param("~adaptive_turn_kd_min", ADAPTIVE_TURN_KD_MIN)
        self.kd_turn_max = rospy.get_param("~adaptive_turn_kd_max", ADAPTIVE_TURN_KD_MAX)
        self.kd_turn_step_inc = rospy.get_param("~adaptive_turn_kd_step_increase", ADAPTIVE_TURN_KD_STEP_INCREASE)
        self.kd_turn_step_dec = rospy.get_param("~adaptive_turn_kd_step_decrease", ADAPTIVE_TURN_KD_STEP_DECREASE)

        # 性能监测参数
        self.overshoot_threshold_rad = math.radians(rospy.get_param("~overshoot_threshold_deg", OVERSHOOT_THRESHOLD_DEG))
        self.oscillation_count_threshold = rospy.get_param("~oscillation_count_threshold", OSCILLATION_COUNT_THRESHOLD)
        self.oscillation_error_magnitude_rad = math.radians(rospy.get_param("~oscillation_error_magnitude_deg", OSCILLATION_ERROR_MAGNITUDE_DEG))
        self.stagnation_time_threshold = rospy.Duration(rospy.get_param("~stagnation_time_threshold_sec", STAGNATION_TIME_THRESHOLD_SEC))


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

        self.current_pose = Point(); self.current_yaw = 0.0
        self.current_linear_vel = 0.0; self.current_angular_vel = 0.0
        self.odom_received = False; self.waypoints = []
        self.current_waypoint_index = 0
        self.prev_angular_error_move = 0.0; self.integral_angular_error_move = 0.0
        self.prev_angular_error_turn = 0.0; self.integral_angular_error_turn = 0.0

        # 自适应状态变量 (用于转向性能监测)
        self.turn_start_time = None
        self.turn_start_error_abs = None
        self.turn_overshot = False
        self.turn_oscillation_counter = 0
        self.turn_last_error_sign = 0

        self.STATE_IDLE = 0; self.STATE_MOVING_TO_WAYPOINT = 1
        self.STATE_TURNING_FOR_NEXT_WAYPOINT = 2; self.STATE_PAUSING_AT_WAYPOINT = 3
        self.STATE_FINAL_TURNING = 4; self.STATE_FINISHED = 5
        self.current_state = self.STATE_IDLE
        self.rate = rospy.Rate(ROS_RATE_HZ)
        self._prepare_waypoints()

    # ... (_prepare_waypoints, odom_callback, get_target_yaw, normalize_angle, stop_robot, calculate_angular_pid_output, reset_pid_states 与之前版本相同) ...
    # (确保 calculate_angular_pid_output 和 reset_pid_states 使用正确的 prev_error 和 integral_error 变量)
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
            
    def adaptive_tune_turn_pid(self, error_yaw_abs, time_in_turn_state, just_finished_turn):
        """根据转向性能启发式地调整转向PID增益"""
        # --- 规则1: 检测到超调或持续震荡，则减P，增D ---
        if self.turn_overshot or self.turn_oscillation_counter >= self.oscillation_count_threshold:
            self.current_pid_turn_kp = max(self.kp_turn_min, self.current_pid_turn_kp - self.kp_turn_step_dec)
            self.current_pid_turn_kd = min(self.kd_turn_max, self.current_pid_turn_kd + self.kd_turn_step_inc)
            rospy.logwarn_throttle(1.0, "[ADAPTIVE] 检测到超调/震荡! Kp_turn -> %.2f, Kd_turn -> %.2f" % \
                                (self.current_pid_turn_kp, self.current_pid_turn_kd))
            # 重置监测状态
            self.turn_overshot = False
            self.turn_oscillation_counter = 0
            return True # 表示参数已调整

        # --- 规则2: 如果长时间无法达到目标（停滞），则增P，可能减D ---
        # （仅在转向刚刚完成时评估，避免在转向过程中误判）
        if just_finished_turn and time_in_turn_state > self.stagnation_time_threshold:
            # 并且初始误差较大，说明不是因为目标太近导致的慢
            if self.turn_start_error_abs is not None and self.turn_start_error_abs > math.radians(10.0):
                self.current_pid_turn_kp = min(self.kp_turn_max, self.current_pid_turn_kp + self.kp_turn_step_inc)
                # 如果P增加了，D可能需要微调，或者如果系统响应非常迟钝，可以尝试略微减小D
                # self.current_pid_turn_kd = max(self.kd_turn_min, self.current_pid_turn_kd - self.kd_turn_step_dec)
                rospy.logwarn_throttle(1.0, "[ADAPTIVE] 转向响应慢! Kp_turn -> %.2f" % self.current_pid_turn_kp)
                return True

        return False # 参数未调整
        
    def monitor_turn_performance(self, error_yaw, target_yaw_rad):
        """在转向过程中监测超调和震荡"""
        current_error_sign = np.sign(error_yaw)
        error_yaw_abs = abs(error_yaw)

        # 监测超调 (从误差较大 -> 越过目标 -> 误差反向且超过阈值)
        if self.turn_start_error_abs is not None and not self.turn_overshot:
            # 如果误差符号改变，并且曾经误差较大，现在误差的绝对值又超过了超调阈值
            # 意味着机器人可能已经越过了目标点，并且偏离了不少
            if current_error_sign != np.sign(self.prev_angular_error_turn) and \
               self.turn_start_error_abs > self.overshoot_threshold_rad * 2 and \
               error_yaw_abs > self.overshoot_threshold_rad :
                # 额外判断：是否真的越过了目标？检查当前yaw与目标yaw的关系
                # （需要更复杂的逻辑来精确判断超调，这里简化处理）
                # 一个简单的判断是：如果误差符号变了，并且当前误差的绝对值仍然不小
                self.turn_overshot = True
                rospy.loginfo_throttle(0.5, "[ADAPTIVE] 检测到潜在超调 (overshoot).")


        # 监测震荡 (误差符号在目标附近多次改变)
        if error_yaw_abs < self.oscillation_error_magnitude_rad * 1.5 and \
           error_yaw_abs > self.goal_tolerance_angular * 0.8: # 在目标附近但未稳定
            if self.turn_last_error_sign != 0 and current_error_sign != self.turn_last_error_sign:
                self.turn_oscillation_counter += 1
                # rospy.loginfo_throttle(0.2, "[ADAPTIVE] 震荡计数: %d" % self.turn_oscillation_counter)
        
        self.turn_last_error_sign = current_error_sign


    def run_navigation(self):
        rospy.loginfo("等待里程计数据...")
        while not self.odom_received and not rospy.is_shutdown(): self.rate.sleep()
        if rospy.is_shutdown() or not self.waypoints:
            rospy.logwarn("没有航点或ROS关闭，退出。"); return

        self.current_state = self.STATE_MOVING_TO_WAYPOINT
        self.current_waypoint_index = 0
        self.reset_pid_states("all")
        rospy.loginfo("开始自适应PID定点巡视。总共 %d 个航点。" % len(self.waypoints))

        # 用于记录当前状态的开始时间，用于判断停滞
        state_start_time = rospy.Time.now()

        while not rospy.is_shutdown() and self.current_state != self.STATE_FINISHED:
            current_time = rospy.Time.now()
            time_in_current_state = current_time - state_start_time

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
                    state_start_time = rospy.Time.now() # 重置状态计时器
                    self.turn_start_error_abs = abs(self.normalize_angle(self.initial_yaw_target_rad - self.current_yaw))
                    self.turn_overshot = False; self.turn_oscillation_counter = 0; self.turn_last_error_sign = 0
                else:
                    self.current_state = self.STATE_FINISHED
                continue

            twist_msg = Twist()
            # --- 获取当前线速度的绝对值，用于增益调度 ---
            # (注意：self.current_linear_vel 是从odom获取的，可能与指令速度有延迟或差异)
            # 如果希望基于指令速度进行调度，可以使用上一次发布的twist_msg.linear.x
            # 这里我们暂时用odom的速度
            current_linear_velocity_for_scheduling = abs(self.current_linear_vel)


            if self.current_state == self.STATE_MOVING_TO_WAYPOINT:
                distance_to_target = math.sqrt((target_waypoint.x - self.current_pose.x)**2 + \
                                               (target_waypoint.y - self.current_pose.y)**2)
                desired_yaw = self.get_target_yaw(target_waypoint)
                error_yaw = self.normalize_angle(desired_yaw - self.current_yaw)

                # 移动时的角度PID (增益调度暂时不用于移动，保持固定或简单调度)
                # (如果需要，可以调用 get_scheduled_pid_gains)
                kp_move, kd_move, ki_move = self.pid_angular_move_kp, self.pid_angular_move_kd, self.pid_angular_move_ki
                
                angular_velocity_pid_raw = self.calculate_angular_pid_output(error_yaw, 
                                                                         kp_move, kd_move, ki_move,
                                                                         [self.prev_angular_error_move],
                                                                         [self.integral_angular_error_move],
                                                                         mode="move")
                
                # --- 线速度控制逻辑 (与之前版本类似，可以进一步优化S曲线) ---
                angle_error_factor = math.exp(-2.0 * (abs(error_yaw) / math.radians(30.0))**2)
                current_target_linear_speed = self.linear_speed_max * angle_error_factor
                deceleration_start_distance = self.linear_speed_max * self.deceleration_start_time_factor
                if distance_to_target < deceleration_start_distance:
                    effective_decel_range = deceleration_start_distance - self.goal_tolerance_linear * 0.2
                    normalized_dist_in_decel_range = max(0, (distance_to_target - self.goal_tolerance_linear * 0.2) / (effective_decel_range + 1e-6) )
                    distance_factor = math.tanh(self.k_dist_deceleration * normalized_dist_in_decel_range**1.2)
                    current_target_linear_speed *= distance_factor
                final_linear_speed = max(0, current_target_linear_speed)
                is_approaching_final = (distance_to_target < self.approaching_distance_threshold)
                current_angular_cap = self.angular_speed_max
                if is_approaching_final:
                    final_linear_speed = min(final_linear_speed, self.final_approach_linear_speed_cap)
                    current_angular_cap = self.final_approach_angular_speed_cap
                twist_msg.angular.z = max(min(angular_velocity_pid_raw, current_angular_cap), -current_angular_cap)
                twist_msg.linear.x = final_linear_speed
                min_move_linear_speed = 0.03; min_move_angular_speed = 0.06
                if not is_approaching_final: # Minimal drive logic
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
                # --- 日志 ---
                rospy.loginfo_throttle(0.2, "MOVE Wpt%d D:%.2fm Y:%.1f E:%.1f ActVx:%.2f OdoVx:%.2f Wz:%.2f OdoWz:%.2f KpM:%.1f KdM:%.1f Appr:%d" % \
                    (self.current_waypoint_index, distance_to_target, math.degrees(self.current_yaw), math.degrees(error_yaw), \
                     twist_msg.linear.x, self.current_linear_vel, twist_msg.angular.z, self.current_angular_vel, \
                     kp_move, kd_move, int(is_approaching_final))) #显示固定move_pid

                if distance_to_target < self.goal_tolerance_linear:
                    is_stable_now = (abs(self.current_linear_vel) < self.final_stop_linear_vel_threshold and \
                                     abs(self.current_angular_vel) < self.final_stop_angular_vel_threshold and \
                                     abs(twist_msg.linear.x) < self.final_stop_linear_vel_threshold * 1.2 and \
                                     abs(twist_msg.angular.z) < self.final_stop_angular_vel_threshold * 1.2)
                    if is_stable_now:
                        rospy.loginfo("已稳定到达航点 %d。" % self.current_waypoint_index)
                        self.stop_robot(); self.current_state = self.STATE_PAUSING_AT_WAYPOINT
                        state_start_time = rospy.Time.now() # 重置状态计时器
                        continue
                    else: rospy.loginfo_throttle(0.1, "接近航点 %d 但需稳定..." % self.current_waypoint_index)
            
            elif self.current_state == self.STATE_PAUSING_AT_WAYPOINT:
                self.stop_robot()
                if self.pause_duration > DT : rospy.sleep(self.pause_duration - DT) # time.sleep is blocking
                rospy.loginfo("在航点 %d 暂停完毕。" % self.current_waypoint_index)
                self.reset_pid_states("all")
                if self.current_waypoint_index + 1 < len(self.waypoints):
                    self.current_state = self.STATE_TURNING_FOR_NEXT_WAYPOINT
                    # 为下一次转向初始化性能监测变量
                    self.turn_start_error_abs = None # 会在转向状态开始时设置
                    self.turn_overshot = False; self.turn_oscillation_counter = 0; self.turn_last_error_sign = 0
                else:
                    self.current_state = self.STATE_FINAL_TURNING if self.final_turn else self.STATE_FINISHED
                    if self.final_turn:
                         self.turn_start_error_abs = abs(self.normalize_angle(self.initial_yaw_target_rad - self.current_yaw))
                         self.turn_overshot = False; self.turn_oscillation_counter = 0; self.turn_last_error_sign = 0

                state_start_time = rospy.Time.now() # 重置状态计时器
                continue

            elif self.current_state == self.STATE_TURNING_FOR_NEXT_WAYPOINT or \
                 self.current_state == self.STATE_FINAL_TURNING:

                is_final_turn = (self.current_state == self.STATE_FINAL_TURNING)
                
                if is_final_turn:
                    target_yaw_rad = self.initial_yaw_target_rad
                    log_prefix = "FINAL_TURN"
                else: # TURNING_FOR_NEXT_WAYPOINT
                    next_waypoint_idx = self.current_waypoint_index + 1
                    if next_waypoint_idx >= len(self.waypoints): self.current_state = self.STATE_FINISHED; continue
                    next_target_wp = self.waypoints[next_waypoint_idx]
                    target_yaw_rad = self.get_target_yaw(next_target_wp)
                    log_prefix = "TURN Wpt%d" % next_waypoint_idx
                
                error_yaw = self.normalize_angle(target_yaw_rad - self.current_yaw)

                # 在转向状态开始时，记录初始误差和时间
                if self.turn_start_error_abs is None: # 仅在进入此转向状态的第一次循环执行
                    self.turn_start_error_abs = abs(error_yaw)
                    self.turn_start_time = current_time # 使用当前循环的时间
                    self.turn_overshot = False
                    self.turn_oscillation_counter = 0
                    self.turn_last_error_sign = np.sign(error_yaw) if abs(error_yaw) > 1e-3 else 0


                # 监测转向性能
                self.monitor_turn_performance(error_yaw, target_yaw_rad)

                # 使用当前自适应调整后的转向PID增益
                kp, kd, ki = self.current_pid_turn_kp, self.current_pid_turn_kd, self.current_pid_turn_ki
                
                angular_velocity_pid_raw = self.calculate_angular_pid_output(error_yaw,
                                                                         kp, kd, ki, # 使用当前的（可能已自适应的）增益
                                                                         [self.prev_angular_error_turn],
                                                                         [self.integral_angular_error_turn],
                                                                         mode="turn_adaptive")
                
                current_angular_speed_cap = self.angular_speed_max if not is_final_turn else self.angular_speed_max * 0.85
                twist_msg.angular.z = max(min(angular_velocity_pid_raw, current_angular_speed_cap), -current_angular_speed_cap)
                min_effective_angular_speed_for_turn = 0.08
                if abs(error_yaw) > self.goal_tolerance_angular * 0.8 and \
                   abs(twist_msg.angular.z) < min_effective_angular_speed_for_turn and \
                   abs(twist_msg.angular.z) > 1e-3 :
                     twist_msg.angular.z = math.copysign(min_effective_angular_speed_for_turn, error_yaw)
                twist_msg.linear.x = 0.0

                rospy.loginfo_throttle(0.1, "%s TrgY:%.1f CurY:%.1f E:%.1f Wz:%.2f KpT:%.1f KdT:%.1f Osc:%d OvS:%s" % \
                    (log_prefix, math.degrees(target_yaw_rad), math.degrees(self.current_yaw), math.degrees(error_yaw),\
                     twist_msg.angular.z, kp, kd, self.turn_oscillation_counter, str(self.turn_overshot)))

                if abs(error_yaw) < self.goal_tolerance_angular:
                    if abs(self.current_angular_vel) < self.final_stop_angular_vel_threshold and \
                       abs(twist_msg.angular.z) < self.final_stop_angular_vel_threshold * 1.2:
                        
                        rospy.loginfo("%s 转向稳定完毕。KpT:%.2f, KdT:%.2f" % (log_prefix, self.current_pid_turn_kp, self.current_pid_turn_kd))
                        # --- 在转向完成后，根据本次转向的性能调整PID参数 ---
                        time_taken_for_turn = current_time - self.turn_start_time if self.turn_start_time else rospy.Duration(0)
                        self.adaptive_tune_turn_pid(abs(error_yaw), time_taken_for_turn, True)
                        # --- 调整完毕 ---
                        
                        self.stop_robot(); time.sleep(0.05)
                        self.turn_start_error_abs = None # 重置，为下一次转向做准备

                        if is_final_turn:
                            self.current_state = self.STATE_FINISHED
                        else:
                            self.current_waypoint_index += 1
                            self.current_state = self.STATE_MOVING_TO_WAYPOINT
                            self.reset_pid_states("move")
                        state_start_time = rospy.Time.now() # 重置状态计时器
                        continue
                    else:
                         rospy.loginfo_throttle(0.05, "%s 接近目标角度但角速度(Cmd:%.2f Odo:%.2f)仍较大..." % \
                                                (log_prefix, twist_msg.angular.z, self.current_angular_vel))
                elif time_in_current_state > self.stagnation_time_threshold * 1.5 and \
                     self.turn_start_error_abs is not None and abs(error_yaw) > self.turn_start_error_abs * 0.3 : # 长时间误差没有显著减小
                    rospy.logwarn_throttle(1.0, "[ADAPTIVE] %s 转向可能停滞! 尝试调整PID。" % log_prefix)
                    # 触发一次PID调整，即使还没完成转向
                    time_taken_for_turn = current_time - self.turn_start_time if self.turn_start_time else rospy.Duration(0)
                    if self.adaptive_tune_turn_pid(abs(error_yaw), time_taken_for_turn, False): # just_finished_turn = False
                         self.reset_pid_states("turn") # 如果参数调整了，重置PID状态
                         self.turn_start_error_abs = None # 强制在下个循环重新初始化转向监控
            
            if self.current_state != self.STATE_PAUSING_AT_WAYPOINT:
                 self.cmd_vel_pub.publish(twist_msg)

            self.rate.sleep()

        rospy.loginfo("所有航点巡视完毕。")
        self.stop_robot()

if __name__ == '__main__':
    try:
        navigator = AdaptiveWaypointNavigation()
        navigator.run_navigation()
    except rospy.ROSInterruptException: rospy.loginfo("导航被中断。")
    except Exception as e:
        rospy.logerr("发生错误: {}".format(e))
        import traceback; traceback.print_exc()
    finally:
        if 'navigator' in locals() and isinstance(navigator, AdaptiveWaypointNavigation):
             navigator.stop_robot()
        else:
            rospy.logwarn("Navigator 对象未完全初始化，发送手动停止指令。")
            # ... (与之前版本相同的最终停止逻辑) ...
            pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            stop_twist = Twist()
            for _ in range(5): 
                if pub.get_num_connections() > 0: pub.publish(stop_twist); rospy.loginfo("已发送最终手动停止指令。"); break
                rospy.loginfo_once("等待 /cmd_vel 发布者连接..."); rospy.sleep(0.02)
            if pub.get_num_connections() == 0: rospy.logerr("无法连接到 /cmd_vel 发布者！")