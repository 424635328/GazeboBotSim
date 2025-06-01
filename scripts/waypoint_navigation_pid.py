#!/usr/bin/env python
# -*- coding: utf-8 -*- # 声明UTF-8编码以支持中文注释

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import time

# ==============================================================================
# 用户可配置参数区域
# ==============================================================================
# 目标航点列表: [(x1, y1), (x2, y2), ...] 单位：米
TARGET_WAYPOINTS = [
    (5.0, 0.0),
    (5.0, 2.0),
    (2.0, 5.0),
    (0.0, 5.0),
    (0.0, 0.0)
]

# --- 速度与容忍度 ---
DEFAULT_LINEAR_SPEED_MAX = 0.8  # 稍微降低最大线速度，以便更容易稳定方向
DEFAULT_ANGULAR_SPEED_MAX = 1.0 # 最大角速度可以保持或略微降低
GOAL_TOLERANCE_LINEAR = 0.03
GOAL_TOLERANCE_ANGULAR_DEG = 2.0 # 角度容忍度可以稍微放宽一点，如果过于追求1.5度导致震荡

# --- PID 控制器增益 ---
# 移动时的PID (边走边调整方向) - **核心调整区域**
PID_ANGULAR_MOVE_KP = 1.2   # **显著降低P!**
PID_ANGULAR_MOVE_KD = 0.3   # **适当增加D!**
PID_ANGULAR_MOVE_KI = 0.00  # **强烈建议先设为0!**
# 原地转向时的PID (在航点处纯旋转) - 这部分可以保持相对激进
PID_ANGULAR_TURN_KP = 3.0
PID_ANGULAR_TURN_KD = 0.45
PID_ANGULAR_TURN_KI = 0.005

# PID积分项抗饱和参数
INTEGRAL_MAX_ANGULAR_CONTRIBUTION = 0.15
INTEGRAL_RESET_ERROR_THRESHOLD_DEG = 15

# --- 减速与末端控制参数 ---
DECELERATION_START_TIME_FACTOR = 1.0 # 减速开始时机
K_DIST_DECELERATION = 2.8            # S型减速曲线陡峭度

APPROACHING_DISTANCE_THRESHOLD = 0.30
FINAL_APPROACH_LINEAR_SPEED_CAP = 0.04
FINAL_APPROACH_ANGULAR_SPEED_CAP = 0.25

FINAL_STOP_LINEAR_VEL_THRESHOLD = 0.015
FINAL_STOP_ANGULAR_VEL_THRESHOLD_DEG = 1.0

# --- 其他行为参数 ---
PAUSE_AT_WAYPOINT_DURATION = 0.2
FINAL_TURN_TO_INITIAL_YAW = True
INITIAL_YAW_TARGET_DEG = 0.0

ROS_RATE_HZ = 20
DT = 1.0 / ROS_RATE_HZ
# ==============================================================================

class WaypointNavigationPIDController: # <<<--- 修改了类名，不再继承
    def __init__(self):
        rospy.init_node('waypoint_navigation_pid_controller_node', anonymous=False) # <<<--- 修改了节点名

        # --- 加载参数 ---
        self.target_waypoints_coords = TARGET_WAYPOINTS
        self.linear_speed_max = rospy.get_param("~linear_speed_max", DEFAULT_LINEAR_SPEED_MAX)
        self.angular_speed_max = rospy.get_param("~angular_speed_max", DEFAULT_ANGULAR_SPEED_MAX)
        self.goal_tolerance_linear = rospy.get_param("~goal_tolerance_linear", GOAL_TOLERANCE_LINEAR)
        self.goal_tolerance_angular = math.radians(rospy.get_param("~goal_tolerance_angular_deg", GOAL_TOLERANCE_ANGULAR_DEG))
        
        self.pid_angular_move_kp = rospy.get_param("~pid_angular_move_kp", PID_ANGULAR_MOVE_KP)
        self.pid_angular_move_kd = rospy.get_param("~pid_angular_move_kd", PID_ANGULAR_MOVE_KD)
        self.pid_angular_move_ki = rospy.get_param("~pid_angular_move_ki", PID_ANGULAR_MOVE_KI)

        self.pid_angular_turn_kp = rospy.get_param("~pid_angular_turn_kp", PID_ANGULAR_TURN_KP)
        self.pid_angular_turn_kd = rospy.get_param("~pid_angular_turn_kd", PID_ANGULAR_TURN_KD)
        self.pid_angular_turn_ki = rospy.get_param("~pid_angular_turn_ki", PID_ANGULAR_TURN_KI)

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
        # --- 参数加载完毕 ---

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.current_pose = Point()
        self.current_yaw = 0.0
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

    def _prepare_waypoints(self):
        self.waypoints = []
        if not self.target_waypoints_coords: rospy.logwarn("目标航点列表为空！"); return
        for i, coord_tuple in enumerate(self.target_waypoints_coords):
            if len(coord_tuple) >= 2:
                wp = Point(); wp.x = coord_tuple[0]; wp.y = coord_tuple[1]; wp.z = 0
                self.waypoints.append(wp)
                rospy.loginfo("已加载航点 %d: (%.2f, %.2f)" % (i, wp.x, wp.y))
            else: rospy.logwarn("航点 %s 格式不正确，已忽略。" % str(coord_tuple))
        if not self.waypoints: rospy.logwarn("没有有效的航点被加载！")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.current_yaw) = euler_from_quaternion(orientation_list)
        self.odom_received = True

    def get_target_yaw(self, target_point):
        return math.atan2(target_point.y - self.current_pose.y, target_point.x - self.current_pose.x)

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def stop_robot(self):
        twist_msg = Twist()
        self.cmd_vel_pub.publish(twist_msg)

    def calculate_angular_pid_output(self, current_error, kp, kd, ki, prev_error_list, integral_error_list, mode="turn"):
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

    def reset_pid_states(self, mode="all"):
        if mode == "all" or mode == "move":
            self.prev_angular_error_move = 0.0
            self.integral_angular_error_move = 0.0
        if mode == "all" or mode == "turn":
            self.prev_angular_error_turn = 0.0
            self.integral_angular_error_turn = 0.0
            
    def run_navigation(self):
        rospy.loginfo("等待里程计数据...")
        while not self.odom_received and not rospy.is_shutdown(): self.rate.sleep()
        if rospy.is_shutdown() or not self.waypoints:
            rospy.logwarn("没有航点或ROS关闭，退出。"); return

        self.current_state = self.STATE_MOVING_TO_WAYPOINT
        self.current_waypoint_index = 0
        self.reset_pid_states("all")
        rospy.loginfo("开始PID稳定巡视。总共 %d 个航点。" % len(self.waypoints))

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

                angular_velocity_pid_raw = self.calculate_angular_pid_output(error_yaw, 
                                                                         self.pid_angular_move_kp, 
                                                                         self.pid_angular_move_kd, 
                                                                         self.pid_angular_move_ki,
                                                                         [self.prev_angular_error_move],
                                                                         [self.integral_angular_error_move],
                                                                         mode="move")
                
                current_target_linear_speed = self.linear_speed_max
                
                angle_error_threshold_for_slowdown_deg_move = 20.0
                if abs(error_yaw) > math.radians(angle_error_threshold_for_slowdown_deg_move * 0.25): # 误差大于5度开始影响
                    if abs(error_yaw) > math.radians(angle_error_threshold_for_slowdown_deg_move * 0.75): # 误差大于15度
                         angle_decay_factor = 0.25
                    elif abs(error_yaw) > math.radians(angle_error_threshold_for_slowdown_deg_move * 0.5): # 误差大于10度
                         angle_decay_factor = 0.55
                    else: # 误差在 5-10 度之间
                         angle_decay_factor = 0.80
                    current_target_linear_speed *= angle_decay_factor

                deceleration_start_distance = self.linear_speed_max * self.deceleration_start_time_factor
                if distance_to_target < deceleration_start_distance:
                    effective_decel_range = deceleration_start_distance - self.goal_tolerance_linear * 0.3
                    normalized_dist_in_decel_range = max(0, (distance_to_target - self.goal_tolerance_linear * 0.3) / (effective_decel_range + 1e-6) )
                    distance_factor = math.tanh(self.k_dist_deceleration * normalized_dist_in_decel_range)
                    current_target_linear_speed *= distance_factor
                
                final_linear_speed = max(0, current_target_linear_speed)

                is_approaching_final = (distance_to_target < self.approaching_distance_threshold)
                
                if is_approaching_final:
                    final_linear_speed = min(final_linear_speed, self.final_approach_linear_speed_cap)
                    effective_angular_speed_cap_final = self.final_approach_angular_speed_cap
                else:
                    effective_angular_speed_cap_final = self.angular_speed_max

                twist_msg.angular.z = max(min(angular_velocity_pid_raw, effective_angular_speed_cap_final), -effective_angular_speed_cap_final)
                twist_msg.linear.x = final_linear_speed

                min_move_linear_speed = 0.025
                min_move_angular_speed = 0.05 # **减小这个值**
                if not is_approaching_final:
                    if twist_msg.linear.x > 1e-3 and twist_msg.linear.x < min_move_linear_speed and \
                       distance_to_target > self.goal_tolerance_linear * 3.0 and abs(error_yaw) < math.radians(8):
                        twist_msg.linear.x = min_move_linear_speed
                    if abs(error_yaw) > self.goal_tolerance_angular * 1.1 and \
                       abs(twist_msg.angular.z) < min_move_angular_speed and \
                       abs(twist_msg.angular.z) > 1e-4:
                        twist_msg.angular.z = math.copysign(min_move_angular_speed, error_yaw)
                else: 
                    if abs(twist_msg.linear.x) < (self.final_stop_linear_vel_threshold * 0.7):
                        twist_msg.linear.x = 0.0
                    if abs(twist_msg.angular.z) < (self.final_stop_angular_vel_threshold * 0.7):
                        twist_msg.angular.z = 0.0

                rospy.loginfo_throttle(0.2, "MOVE Wpt%d(%.1f,%.1f) D:%.3fm Y:%.1f° E:%.1f° TargetVx:%.2f ActVx:%.3f Wz:%.3f Appr:%d" % \
                    (self.current_waypoint_index, target_waypoint.x, target_waypoint.y, \
                     distance_to_target, math.degrees(self.current_yaw), math.degrees(error_yaw), \
                     current_target_linear_speed, twist_msg.linear.x, twist_msg.angular.z, int(is_approaching_final)))

                if distance_to_target < self.goal_tolerance_linear:
                    is_stable_now = (abs(twist_msg.linear.x) < self.final_stop_linear_vel_threshold and \
                                     abs(twist_msg.angular.z) < self.final_stop_angular_vel_threshold)
                    if is_stable_now:
                        rospy.loginfo("已精确稳定到达航点 %d (D:%.3fm, CmdVx:%.3f, CmdWz:%.3f)." % \
                                      (self.current_waypoint_index, distance_to_target, twist_msg.linear.x, twist_msg.angular.z))
                        self.stop_robot()
                        self.current_state = self.STATE_PAUSING_AT_WAYPOINT
                        continue
                    else:
                        rospy.loginfo_throttle(0.1, "接近航点 %d 但速度/角速度仍需稳定. D:%.3fm, CmdVx:%.3f, CmdWz:%.3f" % \
                                               (self.current_waypoint_index, distance_to_target, twist_msg.linear.x, twist_msg.angular.z))
            
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
                if is_final_turn:
                    target_yaw_rad = self.initial_yaw_target_rad
                    log_prefix = "FINAL_TURN"
                    target_log_info = "TrgY:%.1f°" % math.degrees(target_yaw_rad)
                else:
                    next_waypoint_idx = self.current_waypoint_index + 1
                    if next_waypoint_idx >= len(self.waypoints): self.current_state = self.STATE_FINISHED; continue
                    next_target_wp = self.waypoints[next_waypoint_idx]
                    target_yaw_rad = self.get_target_yaw(next_target_wp)
                    log_prefix = "TURN Wpt%d(%.1f,%.1f)" % (next_waypoint_idx, next_target_wp.x, next_target_wp.y)
                    target_log_info = "TrgY:%.1f°" % math.degrees(target_yaw_rad)

                error_yaw = self.normalize_angle(target_yaw_rad - self.current_yaw)
                angular_velocity_pid_raw = self.calculate_angular_pid_output(error_yaw,
                                                                         self.pid_angular_turn_kp,
                                                                         self.pid_angular_turn_kd,
                                                                         self.pid_angular_turn_ki,
                                                                         [self.prev_angular_error_turn],
                                                                         [self.integral_angular_error_turn],
                                                                         mode="turn" if not is_final_turn else "final_turn")
                
                current_angular_speed_cap = self.angular_speed_max if not is_final_turn else self.angular_speed_max * 0.8
                twist_msg.angular.z = max(min(angular_velocity_pid_raw, current_angular_speed_cap), -current_angular_speed_cap)
                
                min_effective_angular_speed_for_turn = 0.1
                if abs(error_yaw) > self.goal_tolerance_angular * 0.8 and \
                   abs(twist_msg.angular.z) < min_effective_angular_speed_for_turn and \
                   abs(twist_msg.angular.z) > 1e-3 :
                     twist_msg.angular.z = math.copysign(min_effective_angular_speed_for_turn, error_yaw)
                twist_msg.linear.x = 0.0

                rospy.loginfo_throttle(0.15, "%s %s CurY:%.1f° E:%.1f° Wz:%.3f" % \
                    (log_prefix, target_log_info, math.degrees(self.current_yaw), math.degrees(error_yaw),\
                     twist_msg.angular.z))

                if abs(error_yaw) < self.goal_tolerance_angular:
                    if abs(twist_msg.angular.z) < self.final_stop_angular_vel_threshold:
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
                         rospy.loginfo_throttle(0.05, "%s 接近目标角度但角速度(%.3f)仍较大，继续微调." % \
                                                (log_prefix, twist_msg.angular.z))
            
            if self.current_state != self.STATE_PAUSING_AT_WAYPOINT:
                 self.cmd_vel_pub.publish(twist_msg)

            self.rate.sleep()

        rospy.loginfo("所有航点巡视完毕。")
        self.stop_robot()

if __name__ == '__main__':
    try:
        navigator = WaypointNavigationPIDController() # <<<--- 使用新的类名
        navigator.run_navigation()
    except rospy.ROSInterruptException: rospy.loginfo("导航被中断。")
    except Exception as e:
        rospy.logerr("发生错误: {}".format(e))
        import traceback; traceback.print_exc()
    finally:
        if 'navigator' in locals() and isinstance(navigator, WaypointNavigationPIDController): # <<<--- 使用新的类名
             navigator.stop_robot()
        else:
            rospy.logwarn("Navigator 对象未完全初始化或类型错误，发送手动停止指令。")
            pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            stop_twist = Twist()
            for _ in range(5): 
                if pub.get_num_connections() > 0:
                    pub.publish(stop_twist)
                    rospy.loginfo("已发送最终手动停止指令。")
                    break
                rospy.loginfo_once("等待 /cmd_vel 发布者连接...")
                rospy.sleep(0.02)
            if pub.get_num_connections() == 0:
                rospy.logerr("无法连接到 /cmd_vel 发布者，机器人可能未停止！")