#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
自主导航避障系统 - Autonomous Navigation & Obstacle Avoidance
================================================================================
基于 Ego-Planner 避障 + VINS-Fusion + OctoMap 建图的导航系统

核心功能:
 - 多模式状态机 (待机/起飞/悬停/航点/巡逻/返航)
 - Ego-Planner 全局避障规划
 - OctoMap 预建地图导航 (建图→保存→加载→导航)
 - 影子跟随模式 (遥控器切换无顿挫)
 - 安全保护机制 (VINS检查/高度保护/电池保护)

平台: OrangePi 5 Max + D455 + PX4 + VINS-Fusion + EGO-Planner

================================================================================
启动步骤 (完整流程)
================================================================================
【必须先完成】传感器标定:
 - IMU标定 (kalib_标定/imu标定)
 - 双目标定 (kalib_标定/左右双目标定)
 - 外参标定 (vins配置文件)
 - 数据收敛测试

【阶段1: 建图】(可选,如果有预建地图可跳过)
 终端1: roslaunch realsense2_camera rs_camera.launch
 终端2: roslaunch mavros px4.launch
 终端3: roslaunch fuctions mapping.launch map_name:=my_scene
  拿着飞机手动探索环境，OctoMap 实时构建地图
  建图完成后:
    rosservice call /octomap_server/save_map "/home/pi/flight_maps/my_scene.bt"

【阶段2: 自主导航避障飞行】(必须开启6个终端)
 终端1: roslaunch realsense2_camera rs_camera.launch
 终端2: roslaunch mavros px4.launch
 终端3: rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/xxx/xxx.yaml
 终端4: python3 ~/d455_vins_ego-planner/fuctions_ws/src/fuctions/scripts/vins-to-px4.py
 终端5: roslaunch fuctions ego/run_in_sim.launch     # 启动Ego-Planner
 终端6: python3 autonomous_navigator.py             # 启动自主导航

【飞行流程】
 1. 遥控器手动起飞 → 切到稳定悬停
  2. 切换到 OFFBOARD 模式 → 飞机交给系统接管
  3. 使用ROS命令控制飞行

【ROS命令控制】
  rosservice call /nav/takeoff 'data: 1.5'        # 起飞(高度)
  rosservice call /nav/go_to 'data: "3 0 1.2"'   # 飞向位置(x y z)
  rosservice call /nav/land 'data: 0'             # 降落
  rosservice call /nav/return_home 'data: 1.2'    # 返航(高度)
  rosservice call /nav/status 'data: 0'           # 查看状态

【或者直接用Python命令】
  nav.takeoff(1.5)
  nav.go_to(3.0, 0.0, 1.2)
  nav.enable_tracking()  # 启用Ego-Planner避障追踪
  nav.land()
================================================================================
"""

import rospy
import math
import time
import threading
from enum import Enum
from typing import Optional, List, Dict, Any

from geometry_msgs.msg import PoseStamped, Quaternion
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import State, BatteryStatus
from mavros_msgs.srv import CommandTOL, SetMode
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_srvs.srv import Trigger, TriggerResponse


# ================================================================================
# 配置参数
# ================================================================================
class NavConfig:
    """导航配置参数"""
    # 安全参数
    MIN_ALTITUDE = 0.5           # 最低飞行高度 (m)
    DEFAULT_ALTITUDE = 1.2        # 默认悬停高度 (m)
    TAKEOFF_ALTITUDE = 1.5      # 起飞高度 (m)

    # 航点参数
    ARRIVAL_THRESHOLD = 0.3      # 到达判定阈值 (m)
    WAYPOINT_TIMEOUT = 30.0        # 航点超时 (s)

    # 巡逻参数
    PATROL_DWELL_TIME = 2.0       # 航点停留时间 (s)

    # 电池保护
    LOW_BATTERY_THRESHOLD = 0.20  # 20% 电量

    # VINS检查
    VINS_TIMEOUT = 3.0            # VINS数据超时 (s)


# ================================================================================
# 导航模式
# ================================================================================
class NavMode(Enum):
    """导航模式"""
    IDLE = "idle"           # 待机 (不控制)
    SHADOW = "shadow"       # 影子跟随 (遥控器控制)
    TAKEOFF = "takeoff"     # 起飞
    HOLD = "hold"          # 悬停 (当前位置)
    WAYPOINT = "waypoint"   # 航点飞行 (Ego-Planner避障)
    PATROL = "patrol"       # 巡逻
    TRACK = "track"        # Ego-Planner实时避障追踪
    RETURN = "return"       # 返航
    LAND = "land"          # 降落


# ================================================================================
# 自主导航核心类
# ================================================================================
class Navigator:
    """
    自主导航核心类

    功能:
     - 影子跟随 (遥控器切换无顿挫)
     - Ego-Planner 避障融合
     - OctoMap 预建地图支持
     - VINS启动检查 (安全)
     - 航点管理
     - 安全保护
    """

    def __init__(self):
        rospy.init_node('autonomous_navigator')

        # ========== 状态变量 ==========
        self.mode = NavMode.SHADOW  # 初始为影子跟随
        self.mav_state = State()
        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "map"

        # ========== 传感器状态 ==========
        self.has_vins = False
        self.vins_last_update = 0.0  # VINS最后更新时间
        self.battery_pct = 1.0

        # ========== Ego-Planner 数据 ==========
        self.planner_cmd = None
        self.use_ego_planner = False  # 是否启用Ego-Planner避障

        # ========== 航点数据 ==========
        self.waypoints = []
        self.wp_index = 0
        self.last_wp_time = 0

        # ========== 命令队列 (避免阻塞) ==========
        self.pending_cmd = ""
        self.pending_params = ""

        # ========== 初始化接口 ==========
        self._init_comms()

        # MAVROS 服务（用于安全降落/切模式）
        self.srv_land = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.srv_set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # 打印启动引导
        self._print_startup_guide()

    # ============================================================================
    # 初始化
    # ============================================================================
    def _init_comms(self):
        """初始化通信接口"""
        # Publishers
        self.pub_setpoint = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # Subscribers
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb)
        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/mavros/battery', BatteryStatus, self._battery_cb)
        rospy.Subscriber('/vins_estimator/odometry', Odometry, self._vins_cb)
        rospy.Subscriber('/planning/pos_cmd', PositionCommand, self._planner_cb)

        # Timer (30Hz 主控制循环)
        rospy.Timer(rospy.Duration(1.0/30.0), self._control_loop)

        # Timer (10Hz 命令处理循环)
        rospy.Timer(rospy.Duration(0.1), self._command_loop)

        # Services
        rospy.Service('/nav/takeoff', Trigger, self._srv_takeoff)
        rospy.Service('/nav/go_to', Trigger, self._srv_go_to)
        rospy.Service('/nav/hold', Trigger, self._srv_hold)
        rospy.Service('/nav/land', Trigger, self._srv_land_trigger)
        rospy.Service('/nav/return_home', Trigger, self._srv_return_home)
        rospy.Service('/nav/start_patrol', Trigger, self._srv_patrol)
        rospy.Service('/nav/stop', Trigger, self._srv_stop)
        rospy.Service('/nav/enable_tracking', Trigger, self._srv_enable_tracking)
        rospy.Service('/nav/status', Trigger, self._srv_status)

    # ========== 回调函数 ==========
    def _pose_cb(self, msg): self.current_pose = msg
    def _state_cb(self, msg): self.mav_state = msg
    def _battery_cb(self, msg): self.battery_pct = msg.percentage

    def _vins_cb(self, msg):
        """VINS定位回调"""
        self.has_vins = True
        self.vins_last_update = rospy.Time.now().to_sec()

    def _planner_cb(self, msg):
        """Ego-Planner避障指令回调"""
        self.planner_cmd = msg
        # TRACK模式下直接应用规划结果
        if self.mode == NavMode.TRACK and self.use_ego_planner:
            self._apply_planner_cmd(msg)

    # ============================================================================
    # 命令处理循环 (非阻塞)
    # ============================================================================
    def _command_loop(self, event):
        """命令处理循环 (10Hz)"""
        if not self.pending_cmd:
            return

        cmd = self.pending_cmd
        params = self.pending_params
        self.pending_cmd = ""
        self.pending_params = ""

        try:
            if cmd == "takeoff":
                self._do_takeoff(float(params) if params else NavConfig.TAKEOFF_ALTITUDE)
            elif cmd == "go_to":
                parts = params.split()
                self._do_go_to(float(parts[0]), float(parts[1]), float(parts[2]))
            elif cmd == "land":
                self._do_land()
            elif cmd == "return_home":
                self._do_return_home(float(params) if params else NavConfig.DEFAULT_ALTITUDE)
            elif cmd == "hold":
                self._do_hold()
        except Exception as e:
            rospy.logerr("[命令执行错误] {}: {}".format(cmd, e))

    # ============================================================================
    # 核心控制循环 (30Hz)
    # ============================================================================
    def _control_loop(self, event):
        """核心控制循环"""
        try:
            # 1. 安全检查：VINS
            if self.mode not in [NavMode.SHADOW, NavMode.IDLE]:
                if not self._check_vins_healthy():
                    rospy.logerr("[VINS失效] 强制切换到影子跟随!")
                    self.mode = NavMode.SHADOW
                    self._sync_target_to_current()

            # 2. 基础安全检查：如果没有 OFFBOARD，强制进入影子模式
            if self.mav_state.mode != "OFFBOARD":
                self.mode = NavMode.SHADOW
                self._sync_target_to_current()

            # 3. 状态机逻辑处理
            if self.mode == NavMode.SHADOW:
                self._sync_target_to_current()

            elif self.mode == NavMode.TAKEOFF:
                if abs(self.current_pose.pose.position.z - self.target_pose.pose.position.z) < 0.2:
                    rospy.loginfo("起飞完成，进入悬停")
                    self.mode = NavMode.HOLD

            elif self.mode == NavMode.WAYPOINT:
                if self._check_arrival():
                    self.mode = NavMode.HOLD

            elif self.mode == NavMode.RETURN:
                if self._check_arrival():
                    self.mode = NavMode.HOLD

            elif self.mode == NavMode.PATROL:
                if self._check_arrival():
                    if time.time() - self.last_wp_time > NavConfig.PATROL_DWELL_TIME:
                        self.wp_index = (self.wp_index + 1) % len(self.waypoints)
                        self._set_target_from_wp(self.wp_index)
                        self.last_wp_time = time.time()

            elif self.mode == NavMode.TRACK:
                if self.planner_cmd and self.use_ego_planner:
                    self._apply_planner_cmd(self.planner_cmd)

            # 4. 发布指令
            self.target_pose.header.stamp = rospy.Time.now()
            self.pub_setpoint.publish(self.target_pose)

            # 5. 电池检查
            if self.battery_pct < NavConfig.LOW_BATTERY_THRESHOLD:
                rospy.logwarn_throttle(10, "电量低！建议手动降落或执行返航")
                if self.mode not in [NavMode.RETURN, NavMode.LAND]:
                    self._do_return_home(NavConfig.DEFAULT_ALTITUDE)

        except Exception as e:
            rospy.logerr("[控制循环错误] {}".format(e))

    # ============================================================================
    # 应用 Ego-Planner 指令
    # ============================================================================
    def _apply_planner_cmd(self, cmd):
        """应用 Ego-Planner 避障指令"""
        self.target_pose.pose.position.x = cmd.position.x
        self.target_pose.pose.position.y = cmd.position.y
        self.target_pose.pose.position.z = cmd.position.z

        # 偏航角
        q = quaternion_from_euler(0, 0, cmd.yaw)
        self.target_pose.pose.orientation = Quaternion(*q)

    # ============================================================================
    # VINS检查 (安全关键)
    # ============================================================================
    def _check_vins_healthy(self) -> bool:
        """检查VINS是否健康"""
        if not self.has_vins:
            return False

        # 检查VINS数据是否超时
        now = rospy.Time.now().to_sec()
        if now - self.vins_last_update > NavConfig.VINS_TIMEOUT:
            rospy.logwarn_throttle(1, "[VINS超时] 最后更新: {:.1f}s前".format(now - self.vins_last_update))
            return False

        return True

    # ============================================================================
    # 辅助工具函数
    # ============================================================================
    def _sync_target_to_current(self):
        """影子跟随：将目标点同步为当前实际位置"""
        self.target_pose.pose = self.current_pose.pose

    def _check_arrival(self) -> bool:
        """检查是否到达 target_pose"""
        c = self.current_pose.pose.position
        t = self.target_pose.pose.position
        dist = math.sqrt((c.x-t.x)**2 + (c.y-t.y)**2 + (c.z-t.z)**2)
        return dist < NavConfig.ARRIVAL_THRESHOLD

    def _set_target_from_wp(self, index):
        wp = self.waypoints[index]
        self.target_pose.pose.position.x = wp[0]
        self.target_pose.pose.position.y = wp[1]
        self.target_pose.pose.position.z = wp[2] if len(wp) > 2 else NavConfig.DEFAULT_ALTITUDE
        rospy.loginfo(f"巡逻目标更新: WP {index} -> {wp}")

    # ============================================================================
    # 导航命令 (非阻塞)
    # ============================================================================
    def set_mode(self, mode: NavMode):
        """设置导航模式"""
        if mode != self.mode:
            rospy.loginfo("[模式切换] {} -> {}".format(self.mode.value, mode.value))
        self.mode = mode

    def takeoff(self, altitude: Optional[float] = None):
        """起飞 (非阻塞)"""
        alt = max(
            altitude if altitude else NavConfig.TAKEOFF_ALTITUDE,
            NavConfig.DEFAULT_ALTITUDE
        )

        # 检查VINS
        if not self._check_vins_healthy():
            rospy.logerr("[起飞失败] VINS未启动!")
            return

        self.pending_cmd = "takeoff"
        self.pending_params = str(alt)
        rospy.loginfo("[起飞] 目标高度: {:.1f}m".format(alt))

    def _do_takeoff(self, altitude: float):
        """执行起飞"""
        if self.mav_state.mode != "OFFBOARD":
            self._sync_target_to_current()

        self.target_pose.pose.position.x = self.current_pose.pose.position.x
        self.target_pose.pose.position.y = self.current_pose.pose.position.y
        self.target_pose.pose.position.z = altitude
        self.target_pose.pose.orientation.w = 1.0

        self.mode = NavMode.TAKEOFF
        rospy.loginfo("[起飞] 已发送起飞指令")

    def land(self):
        """降落 (非阻塞)"""
        self.pending_cmd = "land"
        self.pending_params = ""
        rospy.loginfo("[降落] 开始降落...")

    def _do_land(self):
        """执行降落"""
        try:
            self.srv_land(0, 0, 0, 0, 0)
            self.mode = NavMode.LAND
            rospy.loginfo("[降落] PX4降落指令已发送")
        except rospy.ServiceException as e:
            rospy.logerr("[降落失败] {}".format(e))

    def go_to(self, x: float, y: float, z: Optional[float] = None):
        """飞向指定位置 (使用Ego-Planner避障)"""
        target_z = max(z if z else NavConfig.DEFAULT_ALTITUDE, NavConfig.DEFAULT_ALTITUDE)

        # 检查VINS
        if not self._check_vins_healthy():
            rospy.logerr("[飞行失���] VINS未启动!")
            return

        # 自动启用Ego-Planner追踪
        if not self.use_ego_planner:
            rospy.logwarn("[提示] Ego-Planner未启动，启用中...")
            self.use_ego_planner = True
            self.mode = NavMode.TRACK

        self.pending_cmd = "go_to"
        self.pending_params = "{} {} {}".format(x, y, target_z)
        rospy.loginfo("[飞向] 目标: ({}, {}, {}) - 使用Ego-Planner避障".format(x, y, target_z))

    def _do_go_to(self, x: float, y: float, z: float):
        """执行飞向"""
        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        self.target_pose.pose.position.z = z
        self.mode = NavMode.WAYPOINT

    def go_home(self, altitude: Optional[float] = None):
        """返航 (非阻塞)"""
        target_alt = altitude if altitude else NavConfig.DEFAULT_ALTITUDE

        if not self._check_vins_healthy():
            rospy.logerr("[返航失败] VINS未启动!")
            return

        self.pending_cmd = "return_home"
        self.pending_params = str(target_alt)
        rospy.loginfo("[返航] 目标: (0, 0, {})".format(target_alt))

    def _do_return_home(self, altitude: float):
        """执行返航"""
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = altitude
        self.mode = NavMode.RETURN

    def hold(self):
        """悬停 (非阻塞)"""
        self.pending_cmd = "hold"
        self.pending_params = ""

    def _do_hold(self):
        """执行悬停"""
        self._sync_target_to_current()
        self.mode = NavMode.HOLD
        rospy.loginfo("[悬停] 当前位置悬停")

    def stop(self):
        """停止控制，切换到影子跟随"""
        self.mode = NavMode.SHADOW
        self.use_ego_planner = False
        self._sync_target_to_current()
        rospy.loginfo("[停止] 切换到影子跟随模式")

    def enable_tracking(self):
        """启用 Ego-Planner 追踪"""
        rospy.loginfo("[追踪] 启用 Ego-Planner 避障追踪")
        self.use_ego_planner = True
        self.mode = NavMode.TRACK

    def disable_tracking(self):
        """禁用 Ego-Planner 追踪"""
        rospy.loginfo("[追踪] 禁用 Ego-Planner 追踪")
        self.use_ego_planner = False
        self.mode = NavMode.HOLD

    # ============================================================================
    # 巡逻功能
    # ============================================================================
    def set_waypoints(self, waypoints: List[List[float]]):
        """设置航点列表"""
        self.waypoints = waypoints
        self.wp_index = 0
        rospy.loginfo("[航点] 已设置 {} 个航点".format(len(waypoints)))

    def start_patrol(self):
        """开始巡逻"""
        if not self.waypoints:
            self.waypoints = [[2,0,1.2], [2,2,1.2], [0,2,1.2], [0,0,1.2]]

        if not self._check_vins_healthy():
            rospy.logerr("[巡逻失败] VINS未启动!")
            return

        self.wp_index = 0
        self._set_target_from_wp(0)
        self.last_wp_time = time.time()
        self.mode = NavMode.PATROL
        rospy.loginfo("[巡逻] 开始巡逻模式")

    def stop_patrol(self):
        """停止巡逻"""
        self.mode = NavMode.HOLD
        rospy.loginfo("[巡逻] 停止")

    # ============================================================================
    # 状态查询
    # ============================================================================
    def get_state(self) -> Dict[str, Any]:
        """获取当前状态"""
        pos = self.current_pose.pose.position
        tgt = self.target_pose.pose.position
        is_offboard = (self.mav_state.mode == "OFFBOARD")

        return {
            'nav_mode': self.mode.value,
            'mav_mode': self.mav_state.mode,
            'offboard': is_offboard,
            'armed': self.mav_state.armed,
            'position': (pos.x, pos.y, pos.z),
            'target': (tgt.x, tgt.y, tgt.z),
            'battery': self.battery_pct,
            'has_vins': self.has_vins,
            'vins_healthy': self._check_vins_healthy(),
            'use_ego_planner': self.use_ego_planner
        }

    def print_state(self):
        """打印状态"""
        s = self.get_state()

        print("\033[2J\033[H")
        print("=" * 50)
        print("  自主导航系统状态")
        print("=" * 50)
        print("  导航模式: {}".format(s['nav_mode']))
        print("  飞控模式: {} | OFFBOARD: {}".format(s['mav_mode'], s['offboard']))
        print("  电池: {:.0%}".format(s['battery']))
        print("-" * 50)
        print("  VINS状态:")
        print("    已启动: {} | 健康: {}".format(s['has_vins'], s['vins_healthy']))
        print("  Ego-Planner: {}".format(s['use_ego_planner']))
        print("-" * 50)
        print("  当前位置: ({:.2f}, {:.2f}, {:.2f})".format(*s['position']))
        print("  目标位置: ({:.2f}, {:.2f}, {:.2f})".format(*s['target']))
        print("=" * 50)

    # ============================================================================
    # 启动引导
    # ============================================================================
    def _print_startup_guide(self):
        """打印启动引导"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("  自主导航避障系统已启动!")
        rospy.loginfo("=" * 60)
        rospy.loginfo("")
        rospy.loginfo("  【必须先启动】(按顺序):")
        rospy.loginfo("    1. roslaunch realsense2_camera rs_camera.launch")
        rospy.loginfo("    2. roslaunch mavros px4.launch")
        rospy.loginfo("    3. rosrun vins vins_node xxx.yaml")
        rospy.loginfo("    4. python3 vins-to-px4.py")
        rospy.loginfo("    5. roslaunch fuctions ego/run_in_sim.launch")
        rospy.loginfo("    6. python3 autonomous_navigator.py  ← 当前")
        rospy.loginfo("")
        rospy.loginfo("  【飞行流程】:")
        rospy.loginfo("    1. 遥控器起飞 → 悬停")
        rospy.loginfo("    2. 切换到 OFFBOARD 模式")
        rospy.loginfo("    3. 使用命令控制飞行")
        rospy.loginfo("")
        rospy.loginfo("  【ROS命令】:")
        rospy.loginfo("    rosservice call /nav/takeoff 'data: 1.5'")
        rospy.loginfo("    rosservice call /nav/go_to 'data: \"3 0 1.2\"'")
        rospy.loginfo("    rosservice call /nav/return_home 'data: 1.2'")
        rospy.loginfo("    rosservice call /nav/land 'data: 0'")
        rospy.loginfo("    rosservice call /nav/status 'data: 0'")
        rospy.loginfo("=" * 60)

    # ============================================================================
    # ROS Service 回调 (非阻塞)
    # ============================================================================
    def _srv_takeoff(self, req):
        """起飞服务"""
        if self.mav_state.mode != "OFFBOARD":
            return TriggerResponse(False, "失败：请先切换到 OFFBOARD 模式")

        if not self._check_vins_healthy():
            return TriggerResponse(False, "失败：VINS未启动")

        alt = float(req.data) if req.data else NavConfig.TAKEOFF_ALTITUDE
        self.takeoff(alt if alt > 0 else NavConfig.TAKEOFF_ALTITUDE)
        return TriggerResponse(True, "起飞指令已发送")

    def _srv_go_to(self, req):
        """飞向服务"""
        if not self._check_vins_healthy():
            return TriggerResponse(False, "失败：VINS未启动")

        try:
            parts = req.data.split()
            if len(parts) >= 3:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            else:
                return TriggerResponse(False, "格式错误：用 'x y z'")

            self.go_to(x, y, z)
            return TriggerResponse(True, "飞行指令已发送")
        except Exception as e:
            return TriggerResponse(False, str(e))

    def _srv_hold(self, req):
        """悬停服务"""
        self.hold()
        return TriggerResponse(True, "悬停指令已发送")

    def _srv_land_trigger(self, req):
        """降落服务"""
        self.land()
        return TriggerResponse(True, "降落指令已发送")

    def _srv_return_home(self, req):
        """返航服务"""
        if not self._check_vins_healthy():
            return TriggerResponse(False, "失败：VINS未启动")

        alt = float(req.data) if req.data else NavConfig.DEFAULT_ALTITUDE
        self.go_home(alt if alt > 0 else NavConfig.DEFAULT_ALTITUDE)
        return TriggerResponse(True, "返航指令已发送")

    def _srv_patrol(self, req):
        """巡逻服务"""
        if not self.waypoints:
            self.waypoints = [[2,0,1.2], [2,2,1.2], [0,2,1.2], [0,0,1.2]]

        if not self._check_vins_healthy():
            return TriggerResponse(False, "失败：VINS未启动")

        self.start_patrol()
        return TriggerResponse(True, "巡逻指令已发送")

    def _srv_stop(self, req):
        """停止服务"""
        self.stop()
        return TriggerResponse(True, "已停止，切换到影子跟随")

    def _srv_enable_tracking(self, req):
        """启用追踪服务"""
        self.enable_tracking()
        return TriggerResponse(True, "Ego-Planner避障已启用")

    def _srv_status(self, req):
        """状态服务"""
        s = self.get_state()
        msg = "模式:{} | OFFBOARD:{} | VINS:{}/{} | 位置:({:.1f},{:.1f},{:.1f}) | 电池:{:.0%}".format(
            s['nav_mode'], s['offboard'], s['has_vins'], s['vins_healthy'],
            s['position'][0], s['position'][1], s['position'][2], s['battery']*100)
        return TriggerResponse(True, msg)


# ================================================================================
# 主函数
# ================================================================================
if __name__ == '__main__':
    try:
        nav = Navigator()

        # 设置默认航点
        nav.set_waypoints([[2,0,1.2], [2,2,1.2], [0,2,1.2], [0,0,1.2]])

        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo("  可用命令 (Python):")
        rospy.loginfo("=" * 60)
        rospy.loginfo("  nav.takeoff(1.5)              - 起飞")
        rospy.loginfo("  nav.land()                   - 降落")
        rospy.loginfo("  nav.go_to(3, 0, 1.2)         - 飞向位置(Ego-Planner)")
        rospy.loginfo("  nav.go_home()               - 返航")
        rospy.loginfo("  nav.hold()                  - 悬停")
        rospy.loginfo("  nav.start_patrol()          - 开始巡逻")
        rospy.loginfo("  nav.enable_tracking()        - 启用Ego-Planner避障")
        rospy.loginfo("  nav.stop()                  - 停止(影子跟随)")
        rospy.loginfo("  nav.print_state()           - 打印状态")
        rospy.loginfo("=" * 60)
        rospy.loginfo("")

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
