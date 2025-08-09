import math
import numpy as np
from Controller.maybe import Maybesomewhere
from .uav_fov_manager import UAVFOVManager

def pi_to_pi(angle):
    """将角度归一化到 [-pi, pi] 范围内"""
    return (angle + math.pi) % (2 * math.pi) - math.pi

# --- UAV 控制类 --- #
class UAVController:
    def __init__(self, dt=0.5, enable_fov_coverage=True):
        self.dt = dt
        
        # --- UAV 物理参数 --- #
        self.UAV_MAX_SPEED = 33.3
        self.UAV_SEARCH_SPEED = 33.3
        self.UAV_TURN_SPEED = 33.3
        self.UAV_MIN_TURN_RADIUS = 100.0
        
        # --- UAV 控制参数 --- #
        self.ARRIVAL_THRESHOLD = 50.0   # 到点判断阈值
        self.HEADING_KP = 2.0   # 朝向控制增益系数
        self.LOITER_TOTAL_STEPS = int((2 * math.pi * self.UAV_MIN_TURN_RADIUS) / self.UAV_TURN_SPEED / self.dt)   # 转弯步数
        
        self.TARGET_TRACKING_DURATION = int(10.0 / self.dt)   # 目标确认时间 10s
        self.TARGET_COOLDOWN_DURATION = int(90.0 / self.dt)   # 探测冷却时间
        self.MAX_USV_ENGAGEMENT_DISTANCE = 1030.0     # 期望最大距离
        self.UAV_SUPPORT_INTERVAL_STEPS = int(90.0 / self.dt)
        
        # --- 航点设置 --- #
        self.waypoints = {
            'P2': np.array([800.0, 5630.0]), 'P3': np.array([800.0, 8000.0]),
            'P4': np.array([4630.0, 8000.0]), 'P5': np.array([4630.0, 6680.0]),
            'TL': np.array([2580.0, 6680.0]), 'Q2': np.array([800.0, 3630.0]),
            'Q3': np.array([800.0, 1260.0]), 'Q4': np.array([4630.0, 1260.0]),
            'Q5': np.array([4630.0, 2580.0]), 'BR': np.array([6680.0, 2580.0]),
            'TR': np.array([6680.0, 6680.0]), 'BL': np.array([2580.0, 2580.0])
        }
        self.loop_points = ['BR', 'TR', 'TL', 'BL']
        
        # 103秒 FOV 覆盖功能开关
        self.enable_fov_coverage = enable_fov_coverage
        
        # 初始化 FOV 管理器
        self.fov_manager = UAVFOVManager(dt=self.dt)

        # UAV 初始任务分配
        self.uav_states = {
            "1": {"state": "INITIAL_MOVE", "path": ['P2', 'P3', 'P4', 'P5', 'TL'], "waypoint_idx": 0, "loiter_steps_left": 0, "loop_idx": 0, "tracking_timer": 0, "support_timer": 0, "target_info": None, "pre_mission_state": None, "tracked_targets_cooldown": {}, "detour_point": None, "detour_mode": False},
            "2": {"state": "INITIAL_MOVE", "path": ['Q2', 'Q3', 'Q4', 'Q5', 'BR'], "waypoint_idx": 0, "loiter_steps_left": 0, "loop_idx": 0, "tracking_timer": 0, "support_timer": 0, "target_info": None, "pre_mission_state": None, "tracked_targets_cooldown": {}, "detour_point": None, "detour_mode": False}
        }
        self.usv_states_ref = None
    
    # USV 信息共享
    def set_usv_states_ref(self, usv_states):
        self.usv_states_ref = usv_states
        self.fov_manager.set_usv_states_ref(usv_states)

    def set_fov_coverage_enabled(self, enabled):
        """动态开关103秒FOV覆盖功能"""
        self.enable_fov_coverage = enabled
        if not enabled:
            # 清理所有detour状态
            for uav_id, state_info in self.uav_states.items():
                state_info["detour_mode"] = False
                state_info["detour_point"] = None
            # 释放FOV管理器中的所有分配
            self.fov_manager.release_all_assignments()

    def _update_target_cooldowns(self, state_info):
        for target_id in list(state_info["tracked_targets_cooldown"].keys()):
            state_info["tracked_targets_cooldown"][target_id] -= 1
            if state_info["tracked_targets_cooldown"][target_id] <= 0:
                del state_info["tracked_targets_cooldown"][target_id]

    def _should_start_tracking(self, target, state_info, manager):
        target_id = target[1]
        
        # 🔥 优先检查：如果目标在USV探测范围内，立即拒绝追踪（防止摆头）
        if self.fov_manager.check_usv_range_exit(target_id, target[2], manager):
            return False
            
        # 检查USV是否已经探测到这个目标
        detected_usv = manager.get_detected_usv()
        if detected_usv and target_id in detected_usv: 
            return False
            
        # 检查目标是否在冷却期
        if target_id in state_info["tracked_targets_cooldown"]: 
            return False
            
        return True

    def _start_mission_interrupt(self, new_state, target, state_info):
        state_info["pre_mission_state"] = {"state": state_info["state"], "waypoint_idx": state_info["waypoint_idx"], "loiter_steps_left": state_info["loiter_steps_left"], "loop_idx": state_info["loop_idx"]}
        state_info["state"] = new_state
        state_info["target_info"] = {"id": target[1], "lkp": np.array(target[2])}
        if new_state == "TRACKING":
            state_info["tracking_timer"] = self.TARGET_TRACKING_DURATION
        elif new_state == "SUPPORTING":
            state_info["support_timer"] = self.UAV_SUPPORT_INTERVAL_STEPS

    def _end_mission_interrupt(self, state_info):
        if state_info["target_info"]:
            target_id = state_info["target_info"]["id"]
            state_info["tracked_targets_cooldown"][target_id] = self.TARGET_COOLDOWN_DURATION
        
        saved_state = state_info["pre_mission_state"]
        if saved_state:
            # 直接恢复保存的状态
            state_info.update(saved_state)
        else:
            # 如果没有保存状态，恢复到正常巡逻
            state_info["state"] = "TRAVELING" 
            state_info["loop_idx"] = 0
            state_info["waypoint_idx"] = 0
            state_info["loiter_steps_left"] = 0
        
        state_info["pre_mission_state"] = None
        state_info["target_info"] = None
        state_info["tracking_timer"] = 0
        state_info["support_timer"] = 0

    def _find_nearest_loop_point(self, current_pos):
        """🔥 智能循环点选择：根据当前位置找到最近的循环点"""
        if not current_pos or len(current_pos) < 2:
            return 0  # 默认返回第一个循环点
            
        min_distance = float('inf')
        nearest_idx = 0
        
        for i, point_name in enumerate(self.loop_points):
            if point_name in self.waypoints:
                point_pos = self.waypoints[point_name]
                distance = np.linalg.norm(np.array(current_pos) - np.array(point_pos))
                if distance < min_distance:
                    min_distance = distance
                    nearest_idx = i
        
        return nearest_idx

    def _calculate_patrol_movement(self, state_info, pos, heading):
        """计算巡逻移动 - 简化版（移除detour逻辑）"""
        current_state = state_info["state"]
        
        # LOITERING状态逻辑
        if current_state == "LOITERING":
            v, omega = self.UAV_TURN_SPEED, -self.UAV_TURN_SPEED / self.UAV_MIN_TURN_RADIUS
            state_info["loiter_steps_left"] -= 1
            if state_info["loiter_steps_left"] <= 0:
                state_info["state"] = "TRAVELING"
                state_info["loop_idx"] = (state_info["loop_idx"] + 1) % len(self.loop_points)
            return v, omega
        
        # 检查FOV detour需求（仅在启用时委托给FOV管理器）
        if self.enable_fov_coverage:
            detour_result = self.fov_manager.handle_detour_logic(state_info, pos, heading)
            if detour_result is not None:
                return detour_result
        
        # 正常巡逻逻辑 - 使用FOV管理器提供的目标点（确保与将军饮马问题一致）
        if current_state == "INITIAL_MOVE":
            if self.enable_fov_coverage:
                target_pos = self.fov_manager._get_current_target_waypoint(state_info)
            else:
                target_pos = self.waypoints[state_info["path"][state_info["waypoint_idx"]]]
            v, omega = self._move_to_waypoint(pos, heading, target_pos, self.UAV_MAX_SPEED)
            if np.linalg.norm(target_pos - pos) < self.ARRIVAL_THRESHOLD:
                if state_info["waypoint_idx"] == len(state_info["path"]) - 1: 
                    state_info["state"] = "TRAVELING"
                    state_info["loop_idx"] = self.loop_points.index(state_info["path"][-1])
                else: 
                    state_info["waypoint_idx"] += 1
                    
        elif current_state == "TRAVELING":
            if self.enable_fov_coverage:
                target_pos = self.fov_manager._get_current_target_waypoint(state_info)
            else:
                target_pos = self.waypoints[self.loop_points[state_info["loop_idx"]]]
            v, omega = self._move_to_waypoint(pos, heading, target_pos, self.UAV_SEARCH_SPEED)
            if np.linalg.norm(target_pos - pos) < self.ARRIVAL_THRESHOLD:
                state_info["state"] = "LOITERING"
                state_info["loiter_steps_left"] = self.LOITER_TOTAL_STEPS
        else:
            v, omega = 0, 0
            
        return v, omega

    def _move_to_waypoint(self, current_pos, current_heading, target_pos, speed):
        target_angle = math.atan2(target_pos[1] - current_pos[1], target_pos[0] - current_pos[0])
        return speed, self.HEADING_KP * pi_to_pi(target_angle - current_heading)

    def fix_patrol_point_conflict(self):
        """每帧全局修正：如两UAV目标点重合，让编号大的自动跳点"""
        uav1 = self.uav_states["1"]
        uav2 = self.uav_states["2"]

        # 获取两个UAV的当前目标点
        tgt1 = self.fov_manager._get_current_target_waypoint(uav1)
        tgt2 = self.fov_manager._get_current_target_waypoint(uav2)

        # 仅在都在巡逻/初始路径/loitering时生效（不干扰追踪/支援等特殊状态）
        patrol_states = ["TRAVELING", "LOITERING", "INITIAL_MOVE"]
        if uav1["state"] not in patrol_states or uav2["state"] not in patrol_states:
            return

        # 检查冲突
        if np.allclose(tgt1, tgt2, atol=1e-3):
            # 优先让2号UAV自动跳到下一个目标点（避免停留）
            uav2_changed = False
            if uav2["state"] == "TRAVELING" or uav2["state"] == "LOITERING":
                # loop_idx推进
                loop_len = len(self.loop_points)
                old_idx = uav2["loop_idx"]
                uav2["loop_idx"] = (old_idx + 1) % loop_len
                uav2["state"] = "TRAVELING"
                uav2["loiter_steps_left"] = 0
                uav2_changed = True
            elif uav2["state"] == "INITIAL_MOVE":
                path = uav2["path"]
                idx = uav2["waypoint_idx"]
                if idx < len(path) - 1:
                    uav2["waypoint_idx"] += 1
                    uav2_changed = True
            # 再次检查，如果还是冲突，继续推进
            if uav2_changed:
                tgt2_new = self.fov_manager._get_current_target_waypoint(uav2)
                if np.allclose(tgt1, tgt2_new, atol=1e-3):
                    # 再推进一格
                    if uav2["state"] == "TRAVELING" or uav2["state"] == "LOITERING":
                        loop_len = len(self.loop_points)
                        uav2["loop_idx"] = (uav2["loop_idx"] + 1) % loop_len
                    elif uav2["state"] == "INITIAL_MOVE":
                        if uav2["waypoint_idx"] < len(path) - 1:
                            uav2["waypoint_idx"] += 1
    def update(self, manager):
        # 仅在启用时委托FOV管理给独立的管理器
        if self.enable_fov_coverage:
            self.fov_manager.update_global_targets(manager)
            self.fov_manager.assign_dynamic_fov_tasks(manager, self.uav_states)
        
        controls, uav_controls = [["uav", "1", 0, 0], ["uav", "2", 0, 0], ["usv", "1", 0, 0], ["usv", "2", 0, 0], ["usv", "3", 0, 0], ["usv", "4", 0, 0]], {}
        
        for uav_id in ["1", "2"]:
            state_info = self.uav_states[uav_id]
            state_info["uav_id"] = uav_id  # 确保uav_id被设置
            self._update_target_cooldowns(state_info)
            
            # 🔥 优先检查：如果当前正在追击的目标进入USV范围，立即停止任务
            if state_info["state"] in ["TRACKING", "SUPPORTING"] and state_info["target_info"]:
                target_id = state_info["target_info"]["id"]
                target_pos = state_info["target_info"]["lkp"]
                if self.fov_manager.check_usv_range_exit(target_id, target_pos, manager):
                    # 目标进入USV 800米范围，立即结束当前任务
                    self._end_mission_interrupt(state_info)
                    # print(f"[强制停止] UAV {uav_id} 停止追击进入USV范围的目标 {target_id}")
            
            vehicle_state = manager.get_state('uav', uav_id)
            if not vehicle_state: continue
            pos, heading = np.array(vehicle_state[0]), vehicle_state[1]
            
            v, omega = 0, 0
            current_state = state_info["state"]

            if current_state == "TRACKING":
                state_info["tracking_timer"] -= 1
                target_id = state_info["target_info"]["id"]
                
                # 检查目标是否已被捕获，如果是则立即结束追踪
                if target_id in manager.get_captured_all():
                    self._end_mission_interrupt(state_info)
                    v, omega = self._calculate_patrol_movement(state_info, pos, heading)
                # 🔥 检查目标是否进入USV范围，如果是则立即停止追踪（目标已从全局列表移除）
                elif self.fov_manager.check_usv_range_exit(target_id, state_info["target_info"]["lkp"], manager):
                    # 目标进入USV范围，立即结束追踪任务
                    self._end_mission_interrupt(state_info)
                    v, omega = self._calculate_patrol_movement(state_info, pos, heading)
                else:
                    v, omega = self._move_to_waypoint(pos, heading, state_info["target_info"]["lkp"], self.UAV_MAX_SPEED)
                    if state_info["tracking_timer"] <= 0:
                        # 检查usv_states_ref是否已初始化
                        if self.usv_states_ref is not None:
                            idle_usv_ids = {uid for uid, info in self.usv_states_ref.items() if info["state"] == "IDLE"}
                            min_dist = float('inf')
                            if idle_usv_ids:
                                for usv_id_idle in idle_usv_ids:
                                   usv_state = manager.get_state('usv', usv_id_idle)
                                   if usv_state and usv_state[0]:
                                       usv_pos = usv_state[0]
                                       dist = np.linalg.norm(np.array(usv_pos) - state_info["target_info"]["lkp"])
                                       if dist < min_dist: min_dist = dist
                            
                            if min_dist > self.MAX_USV_ENGAGEMENT_DISTANCE:
                                state_info["state"] = "SUPPORTING"
                                state_info["support_timer"] = self.UAV_SUPPORT_INTERVAL_STEPS
                            else:
                                self._end_mission_interrupt(state_info)
                        else:
                            # 如果USV状态未初始化，直接结束追踪任务
                            self._end_mission_interrupt(state_info)

            elif current_state == "SUPPORTING":
                state_info["support_timer"] -= 1
                target_id = state_info["target_info"]["id"]
                
                # 检查目标是否已被捕获，如果是则立即结束支援
                if target_id in manager.get_captured_all():
                    self._end_mission_interrupt(state_info)
                    v, omega = self._calculate_patrol_movement(state_info, pos, heading)
                # 🔥 检查目标是否进入USV范围，如果是则立即停止支援（目标已从全局列表移除）
                elif self.fov_manager.check_usv_range_exit(target_id, state_info["target_info"]["lkp"], manager):
                    # 目标进入USV范围，立即结束支援任务
                    self._end_mission_interrupt(state_info)
                    v, omega = self._calculate_patrol_movement(state_info, pos, heading)
                else:
                    detections = manager.get_detected('uav', uav_id)
                    if detections and any(d[1] == target_id for d in detections):
                        self._end_mission_interrupt(state_info)
                        v, omega = self._calculate_patrol_movement(state_info, pos, heading)
                    elif state_info["support_timer"] <= 0:
                        v, omega = self._move_to_waypoint(pos, heading, state_info["target_info"]["lkp"], self.UAV_SEARCH_SPEED)
                    else:
                        v, omega = self._calculate_patrol_movement(state_info, pos, heading)
            else: 
                # 常规巡逻模式下，检查是否发现新目标需要追踪
                detected_targets = manager.get_detected('uav', uav_id)
                if detected_targets and self._should_start_tracking(detected_targets[0], state_info, manager):
                    self._start_mission_interrupt("TRACKING", detected_targets[0], state_info)
                    # 开始追踪后，立即计算追踪移动指令
                    v, omega = self._move_to_waypoint(pos, heading, state_info["target_info"]["lkp"], self.UAV_MAX_SPEED)
                else:
                    # 执行巡逻移动（包含动态detour逻辑）
                    v, omega = self._calculate_patrol_movement(state_info, pos, heading)

            uav_controls[uav_id] = [v, omega]
        self.fix_patrol_point_conflict()

        # 🔥 取消同步检查：移除原有的同步逻辑，UAV各自独立巡逻
        for i, control_item in enumerate(controls):
            if control_item[0] == 'uav':
                if control_item[1] in uav_controls: controls[i][2:4] = uav_controls[control_item[1]]
        return controls

    def get_planning_status(self):
        """获取FOV规划状态 - 委托给FOV管理器"""
        if self.enable_fov_coverage:
            status = self.fov_manager.get_planning_status()
            status["fov_coverage_enabled"] = True
            return status
        else:
            return {
                "fov_coverage_enabled": False,
                "global_targets": 0,
                "urgent_targets": 0,
                "target_details": {}
            }