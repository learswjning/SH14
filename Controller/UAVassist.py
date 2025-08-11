"""
UAV FOV 管理模块 - 优化版本
负责处理视野覆盖、目标追踪和与USV的协调
"""
import math
import numpy as np
from Module.Sensor_Uav import Sensor


def pi_to_pi(angle):
    """将角度归一化到 [-pi, pi] 范围内"""
    return (angle + math.pi) % (2 * math.pi) - math.pi


class UAVassist:
    # 类级常量 - 减少重复定义
    WAYPOINTS = {
        'BR': np.array([6680.0, 2580.0]), 'TR': np.array([6680.0, 6680.0]),
        'TL': np.array([2580.0, 6680.0]), 'BL': np.array([2580.0, 2580.0]),
        'P2': np.array([800.0, 5630.0]), 'P3': np.array([800.0, 8000.0]),
        'P4': np.array([4630.0, 8000.0]), 'P5': np.array([4630.0, 6680.0]),
        'Q2': np.array([800.0, 3630.0]), 'Q3': np.array([800.0, 1260.0]),
        'Q4': np.array([4630.0, 1260.0]), 'Q5': np.array([4630.0, 2580.0])
    }
    LOOP_POINTS = ['BR', 'TR', 'TL', 'BL']
    BOUNDARY = {'x': (500.0, 8760.0), 'y': (500.0, 8760.0)}
    
    def __init__(self, dt=0.5):
        self.dt = dt
        self.FOV_RANGE = 3000.0
        self.FOV_ANGLE = math.radians(30)
        self.EARLY_PLANNING_STEPS = int(60.0 / dt)  # 60秒触发
        self.UAV_SEARCH_SPEED = 33.3
        self.ARRIVAL_THRESHOLD = 50.0
        
        # 状态管理
        self.global_targets = {}
        self.current_step = 0
        self.usv_states_ref = None
        
        # 传感器
        self.sensor = Sensor(detection_range=self.FOV_RANGE, fov_deg=60)

    def set_usv_states_ref(self, usv_states):
        """设置USV状态引用"""
        self.usv_states_ref = usv_states

    def is_target_in_fov(self, uav_pos, uav_heading, target_pos):
        """判断目标是否在FOV范围内"""
        rel_vec = np.array(target_pos) - np.array(uav_pos)
        dist = np.linalg.norm(rel_vec)
        
        if dist > self.FOV_RANGE or dist == 0:
            return False
            
        uav_dir = np.array([np.cos(uav_heading), np.sin(uav_heading)])
        rel_dir = rel_vec / dist
        angle = np.arccos(np.clip(np.dot(rel_dir, uav_dir), -1.0, 1.0))
        
        return angle <= self.FOV_ANGLE / 2

    def check_usv_range_exit(self, target_id, target_pos, manager):
        """检查目标是否进入USV处置范围"""
        if not self.usv_states_ref:
            return False
        
        # 检查USV分配和探测范围
        for usv_id, usv_state in self.usv_states_ref.items():
            if usv_state.get("target_id") == target_id:
                return True
            
            if usv_state["state"] in ["IDLE", "INTERCEPTING", "SEARCHING", "PURSUING", "RETURNING"]:
                usv_vehicle_state = manager.get_state('usv', usv_id)
                if usv_vehicle_state and usv_vehicle_state[0]:
                    usv_pos = usv_vehicle_state[0]
                    if np.linalg.norm(np.array(target_pos) - np.array(usv_pos)) <= 800.0:
                        return True
        
        # 检查USV探测状态
        for usv_id in self.usv_states_ref.keys():
            usv_detections = manager.get_detected('usv', usv_id)
            if usv_detections and any(target_id == str(d[1]) for d in usv_detections):
                return True
        
        return False

    def update_global_targets(self, manager):
        """更新全局目标状态 - 优化版本"""
        self.current_step += 1
        captured_targets = set(manager.get_captured_all())
        
        # 清理无效目标
        for target_id in list(self.global_targets.keys()):
            if (target_id in captured_targets or 
                target_id not in manager.targets or
                self.check_usv_range_exit(target_id, manager.targets[target_id].position, manager)):
                del self.global_targets[target_id]
                continue
            # 更新位置
            self.global_targets[target_id]["position"] = manager.targets[target_id].position
        
        # 收集当前FOV目标
        current_fov_targets = self._collect_fov_targets(manager)
        
        # 更新FOV状态
        for target_id, target_info in self.global_targets.items():
            was_in_fov = target_info.get("in_fov", False)
            is_in_fov = target_id in current_fov_targets
            
            if is_in_fov:
                target_info.update({"last_exit_fov_time": self.current_step, "in_fov": True, "assigned_uav": None})
            elif was_in_fov and not is_in_fov:
                target_info.update({"last_exit_fov_time": self.current_step, "in_fov": False})
        
        # 添加新目标
        for target_id in manager.get_detected_all():
            target_id = str(target_id)
            if (target_id not in self.global_targets and 
                target_id in manager.targets and 
                target_id not in captured_targets):
                
                target_pos = manager.targets[target_id].position
                if not self.check_usv_range_exit(target_id, target_pos, manager):
                    is_in_fov = target_id in current_fov_targets
                    self.global_targets[target_id] = {
                        "last_exit_fov_time": self.current_step,
                        "position": target_pos,
                        "assigned_uav": None,
                        "priority": 0.0,
                        "in_fov": is_in_fov
                    }
        
        # 更新优先级
        for target_info in self.global_targets.values():
            if target_info.get("in_fov", False):
                target_info["priority"] = 0.0
            else:
                target_info["priority"] = self.current_step - target_info["last_exit_fov_time"]

    def _collect_fov_targets(self, manager):
        """收集当前FOV覆盖的目标 - 优化版本"""
        current_fov_targets = {}
        
        for uav_id in ["1", "2"]:
            detections = manager.get_detected('uav', uav_id)
            uav_state = manager.get_state('uav', uav_id)
            
            if detections and uav_state:
                uav_pos, uav_heading = uav_state
                for detection in detections:
                    target_id = str(detection[1])
                    if (target_id in manager.targets and 
                        self.is_target_in_fov(uav_pos, uav_heading, manager.targets[target_id].position)):
                        current_fov_targets.setdefault(target_id, set()).add(uav_id)
        
        return current_fov_targets

    def assign_dynamic_fov_tasks(self, manager, uav_states):
        """动态分配FOV覆盖任务 - 优化版本"""
        # 收集状态信息
        assigned_uavs, assigned_targets = self._collect_assignment_state(uav_states)
        
        # 路径点触发检查
        self._check_waypoint_triggers(manager, uav_states, assigned_uavs, assigned_targets)
        
        # 紧急目标分配
        urgent_targets = self._get_urgent_targets()
        for target_id, target_info, urgency in urgent_targets:
            # 关键：如果该目标已经被分配给某个UAV，则跳过
            if target_id in assigned_targets or self.check_usv_range_exit(target_id, target_info["position"], manager):
                continue
                
            # 可用UAV列表（未分配且未在detour）
            available_uavs = [uid for uid in ["1", "2"] 
                            if uid not in assigned_uavs and not uav_states.get(uid, {}).get("detour_mode", False)]
            
            if available_uavs:
                best_uav = min(available_uavs, 
                            key=lambda uid: self._calculate_assignment_cost(uid, target_info["position"], manager, uav_states))
                
                cost = self._calculate_assignment_cost(best_uav, target_info["position"], manager, uav_states)
                if cost < 15000:
                    # 分配目标给UAV
                    target_info["assigned_uav"] = best_uav
                    assigned_uavs.add(best_uav)
                    assigned_targets.add(target_id)
                    self.plan_detour_for_fov(best_uav, target_info["position"], manager, uav_states)

    def _collect_assignment_state(self, uav_states):
        """收集当前分配状态"""
        assigned_uavs = set()
        assigned_targets = set()
        
        for target_info in self.global_targets.values():
            if target_info.get("assigned_uav"):
                assigned_uavs.add(target_info["assigned_uav"])
        
        for uav_id, uav_state in uav_states.items():
            if uav_state.get("detour_mode", False):
                assigned_uavs.add(uav_id)
        
        for target_id, target_info in self.global_targets.items():
            if target_info.get("assigned_uav"):
                assigned_targets.add(target_id)
        
        return assigned_uavs, assigned_targets

    def _check_waypoint_triggers(self, manager, uav_states, assigned_uavs, assigned_targets):
        """检查路径点触发"""
        for uav_id, state_info in uav_states.items():
            if (state_info["state"] in ["TRAVELING", "LOITERING"] and 
                not state_info.get("detour_mode", False) and 
                uav_id not in assigned_uavs):
                
                for target_id, target_info in self.global_targets.items():
                    if (not target_info.get("in_fov", False) and 
                        target_info.get("assigned_uav") is None and
                        target_id not in assigned_targets):
                        
                        time_since_exit = self.current_step - target_info["last_exit_fov_time"]
                        
                        if (30 * (1/self.dt) <= time_since_exit < self.EARLY_PLANNING_STEPS and
                            self._is_uav_near_waypoint(uav_id, manager, state_info)):
                            
                            target_info["assigned_uav"] = uav_id
                            assigned_uavs.add(uav_id)
                            assigned_targets.add(target_id)
                            self.plan_detour_for_fov(uav_id, target_info["position"], manager, uav_states)
                            break

    def _get_urgent_targets(self):
        """获取紧急目标"""
        return sorted([
            (target_id, target_info, self.current_step - target_info["last_exit_fov_time"])
            for target_id, target_info in self.global_targets.items()
            if (not target_info.get("in_fov", False) and 
                target_info.get("assigned_uav") is None and
                self.current_step - target_info["last_exit_fov_time"] >= self.EARLY_PLANNING_STEPS)
        ], key=lambda x: x[2], reverse=True)

    def _is_uav_near_waypoint(self, uav_id, manager, state_info):
        """检查UAV是否接近路径点"""
        vehicle_state = manager.get_state('uav', uav_id)
        if not vehicle_state:
            return False
            
        pos = np.array(vehicle_state[0])
        target_waypoint = self._get_target_waypoint(state_info)
        
        return target_waypoint is not None and np.linalg.norm(pos - target_waypoint) < 500.0

    def _get_target_waypoint(self, state_info):
        """获取目标路径点 - 统一处理"""
        if state_info["state"] == "TRAVELING":
            loop_idx = state_info.get("loop_idx", 0)
            return self.WAYPOINTS[self.LOOP_POINTS[loop_idx]]
            
        elif state_info["state"] == "INITIAL_MOVE":
            path = state_info["path"]
            waypoint_idx = state_info.get("waypoint_idx", 0)
            if waypoint_idx < len(path):
                return self.WAYPOINTS[path[waypoint_idx]]
        
        return None

    def _calculate_assignment_cost(self, uav_id, target_pos, manager, uav_states):
        """计算分配成本 - 简化版本"""
        vehicle_state = manager.get_state('uav', uav_id)
        if not vehicle_state:
            return float('inf')
            
        pos = np.array(vehicle_state[0])
        heading = vehicle_state[1]
        
        if self.is_target_in_fov(pos, heading, target_pos):
            return np.linalg.norm(pos - np.array(target_pos)) * 0.1
        
        return_point = self._get_optimal_return_point(uav_id, uav_states)
        optimal_point = self._solve_optimal_coverage_point(pos, np.array(target_pos), return_point)
        
        # 简化成本计算
        cost_to_coverage = np.linalg.norm(optimal_point - pos)
        cost_from_coverage = np.linalg.norm(return_point - optimal_point) * 0.3
        
        # 转向惩罚
        current_dir = np.array([np.cos(heading), np.sin(heading)])
        to_coverage_dir = optimal_point - pos
        if np.linalg.norm(to_coverage_dir) > 0:
            to_coverage_dir = to_coverage_dir / np.linalg.norm(to_coverage_dir)
            angle_change = np.arccos(np.clip(np.dot(current_dir, to_coverage_dir), -1.0, 1.0))
            turning_penalty = angle_change * 200.0
        else:
            turning_penalty = 0.0
        
        return cost_to_coverage + cost_from_coverage + turning_penalty

    def plan_detour_for_fov(self, uav_id, target_pos, manager, uav_states):
        """规划FOV覆盖路径"""
        vehicle_state = manager.get_state('uav', uav_id)
        if not vehicle_state or uav_states[uav_id].get("detour_mode", False):
            return
            
        pos = np.array(vehicle_state[0])
        if self.is_target_in_fov(pos, vehicle_state[1], target_pos):
            return
        
        return_point = self._get_optimal_return_point(uav_id, uav_states)
        optimal_point = self._solve_optimal_coverage_point(pos, np.array(target_pos), return_point)
        
        uav_states[uav_id]["detour_point"] = optimal_point
        uav_states[uav_id]["detour_mode"] = True

    def _get_optimal_return_point(self, uav_id, uav_states):
        """
        获取最优返回点（支持距离和相邻判定，近则直接用下一个目标点或不重复目标点）
        增加：可选跳点后与另一个UAV目标点相同则不跳
        """
        state_info = uav_states[uav_id]
        # 支持loop模式
        if state_info["state"] in ["TRAVELING", "LOITERING"]:
            loop_idx = state_info.get("loop_idx", 0)
            cur_wp = self.WAYPOINTS[self.LOOP_POINTS[loop_idx]]
            next_idx = (loop_idx + 1) % len(self.LOOP_POINTS)
            next_wp = self.WAYPOINTS[self.LOOP_POINTS[next_idx]]

            # 判断另一个无人机目标点是否相邻或同点
            other_id = "2" if uav_id == "1" else "1"
            other_state = uav_states.get(other_id, None)
            adjacent_or_same = False
            dist_adjacent = False
            next_conflict = False
            if other_state:
                other_idx = other_state.get("loop_idx", 0)
                # 是否同点
                if loop_idx == other_idx:
                    adjacent_or_same = True
                # 是否相邻(循环)
                elif abs(loop_idx - other_idx) % len(self.LOOP_POINTS) == 1:
                    adjacent_or_same = True
                # 物理距离<1500
                other_wp = self.WAYPOINTS[self.LOOP_POINTS[other_idx]]
                if np.linalg.norm(cur_wp - other_wp) < 1500:
                    dist_adjacent = True
                # 跳点后是否和另一个UAV目标点一样
                if np.allclose(next_wp, other_wp, atol=1e-3):
                    next_conflict = True

            detour_point = state_info.get("detour_point")
            if ((detour_point is not None and np.linalg.norm(cur_wp - detour_point) < 1500)
                or adjacent_or_same or dist_adjacent):
                # 如果跳点后与另一个UAV目标点相同则不跳
                if next_conflict:
                    return cur_wp
                else:
                    return next_wp
            else:
                return cur_wp

        # 支持waypoint模式
        elif state_info["state"] == "INITIAL_MOVE":
            path = state_info["path"]
            waypoint_idx = state_info.get("waypoint_idx", 0)
            cur_wp = self.WAYPOINTS[path[waypoint_idx]]
            next_idx = waypoint_idx + 1
            if next_idx < len(path):
                next_wp = self.WAYPOINTS[path[next_idx]]

                # 检查另一个无人机
                other_id = "2" if uav_id == "1" else "1"
                other_state = uav_states.get(other_id, None)
                adjacent_or_same = False
                dist_adjacent = False
                next_conflict = False
                if other_state and other_state.get("state") == "INITIAL_MOVE":
                    other_path = other_state["path"]
                    other_idx = other_state.get("waypoint_idx", 0)
                    if path[waypoint_idx] == other_path[other_idx]:
                        adjacent_or_same = True
                    elif abs(waypoint_idx - other_idx) == 1:
                        adjacent_or_same = True
                    other_wp = self.WAYPOINTS[other_path[other_idx]]
                    if np.linalg.norm(cur_wp - other_wp) < 1500:
                        dist_adjacent = True
                    # 跳点后是否和另一个UAV目标点一样
                    if np.allclose(next_wp, other_wp, atol=1e-3):
                        next_conflict = True

                detour_point = state_info.get("detour_point")
                if ((detour_point is not None and np.linalg.norm(cur_wp - detour_point) < 1500)
                    or adjacent_or_same or dist_adjacent):
                    if next_conflict:
                        return cur_wp
                    else:
                        return next_wp
                else:
                    return cur_wp
            else:
                return self.WAYPOINTS[path[-1]]
        # fallback
        return np.array([4630.0, 4630.0])
    

    
    def _solve_optimal_coverage_point(self, start_pos, target_pos, return_pos):
        """求解最优覆盖点 - 简化的将军饮马问题"""
        target_center = np.array(target_pos)
        dist_to_target = np.linalg.norm(start_pos - target_center)
        
        # 如果已在FOV范围内
        if dist_to_target <= self.FOV_RANGE:
            to_return = return_pos - target_center
            to_start = start_pos - target_center
            
            if np.linalg.norm(to_return) > 0 and np.linalg.norm(to_start) > 0:
                to_return_norm = to_return / np.linalg.norm(to_return)
                to_start_norm = to_start / np.linalg.norm(to_start)
                optimal_dir = 0.7 * to_return_norm + 0.3 * to_start_norm
                optimal_dir = optimal_dir / np.linalg.norm(optimal_dir)
                optimal_point = target_center + optimal_dir * self.FOV_RANGE * 0.85
            else:
                optimal_point = target_center + np.array([self.FOV_RANGE * 0.8, 0])
            
            return self._apply_boundary_constraints(optimal_point)
        
        # 标准情况：计算切点
        return self._find_optimal_tangent_point(start_pos, target_center, return_pos)

    def _find_optimal_tangent_point(self, start_pos, circle_center, return_pos):
        """找到最优切点 - 简化版本"""
        to_center = circle_center - start_pos
        dist_to_center = np.linalg.norm(to_center)
        
        if dist_to_center <= self.FOV_RANGE:
            direction = to_center / dist_to_center if dist_to_center > 0 else np.array([1.0, 0.0])
            return circle_center + direction * self.FOV_RANGE * 0.9
        
        # 计算切点
        tangent_length = math.sqrt(dist_to_center**2 - self.FOV_RANGE**2)
        sin_alpha = self.FOV_RANGE / dist_to_center
        cos_alpha = tangent_length / dist_to_center
        
        center_direction = to_center / dist_to_center
        perpendicular = np.array([-center_direction[1], center_direction[0]])
        
        # 两个切点
        tangent1 = circle_center + self.FOV_RANGE * (cos_alpha * center_direction + sin_alpha * perpendicular)
        tangent2 = circle_center + self.FOV_RANGE * (cos_alpha * center_direction - sin_alpha * perpendicular)
        
        # 选择总路径最短的切点
        dist1 = np.linalg.norm(tangent1 - start_pos) + np.linalg.norm(return_pos - tangent1)
        dist2 = np.linalg.norm(tangent2 - start_pos) + np.linalg.norm(return_pos - tangent2)
        
        optimal_point = tangent1 if dist1 < dist2 else tangent2
        return self._apply_boundary_constraints(optimal_point)

    def _apply_boundary_constraints(self, point):
        """应用边界约束"""
        return np.array([
            max(self.BOUNDARY['x'][0], min(self.BOUNDARY['x'][1], point[0])),
            max(self.BOUNDARY['y'][0], min(self.BOUNDARY['y'][1], point[1]))
        ])

    def handle_detour_logic(self, state_info, pos, heading):
        """处理detour逻辑 - detour点到达后持续等待目标进入FOV再释放"""
        if not (state_info.get("detour_mode", False) and state_info.get("detour_point") is not None):
            return None

        detour_point = state_info["detour_point"]
        uav_id = state_info.get("uav_id")

        # 检查任务完成——只有探测到目标才算完成
        for target_id, target_info in self.global_targets.items():
            if target_info.get("assigned_uav") == uav_id:
                if self.is_target_in_fov(pos, heading, target_info["position"]):
                    # 任务完成
                    target_info.update({
                        "last_exit_fov_time": self.current_step,
                        "in_fov": True,
                        "assigned_uav": None
                    })
                    self._complete_detour(state_info, uav_id)
                    state_info.pop("detour_turn_sign", None)
                    return None
                break
        else:
            # 没有找到分配的目标
            self._complete_detour(state_info, uav_id)
            state_info.pop("detour_turn_sign", None)
            return None

        distance_to_detour = np.linalg.norm(detour_point - pos)
        # 到达detour点后，如果目标还没进FOV，原地小转圈等待
        if distance_to_detour < self.ARRIVAL_THRESHOLD:
            v = 1 * self.UAV_SEARCH_SPEED  # 低速
            # 方向可以继续用detour_turn_sign来保证旋转方向一致
            if "detour_turn_sign" not in state_info:
                state_info["detour_turn_sign"] = 1  # 默认顺时针
            omega = 0.5 * state_info["detour_turn_sign"]  # 小角速度
            return v, omega

        # detour开始时锁定方向，只赋值一次
        if "detour_turn_sign" not in state_info:
            target_angle = math.atan2(detour_point[1] - pos[1], detour_point[0] - pos[0])
            angle_error = pi_to_pi(target_angle - heading)
            state_info["detour_turn_sign"] = 1 if angle_error >= 0 else -1

        v, omega = self._move_to_waypoint(pos, heading, detour_point)
        omega = abs(omega) * state_info["detour_turn_sign"]
        return v, omega

    def _complete_detour(self, state_info, uav_id, uav_states=None):
        # 释放分配
        for target_info in self.global_targets.values():
            if target_info.get("assigned_uav") == uav_id:
                target_info["assigned_uav"] = None

        cur_state = state_info["state"]
        detour_point = state_info.get("detour_point")
        ARRIVAL_THRESHOLD = getattr(self, 'ARRIVAL_THRESHOLD', 50.0)  # 容错

        # 支持loop模式
        if cur_state in ["TRAVELING", "LOITERING"] and detour_point is not None:
            loop_idx = state_info.get("loop_idx", 0)
            cur_wp = self.WAYPOINTS[self.LOOP_POINTS[loop_idx]]
            next_idx = (loop_idx + 1) % len(self.LOOP_POINTS)
            new_wp = self.WAYPOINTS[self.LOOP_POINTS[next_idx]]

            # 判断相邻/同点/距离条件
            adjacent_or_same = False
            dist_adjacent = False
            next_conflict = False
            if uav_states:
                other_id = "2" if uav_id == "1" else "1"
                other_state = uav_states.get(other_id, None)
                if other_state:
                    other_idx = other_state.get("loop_idx", 0)
                    if loop_idx == other_idx:
                        adjacent_or_same = True
                    elif abs(loop_idx - other_idx) % len(self.LOOP_POINTS) == 1:
                        adjacent_or_same = True
                    other_wp = self.WAYPOINTS[self.LOOP_POINTS[other_idx]]
                    if np.linalg.norm(cur_wp - other_wp) < 1500:
                        dist_adjacent = True
                    # 跳点后是否和另一个UAV目标点一样
                    if np.allclose(new_wp, other_wp, atol=1e-3):
                        next_conflict = True

            if (np.linalg.norm(cur_wp - detour_point) < 1500) or adjacent_or_same or dist_adjacent:
                # 如果跳点后与另一个UAV目标点相同则不跳
                if not next_conflict:
                    state_info["loop_idx"] = next_idx
                    # 如果当前位置也极接近新目标点，直接进入 loiter
                    if np.linalg.norm(new_wp - detour_point) < ARRIVAL_THRESHOLD:
                        state_info["state"] = "LOITERING"
                        state_info["loiter_steps_left"] = 60  # 你可以替换为动态步数

        # 支持waypoint模式
        elif cur_state == "INITIAL_MOVE" and detour_point is not None:
            path = state_info["path"]
            waypoint_idx = state_info.get("waypoint_idx", 0)
            cur_wp = self.WAYPOINTS[path[waypoint_idx]]
            next_idx = waypoint_idx + 1
            if next_idx < len(path):
                new_wp = self.WAYPOINTS[path[next_idx]]
                adjacent_or_same = False
                dist_adjacent = False
                next_conflict = False
                if uav_states:
                    other_id = "2" if uav_id == "1" else "1"
                    other_state = uav_states.get(other_id, None)
                    if other_state and other_state.get("state") == "INITIAL_MOVE":
                        other_path = other_state["path"]
                        other_idx = other_state.get("waypoint_idx", 0)
                        if path[waypoint_idx] == other_path[other_idx]:
                            adjacent_or_same = True
                        elif abs(waypoint_idx - other_idx) == 1:
                            adjacent_or_same = True
                        other_wp = self.WAYPOINTS[other_path[other_idx]]
                        if np.linalg.norm(cur_wp - other_wp) < 1500:
                            dist_adjacent = True
                        # 跳点后与另一个UAV目标点相同判断
                        if np.allclose(new_wp, other_wp, atol=1e-3):
                            next_conflict = True
                if (np.linalg.norm(cur_wp - detour_point) < 1500) or adjacent_or_same or dist_adjacent:
                    if not next_conflict:
                        state_info["waypoint_idx"] += 1
                        # 如果当前位置也极接近新目标点，直接进入 loiter
                        if np.linalg.norm(new_wp - detour_point) < ARRIVAL_THRESHOLD:
                            state_info["state"] = "LOITERING"
                            state_info["loiter_steps_left"] = 60

        state_info["detour_mode"] = False
        state_info["detour_point"] = None

    def _move_to_waypoint(self, current_pos, current_heading, target_pos):
        """移动到目标点 - FOV detour允许顺/逆时针自由选择，始终走最短路径"""
        target_angle = math.atan2(target_pos[1] - current_pos[1], target_pos[0] - current_pos[0])
        angle_diff = pi_to_pi(target_angle - current_heading)
        # 不做符号限制，允许正负，FOV detour不受control影响
        return self.UAV_SEARCH_SPEED, angle_diff

    def _get_current_target_waypoint(self, state_info):
        """获取UAV当前应该前往的目标路径点"""
        if state_info["state"] == "TRAVELING":
            loop_idx = state_info.get("loop_idx", 0)
            return self.WAYPOINTS[self.LOOP_POINTS[loop_idx]]
            
        elif state_info["state"] == "LOITERING":
            # loitering状态下继续在当前循环点
            loop_idx = state_info.get("loop_idx", 0)
            return self.WAYPOINTS[self.LOOP_POINTS[loop_idx]]
            
        elif state_info["state"] == "INITIAL_MOVE":
            path = state_info["path"]
            waypoint_idx = state_info.get("waypoint_idx", 0)
            if waypoint_idx < len(path):
                return self.WAYPOINTS[path[waypoint_idx]]
        
        # 默认返回中心点
        return np.array([4630.0, 4630.0])