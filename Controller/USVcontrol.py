import math
import numpy as np
from Controller.maybe import Maybesomewhere

def pi_to_pi(angle):
    """将角度归一化到 [-pi, pi] 范围内"""
    return (angle + math.pi) % (2 * math.pi) - math.pi

class USVController:
    def __init__(self, dt=0.5):
        self.dt = dt
        self.USV_MAX_SPEED = 10.28
        self.USV_SEARCH_SPEED = 5.0 
        self.DISPOSAL_RANGE = 100.0
        self.ARRIVAL_THRESHOLD = 50.0 
        self.HEADING_KP = 2.0
        self.AVOIDANCE_DISTANCE = 250.0
        self.AVOIDANCE_KP = 5.0
        self.SEARCH_DURATION_STEPS = int(60 / dt) 
        self.SEARCH_TURN_OMEGA = 0.2
        self.history = {}

        self.standby_points = {"1": np.array([8695.0, 8695.0]), "2": np.array([565.0, 8695.0]), "3": np.array([565.0, 565.0]), "4": np.array([8695.0, 565.0])}
        
        self.usv_states = {
            uid: {
                "state": "IDLE", 
                "target_id": None,
                "last_known_pos": None,
                "search_timer": 0,
            } for uid in self.standby_points.keys()
        }

    def _assign_new_tasks(self, manager, usv_positions):
        all_detected_ids = manager.get_detected_all()
        if not all_detected_ids: return

        all_captured_ids = manager.get_captured_all()
        # 允许已分配目标重新分配
        new_task_ids = [tid for tid in all_detected_ids if tid not in all_captured_ids]

        if not new_task_ids: return

        # 所有USV都参与分配（包括IDLE和INTERCEPTING）
        candidate_usv_ids = [uid for uid in self.usv_states.keys()]
        if not candidate_usv_ids: return

        maybe = Maybesomewhere()
        dt = 0.05

        # 记录已分配 USV ID，避免冲突
        assigned_usvs = set()
        assigned_targets = set()

        # 先保存所有目标对应的初始分配信息（包含best_usv_idx等）
        target_assignments = {}

        for target_id in new_task_ids:
            if target_id not in manager.targets: continue
            maybe.update_target_history(manager, self.history, max_history=10)
            pos_history = maybe.get_target_pos_history(self.history, target_id)
            target_velocity = maybe.estimate_velocity(pos_history, dt)
            # print("历史", pos_history)
            target_position = pos_history[-1] if pos_history else manager.targets[target_id].position
            trajectory = maybe.predict_trajectory(target_position, target_velocity, duration=1000, dt=1)
            usv_pos_list = [usv_positions[uid] for uid in candidate_usv_ids]
            usv_speed_list = [self.USV_MAX_SPEED for _ in candidate_usv_ids]

            best_usv_idx, intercept_time = maybe.calculate_fastest_usv(trajectory, usv_pos_list, usv_speed_list, dt=dt)
            target_assignments[target_id] = {
                'trajectory': trajectory,
                'best_usv_idx': best_usv_idx,
                'intercept_time': intercept_time
            }

        # 按时间排序的目标ID列表
        sorted_targets = sorted(target_assignments.items(), key=lambda x: x[1]['intercept_time'])

        final_assignments = []

        for target_id, info in sorted_targets:
            best_usv_idx = info['best_usv_idx']
            if best_usv_idx is None or best_usv_idx < 0 or best_usv_idx >= len(candidate_usv_ids):
                continue

            # 检查该USV是否已被占用
            if best_usv_idx in assigned_usvs:
                # 该USV已被占用，重新计算（剔除已分配USV）
                available_usvs = [uid for idx, uid in enumerate(candidate_usv_ids) if idx not in assigned_usvs]
                if not available_usvs:
                    # 无可用USV，跳过
                    continue

                # 重新计算最快USV，排除已分配USV
                available_pos = [usv_positions[uid] for uid in available_usvs]
                available_speed = [self.USV_MAX_SPEED for _ in available_usvs]

                best_usv_idx_new, intercept_time_new = maybe.calculate_fastest_usv(
                    info['trajectory'], available_pos, available_speed, dt=1)

                if best_usv_idx_new is None or best_usv_idx_new < 0 or best_usv_idx_new >= len(available_usvs):
                    continue

                # 更新 best_usv_idx 和 intercept_time 为重新计算结果
                best_usv_idx = candidate_usv_ids.index(available_usvs[best_usv_idx_new])
                intercept_time = intercept_time_new
            else:
                intercept_time = info['intercept_time']

            # 标记分配
            assigned_usvs.add(best_usv_idx)
            assigned_targets.add(target_id)

            # 计算拦截点
            intercept_step = int(intercept_time)
            trajectory = info['trajectory']
            if intercept_step >= len(trajectory):
                intercept_step = len(trajectory) - 1
            intercept_point = np.array(trajectory[intercept_step])

            final_assignments.append({
                'usv_id': candidate_usv_ids[best_usv_idx],
                'target_id': target_id,
                'lkp': intercept_point,
                'intercept_time': intercept_time
            })

        # 先用字典收集每个目标的最佳分配                                                                        ################################################
        best_assignment_per_target = {}                                                                    ################################################
                                                                                                           ################################################
        for assignment in final_assignments:                                                               ################################################
            target_id = assignment['target_id']                                                            ################################################
            if target_id not in best_assignment_per_target:                                                ############                       #############
                best_assignment_per_target[target_id] = assignment                                         ############    防止目标脚踏两只船    #############
            else:                                                                                          ############                       #############
                # 只保留时间最短的                                                                            ################################################
                if assignment['intercept_time'] < best_assignment_per_target[target_id]['intercept_time']: ################################################
                    best_assignment_per_target[target_id] = assignment                                     ################################################
                                                                                                           ################################################
        # 只留下每个目标最快的USV分配                                                                          ################################################
        final_assignments = list(best_assignment_per_target.values())                                      ################################################

        # 最终更新 usv_states
        for assignment in final_assignments:
            usv_id = assignment['usv_id']
            target_id = assignment['target_id']
            self.usv_states[usv_id].update({
                "state": "INTERCEPTING",
                "target_id": target_id,
                "last_known_pos": assignment['lkp']
            })

    def _update_lkp_from_uav(self, manager):
        all_detected_ids = manager.get_detected_all()
        for state_info in self.usv_states.values():
            if state_info["state"] == "INTERCEPTING":
                target_id = state_info["target_id"]
                if target_id in all_detected_ids and target_id in manager.targets:
                    state_info["last_known_pos"] = np.array(manager.targets[target_id].position)

    def _calculate_avoidance_omega(self, self_id, self_pos, self_heading, usv_positions):
        total_repulsion_vec = np.array([0.0, 0.0])
        for other_id, other_pos in usv_positions.items():
            if self_id == other_id: continue
            dist_vec = self_pos - np.array(other_pos)
            dist = np.linalg.norm(dist_vec)
            if 0 < dist < self.AVOIDANCE_DISTANCE:
                strength = (self.AVOIDANCE_DISTANCE - dist) / self.AVOIDANCE_DISTANCE
                total_repulsion_vec += (dist_vec / dist) * strength
        if np.linalg.norm(total_repulsion_vec) > 0:
            avoidance_angle = math.atan2(total_repulsion_vec[1], total_repulsion_vec[0])
            return self.AVOIDANCE_KP * pi_to_pi(avoidance_angle - self_heading)
        return 0.0

    def update(self, manager):
        all_usvs = [v for (t, _), v in manager.vehicles.items() if t == "usv"]
        all_targets = list(manager.targets.values())
        for usv in all_usvs:
            if hasattr(usv, 'detect'):
                usv.detect(vehicle_position=usv.position, targets=all_targets)
        
        usv_positions = {uid: manager.get_state('usv', uid)[0] for uid in self.usv_states.keys()}
        
        self._update_lkp_from_uav(manager)
        self._assign_new_tasks(manager, usv_positions)

        usv_controls = {}
        for usv_id, state_info in self.usv_states.items():
            pos, heading = np.array(usv_positions[usv_id]), manager.get_state('usv', usv_id)[1]
            v, goal_omega = self.USV_MAX_SPEED, 0.0
            current_state = state_info["state"]

            if current_state == "IDLE":
                goal = self.standby_points[usv_id]
                if np.linalg.norm(goal - pos) < self.ARRIVAL_THRESHOLD: v = 0.0
                else: goal_omega = self.HEADING_KP * pi_to_pi(math.atan2(goal[1] - pos[1], goal[0] - pos[0]) - heading)
            
            elif current_state == "INTERCEPTING":
                target_id = state_info["target_id"]
                if target_id not in manager.targets or target_id in manager.get_captured_all():
                    state_info.update({"state": "RETURNING", "target_id": None, "last_known_pos": None})
                else:
                    goal = state_info["last_known_pos"]
                    if np.linalg.norm(goal - pos) < self.ARRIVAL_THRESHOLD:
                        state_info["state"] = "SEARCHING"
                        state_info["search_timer"] = self.SEARCH_DURATION_STEPS
                    else:
                        goal_omega = self.HEADING_KP * pi_to_pi(math.atan2(goal[1] - pos[1], goal[0] - pos[0]) - heading)
            
            elif current_state == "SEARCHING":
                v, goal_omega = self.USV_SEARCH_SPEED, self.SEARCH_TURN_OMEGA
                state_info["search_timer"] -= 1
                usv_detections = manager.get_detected('usv', usv_id)
                found = any(d[1] == state_info["target_id"] for d in usv_detections) if usv_detections else False
                if found:
                    state_info["state"] = "PURSUING"
                elif state_info["search_timer"] <= 0:
                    state_info.update({"state": "RETURNING", "target_id": None, "last_known_pos": None})

            elif current_state == "PURSUING":
                target_id = state_info["target_id"]
                if target_id not in manager.targets or target_id in manager.get_captured_all():
                    state_info.update({"state": "RETURNING", "target_id": None, "last_known_pos": None})
                else:
                    target_pos = manager.targets[target_id].position
                    if np.linalg.norm(np.array(target_pos) - pos) < self.DISPOSAL_RANGE: v = 0.0
                    else: goal_omega = self.HEADING_KP * pi_to_pi(math.atan2(target_pos[1] - pos[1], target_pos[0] - pos[0]) - heading)
            
            elif current_state == "RETURNING":
                goal = self.standby_points[usv_id]
                if np.linalg.norm(goal - pos) < self.ARRIVAL_THRESHOLD:
                    state_info["state"], v = "IDLE", 0.0
                else:
                    goal_omega = self.HEADING_KP * pi_to_pi(math.atan2(goal[1] - pos[1], goal[0] - pos[0]) - heading)

            avoidance_omega = self._calculate_avoidance_omega(usv_id, pos, heading, usv_positions)
            final_omega = goal_omega + avoidance_omega
            if abs(avoidance_omega) > 0.1: v *= 0.8
            usv_controls[usv_id] = [v, final_omega]
        
        # 清理所有USV的旧分配，只保留本轮分配，防止目标脚踏两只船 
        for usv_id, state_info in self.usv_states.items():
            if state_info["state"] == "INTERCEPTING":
                state_info["target_id"] = None
                state_info["last_known_pos"] = None

        return usv_controls