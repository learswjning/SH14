from Utils.Manager import Manager
from Utils.Refresher import TargetRefresher
from Utils.Scorer import score1, score2

# 初始化系统
manager = Manager()
refresher = TargetRefresher()
init_step = 12000

# UAV 和 USV 初始配置
uavs = [
    ["1", [0, 5630], 0],
    ["2", [0, 3630], 0]
]
usvs = [
    ["1", [0, 6130], 0],
    ["2", [0, 5130], 0],
    ["3", [0, 4130], 0],
    ["4", [0, 3130], 0]
]
targets = ['1', '2']

# 初始化对象
manager.init_objects(uavs, usvs)

# 主仿真循环
max_step = 144000

for step in range(max_step):
    
    # 初始化 Target，可修改
    if step == init_step:
        manager.update_targets(targets, t=step)
    
    # 控制信息（模拟简单控制）
    controls = [
        ["uav", "1", 50, 0], # 类型、编号、线速度、角速度
        ["uav", "2", 50, 0],
        ["usv", "1", 0, 0],
        ["usv", "2", 0, 0],
        ["usv", "3", 0, 0],
        ["usv", "4", 0, 0],
    ]
    for tid in list(manager.targets.keys()):
        controls.append(["target", tid, 0, 0])
        
    # 执行更新
    manager.update(controls, t=step)  # 时间步长0.05s
    new_target_list = refresher.refresh(step)
    manager.update_targets(new_target_list, t=step)

    print(f"\n=== Step {step} 状态 ===")
    for uid in ['1', '2']:
        # detected = manager.get_detected('uav', uid)
        print(f"UAV {uid} 探测到目标: {manager.get_detected('uav', uid)}")
    for uid in ['1', '2', '3', '4']:
        # detected1 = manager.get_detected('usv', uid)
        # captured = manager.get_captured('usv', uid)
        print(f"USV {uid} 探测到目标: {manager.get_detected('usv', uid)}")
        print(f"USV {uid} 捕获的目标: {manager.get_captured('usv', uid)}")
    print(f"总探测目标列表: {manager.get_detected_all()}")
    print(f"总捕获目标列表: {manager.get_captured_all()}")
    # print(f"当前剩余目标数: {len(manager.targets)}")
    # print(f"探测时间记录: {manager.time1}")
    # print(f"捕获时间记录: {manager.time2}")
    P = len(manager.time1) / (refresher.current_id - 1)
    S1 = score1(manager.time1)
    S2 = score2(manager.time2)
    print(f"P = {P}, S1 = {S1}, S2 = {S2}, Total = {P * (S1 + S2)}")
