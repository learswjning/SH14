import time
from Utils.Manager import Manager  
from Utils.Refresher import TargetRefresher

# 初始化系统
manager = Manager()
refresher = TargetRefresher(min_interval=200, max_interval=600)

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

# 初始化对象
manager.init_objects(uavs, usvs, t=0)

# 主仿真循环
max_step = 144000
log_interval = 1
target_refresh_enabled = True
capture_count = 0

for step in range(max_step):
    # 控制信息（模拟简单控制）
    controls = [
        ["uav", "1", 0, 0],
        ["uav", "2", 0, 0],
        ["usv", "1", 100, 0],
        ["usv", "2", 0, 0],
    ]

    # 所有目标以固定速度前进
    for tid in list(manager.targets.keys()):
        controls.append(["target", tid, 20, 0])

    # 执行更新
    manager.update(controls, t=step)  # 时间步长0.05s

    # 自动刷新目标
    if target_refresh_enabled:
        new_targets = refresher.refresh(step)
        if new_targets:
            print(f"[Step {step}] 新目标加入: {new_targets}")
            manager.add_targets(new_targets, step)

    # 每 log_interval 步打印一次系统状态
    if step % log_interval == 0:
        print(f"\n=== Step {step} 状态 ===")
        for uid in ['1', '2']:
            detected = manager.get_detected('uav', uid)
            print(f"UAV {uid} 探测到目标: {detected}")
        for uid in ['1', '2']:
            captured = manager.get_captured('usv', uid)
            print(f"USV {uid} 捕获的目标: {captured}")
        print(f"当前剩余目标数: {len(manager.targets)}")
    print(f"探测时间记录: {manager.time1}")
    print(f"捕获时间记录: {manager.time2}")
    
    time.sleep(0.05)