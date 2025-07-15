import os
import sys
import time
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from Utils.Manager import Manager  # 假设Manager放在Utils文件夹
from Utils.Refresher import TargetRefresher

manager = Manager()
refresher = TargetRefresher(min_interval=200, max_interval=600)

uavs = [
    ["1", [0, 4630], 90],
    ["2", [2000, 4930], 0]
]
usvs = [
    ["1", [0, 4635], 0],
    ["2", [2000, 4935], 0]
]
targets = [
    ['1', [3000, 4600], 0],
    ['2', [2500, 4600], 0]
]
#manager.init_objects(uavs, usvs, refresher.init())
manager.init_objects(uavs, usvs, targets)

for step in range(144000):
    controls = [
        ["uav", "1", 0, 0],
        ["uav", "2", 100, 300],
        ["usv", "1", 0, 0],
        ["usv", "2", 0, 0],
    ]

    for tid in manager.targets:
        controls.append(["target", tid, 20, 0])

    manager.update(controls)

    # 自动刷新目标（id 已内置）
    new_targets = refresher.refresh(step)
    if new_targets:
        print(f"[Step {step}] 刷新目标: {new_targets}")
        manager.add_targets(new_targets)

    # 每5步打印 UAV 和 USV 状态
    
    print(manager.get_detected('uav',2))

    #time.sleep(0.05)
