import os
import sys
import time
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from Vehicle.Uav import Uav
from Vehicle.Usv import Usv
from Vehicle.Target import Target
from Utils.Visualizer import TrajectoryVisualizer2D

# 初始化 UAV（扇形探测）和 USV（圆形探测）
uav = Uav(init_position=[0, 4630], init_theta=0, id='UAV')
usv = Usv(init_position=[0, 4635], init_theta=0, id='USV')

# 初始化一个目标
target = Target(init_position=[3500, 4900], init_theta=0, id='Target1')

# 初始化可视化器（传对象）
visualizer = TrajectoryVisualizer2D(
    labels=["UAV", "USV", "Target1"],
    point_colors=["r", "b", "r"],
    line_colors=["r", "b", "r"],
    markers=["o", "s", "x"],
    trajectory_visible=[False, True, False],
    limits=((0, 9260), (0, 9260))
)

n = 0
while True:
    # 目标缓慢右移
    target.update(0.5, 0)

    # UAV 前进 + 探测目标
    uav.update(0, 0, targets=[target])

    # USV 保持静止（但能展示圆形探测范围）
    usv.update(100, 0.0, targets=[target])

    # 打印被 UAV、USV 探测到的目标
    if uav.detected:
        print(f"[Step {n}] UAV detected:", uav.detected)
        visualizer.point_colors[2] = "g"
    else:
        visualizer.point_colors[2] = "r"
    if usv.detected:
        print(f"[Step {n}] USV detected:", usv.detected)
        visualizer.point_colors[2] = "g"
    else:
        visualizer.point_colors[2] = "r"

    # 更新可视化
    visualizer.update([uav, usv, target], step=n, title="UAV-USV 扇形与圆形探测测试")

    time.sleep(0.05)
    n += 1