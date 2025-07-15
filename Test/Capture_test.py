import os
import sys
import time
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from Vehicle.Uav import Uav
from Vehicle.Usv import Usv
from Vehicle.Target import Target
from Utils.Visualizer import TrajectoryVisualizer2D

# 初始化 UAV（扇形探测）和 USV（圆形探测）
uav1 = Uav(init_position=[0, 4630], init_theta=0, id='1')
usv1 = Usv(init_position=[0, 4635], init_theta=0, id='1')
uav2 = Uav(init_position=[2000, 4630], init_theta=0, id='2')
usv2 = Usv(init_position=[2000, 4935], init_theta=0, id='2')

# 初始化一个目标
target = Target(init_position=[4000, 4635], init_theta=0, id='1')

# 初始化可视化器（传对象）
visualizer = TrajectoryVisualizer2D(
    labels=["UAV1", "UAV2", "USV1", "USV2", "Target1"],
    point_colors=["r", "r", "b", "b", "r"],
    line_colors=["r", "r", "b", "b", "r"],
    markers=["o", "o", "s", "s", "x"],
    trajectory_visible=[False, False, False, False, False],
    limits=((0, 9260), (0, 9260))
)

n = 0
while True:
    # 目标缓慢右移
    target.update(0.5, 0)

    # UAV 前进 + 探测目标
    uav1.update(0, 0, uavs=[uav1,uav2], usvs=[usv1,usv2], targets=[target])
    uav2.update(0, 0, uavs=[uav1,uav2], usvs=[usv1,usv2], targets=[target])

    # USV 保持静止（但能展示圆形探测范围）
    usv1.update(100, 0.0, uavs=[uav1,uav2], usvs=[usv1,usv2], targets=[target])
    usv2.update(0, 0.0, uavs=[uav1,uav2], usvs=[usv1,usv2], targets=[target])

    # 打印被 UAV、USV 探测到的目标
    if uav1.detected:
        print(f"[Step {n}] UAV1 detected:", uav1.detected)
        visualizer.point_colors[4] = "g"
    else:
        visualizer.point_colors[4] = "r"
    if uav2.detected:
        print(f"[Step {n}] UAV2 detected:", uav2.detected)
        visualizer.point_colors[4] = "g"
    else:
        visualizer.point_colors[4] = "r"
    if usv1.detected:
        print(f"[Step {n}] USV1 detected:", usv1.detected)
        visualizer.point_colors[4] = "g"
    else:
        visualizer.point_colors[4] = "r"
    if usv2.detected:
        print(f"[Step {n}] USV2 detected:", usv2.detected)
        visualizer.point_colors[4] = "g"
    else:
        visualizer.point_colors[4] = "r"
    if usv1.captured:
        print(f"[Step {n}] USV1 captured:", usv1.captured)
        target.position = [10000, 10000]
    if usv2.captured:
        print(f"[Step {n}] USV2 captured:", usv2.captured)

    # 更新可视化
    visualizer.update([uav1, uav2, usv1, usv2, target], step=n, title="UAV-USV 扇形与圆形探测测试")

    time.sleep(0.05)
    n += 1