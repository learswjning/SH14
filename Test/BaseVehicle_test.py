import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Vehicle.BaseVehicle import BaseVehicle
from Utils.Visualizer import TrajectoryVisualizer2D

vehicle = BaseVehicle(init_position=[0,0], init_theta=0, max_turn_radius=100, dt=0.05)
visualizer = TrajectoryVisualizer2D(
    labels=["BaseVehicle"],
    point_colors=["r"],
    line_colors=["r"],
    markers=["o"],
    limits=((-9260, 9260), (-9260, 9260))
)

n = 0

while True:
    vehicle.update(100, 300)
    all_positions = [vehicle.position]
    # 更新可视化
    visualizer.update(all_positions.copy(), step=n, title="UAV-USV collaboration")