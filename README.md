#### 一、启动步骤

1、运行 `init.py`，随机初始化洋流轨迹

```bash
python3 init.py
```

2、运行 `main.py`，启动仿真环境

```bash
python3 main.py
```

---

#### 二、项目结构

```bash
SH14
├── init.py		# 初始化程序
├── main.py		# 主程序
├── README.md
│
├── Controller	# 控制库 TODO
│   └── control.py		    # UAV、USV 控制模块 TODO
│
├── Module		# 基本物理库
│   ├── Capture_Usv.py		# USV 捕获模块
│   ├── Collision_Uav.py	# UAV 碰撞模块
│   ├── Collision_Usv.py	# USV 碰撞模块
│   ├── Sensor_Uav.py		# UAV 探测模块
│   └── Sensor_Usv.py		# UAV 探测模块
│
├── Utils
│   ├── Manager.py			# 总控制模块
│   ├── Points.py			# Target 轨迹生成模块
│   ├── Refresher.py		# Target 刷新模块
│   ├── Scorer.py			# 计分模块
│   └── Visualizer.py		# 可视化模块
│
└── Vehicle		
    ├── BaseVehicle.py		# UAV/USV 父类
    ├── Target.py			# Target 控制模块 
    ├── Uav.py				# UAV 初始化模块
    └── Usv.py				# USV 初始化模块
```

对本项目结构有以下几点说明：

- 针对 `UAV` 和 `USV` 的控制文件原则上应写成类，全部放在 `Controller` 文件夹中，并在 `main.py` 中实例化控制类，每个 `step` 输出一个形如 `controls` 的控制矩阵供调用

```python
controls = [
        ["uav", "1", 0, 0], # 类型、编号、线速度、角速度
        ["uav", "2", 0, 0],
        ["usv", "1", 0, 0],
        ["usv", "2", 0, 0],
        ["usv", "3", 0, 0],
        ["usv", "4", 0, 0],
    ]
```

---

#### 三、实时已知参数

- 指定 `UAV` 探测列表 `manager.get_detected('uav', uid) -> List[Target]`
- 指定 `USV` 探测列表 `manager.get_detected('usv', uid) -> List[Target]`
- 指定 `USV` 捕获列表 `manager.get_captured('usv', uid) -> List[Target]`
- 总探测列表（去重）`manager.get_detected_all() -> List[Target.id]`
- 总捕获列表（去重）`manager.get_captured_all() -> List[Target.id]`
- 指定载具位置与朝向 `manager.get_state(typ, id) -> [pos, heading]`

- 实时仿真时间 `step * 0.05s`

