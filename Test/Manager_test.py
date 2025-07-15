import os
import sys
import time
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from Utils.Manager import Manager  # 假设Manager放在Utils文件夹

def main():
    print("开始测试 Manager 模块")
    manager = Manager()

    uavs = [
        ["1", [0, 4630], 0],
        ["2", [2000, 4930], 0]
    ]
    usvs = [
        ["1", [0, 4635], 0],
        ["2", [2000, 4935], 0]
    ]
    targets = [
        ["1", [4000, 4635], 0]
    ]

    manager.init_objects(uavs, usvs, targets)

    for step in range(100000):
        controls = [
            ["uav", "1", 0, 0],
            ["uav", "2", 0, 0],
            ["usv", "1", 100, 0],
            ["usv", "2", 0, 0],
            ["target", "1", 0, 0],
        ]

        manager.update(controls)

        # 第10步时，动态追加目标
        if step == 500:
            new_targets = [
                ["2", [4500, 4700], 0],
                ["3", [4600, 4750], 0]
            ]
            print(f"Step {step}: 动态追加目标 {new_targets}")
            manager.add_targets(new_targets)

        # 每5步打印 UAV 和 USV 状态
        if step % 5 == 0:
            uav_states = manager.get_uav_states()
            print(f"Step {step} UAV states:")
            for sid, pos, heading in uav_states:
                print(f"  UAV {sid}: pos={pos}, heading={heading}")

            usv_states = manager.get_usv_states()
            print(f"Step {step} USV states:")
            for sid, pos, heading in usv_states:
                print(f"  USV {sid}: pos={pos}, heading={heading}")

        #time.sleep(0.05)

    print("测试结束。")

if __name__ == "__main__":
    main()
