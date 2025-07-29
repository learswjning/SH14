# import random
# import math
# import os
# import sys
# sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
# from Vehicle.Target import Target

# class TargetRefresher:
#     def __init__(self, min_interval=50, max_interval=150, area_size=9260, start_id=3):
#         self.min_interval = min_interval
#         self.max_interval = max_interval
#         self.area_size = area_size
#         self.next_refresh_step = self._generate_next_refresh(0)
#         self.current_id = start_id

#     def _generate_next_refresh(self, current_step):
#         return current_step + random.randint(self.min_interval, self.max_interval)

#     def refresh(self, current_step):
#         if current_step >= self.next_refresh_step:
#             num_targets = random.choice([1, 2])
#             new_targets = []
#             for _ in range(num_targets):
#                 tid = str(self.current_id)
#                 new_targets.append(tid)
#                 self.current_id += 1
#             self.next_refresh_step = self._generate_next_refresh(current_step)
#             return new_targets
#         else:
#             return []
import os
import sys
import random
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

class TargetRefresher:
    def __init__(self, refresh_interval=12000, area_size=9260, start_id=3):
        self.refresh_interval = refresh_interval
        self.area_size = area_size
        self.next_refresh_step = 12000
        self.current_id = start_id

    def refresh(self, current_step):
        if current_step >= self.next_refresh_step:
            num_targets = random.choice([0, 1, 2])
            new_targets = []
            for _ in range(num_targets):
                tid = str(self.current_id)
                new_targets.append(tid)
                self.current_id += 1
            self.next_refresh_step = current_step + self.refresh_interval
            return new_targets
        else:
            return []