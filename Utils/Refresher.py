import random
import math

class TargetRefresher:
    def __init__(self, min_interval=50, max_interval=150, area_size=9260, start_id=3):
        self.min_interval = min_interval
        self.max_interval = max_interval
        self.area_size = area_size
        self.next_refresh_step = self._generate_next_refresh(0)
        self.current_id = start_id

    def _generate_next_refresh(self, current_step):
        return current_step + random.randint(self.min_interval, self.max_interval)

    def _generate_point_on_boundary(self):
        edge = random.choice(['AB', 'BC', 'CD', 'DA'])
        if edge == 'AB':
            pos = [random.uniform(0, self.area_size), 0]
        elif edge == 'BC':
            pos = [self.area_size, random.uniform(0, self.area_size)]
        elif edge == 'CD':
            pos = [random.uniform(0, self.area_size), self.area_size]
        else:  # DA
            pos = [0, random.uniform(0, self.area_size)]
        return pos, edge

    def _generate_theta_by_edge(self, edge):
        if edge == 'AB':
            deg = random.uniform(45, 135)
        elif edge == 'BC':
            deg = random.uniform(135, 225)
        elif edge == 'CD':
            deg = random.uniform(225, 315)
        elif edge == 'DA':
            deg = random.uniform(-45, 45)
        else:
            deg = 0
        return math.radians(deg)

    def init(self):
        t1_pos, e1 = self._generate_point_on_boundary()
        t2_pos, e2 = self._generate_point_on_boundary()
        return [
            ['1', t1_pos, self._generate_theta_by_edge(e1)],
            ['2', t2_pos, self._generate_theta_by_edge(e2)],
        ]

    def refresh(self, current_step):
        if current_step >= self.next_refresh_step:
            num_targets = random.choice([1, 2])
            new_targets = []
            for _ in range(num_targets):
                tid = str(self.current_id)
                pos, edge = self._generate_point_on_boundary()
                theta = self._generate_theta_by_edge(edge)
                new_targets.append([tid, pos, theta])
                self.current_id += 1
            self.next_refresh_step = self._generate_next_refresh(current_step)
            return new_targets
        else:
            return []
