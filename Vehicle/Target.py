import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Vehicle.BaseVehicle import BaseVehicle

class Target(BaseVehicle):
    def __init__(self, init_position, init_theta, id, init_t, max_turn_radius=20, dt=0.05):
        BaseVehicle.__init__(self, init_position=init_position, init_theta=init_theta, max_turn_radius=max_turn_radius, dt=dt)
        self.id = id
        self.type = 'target'
        self.T_entry = init_t
        self.T_detect = None
        self.T_capture = None
        self.T_detect_flag = False
    
    def update(self, v, omega):
        BaseVehicle.update(self, v=v, omega=omega)
        
    def score_update(self, t, detect, capture):
        if detect:
            self.T_detect_flag += 1
        else:
            self.T_detect_flag = 0
        if self.T_detect_flag == 10/self.dt:
            self.T_detect = t
        if capture:
            self.T_capture = t
        
    def score(self):
        time1 = None
        time2 = None
        if self.T_detect is not None:
            time1 = self.T_detect - self.T_entry
            time1 = time1 * self.dt
        if self.T_capture is not None:
            time2 = self.T_capture - self.T_entry
            time2 = time2 * self.dt
        return time1, time2
        
        