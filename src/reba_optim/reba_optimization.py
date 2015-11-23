import numpy as np
from sympy.mpmath import *
from sympy import *
from .read_model import ReadModel
from .reba_assess import RebaAssess
from scipy.optimize import minimize
import time

class RebaOptimization:
    def __init__(self, orientation, safety_dist, sum_optim=False, save_score=False, cost_factors=None):
        if cost_factors is None:
            self.cost_factors = [1, 1, 1]
        else:
            self.cost_factors = cost_factors
        self.orientation = orientation
        self.safety_dist = safety_dist
        self.active_joints = ['spine_0', 'spine_1', 'spine_2',
                                'right_shoulder_0', 'right_shoulder_1', 'right_shoulder_2',
                                'right_elbow_0', 'right_elbow_1',
                                'right_wrist_0', 'right_wrist_1']
        # initialize human model
        self.model = ReadModel()
        # initialize REBA technique
        self.reba = RebaAssess(save_score=save_score, sum_optim=sum_optim)

    def get_active_joints_value(self, joints):
        joint_values = []
        for name in self.active_joints:
            joint_values.append(joints[self.model.joint_names.index(name)])
        return joint_values

    def safety_cost(self, T):
        cost = 0
        for i in range(3):
            p = Float(T[i,-1])
            if p < self.safety_dist[i][0]:
                cost += abs(p - self.safety_dist[i][0])
            elif p > self.safety_dist[i][1]:
                cost += abs(p - self.safety_dist[i][1])
        return cost

    def cost_function(self, q):
        # first get the active joints
        active = self.get_active_joints_value(q)
        # calculate the forward kinematic
        T = self.model.forward_kinematic(active)
        # calculate the cost based on safety distance
        C_safe = self.safety_cost(T)
        # calculate cost based on desired orientation
        C_rot = (T[:-1,:-1] - self.orientation).norm()
        # convert joints to REBA norms
        reba_data = self.reba.from_joints_to_reba(q, self.model.joint_names)
        # calculate REBA score
        C_reba = self.reba.reba_optim(reba_data)
        # return the final score
        return self.cost_factors[0]*C_reba + self.cost_factors[1]*C_safe + self.cost_factors[2]*C_rot

    def optimize_posture(self, joints, var=0.1):
        joint_limits = self.model.joint_limits()['limits']
        # call optimization from scipy
        res = minimize(self.cost_function, joints, method='L-BFGS-B', bounds=joint_limits, options={'eps':var})
        return res
