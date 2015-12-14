import numpy as np
from sympy.mpmath import *
from sympy import *
from .read_model import ReadModel
from .reba_assess import RebaAssess
from scipy.optimize import minimize
import time
import math
import transformations
from kinect_skeleton_publisher.joint_transformations import sympy_to_numpy

class RebaOptimization(object):
    def __init__(self, safety_dist, sum_optim=False, save_score=False, cost_factors=None):
        if cost_factors is None:
            self.cost_factors = [1, 1, 1]
        else:
            self.cost_factors = cost_factors
        self.safety_dist = safety_dist
        self.active_joints = [['spine_0', 'spine_1', 'spine_2',
                                'right_shoulder_0', 'right_shoulder_1', 'right_shoulder_2',
                                'right_elbow_0', 'right_elbow_1',
                                'right_wrist_0', 'right_wrist_1'],
                                ['spine_0', 'spine_1', 'spine_2',
                                'left_shoulder_0', 'left_shoulder_1', 'left_shoulder_2',
                                'left_elbow_0', 'left_elbow_1',
                                'left_wrist_0', 'left_wrist_1']]
        # initialize human model
        self.model = ReadModel()
        # initialize REBA technique
        self.reba = RebaAssess(save_score=save_score, sum_optim=sum_optim)

    def get_active_joints_value(self, joints, side=0):
        joint_values = []
        for name in self.active_joints[side]:
            joint_values.append(joints[self.model.joint_names.index(name)])
        return joint_values

    def calculate_safety_cost(self, T):
        cost = 0
        for i in range(3):
            p = Float(T[i,-1])
            if p < self.safety_dist[i][0]:
                cost += abs(p - self.safety_dist[i][0])
            elif p > self.safety_dist[i][1]:
                cost += abs(p - self.safety_dist[i][1])
        return cost

    def calculate_fixed_frame_cost(self, chain, fixed_frames, side):     
        def caclulate_distance_to_frame(pose, ref_pose, coeffs):
            # calculate distance in position
            d_position = np.linalg.norm(pose[0]-ref_pose[0])
            # calculate distance in quaternions
            d_rotation = math.acos(2*np.inner(pose[1],ref_pose[1])**2-1)
            # return the sum of both
            return coeffs[0]*d_position + coeffs[1]*d_rotation
        cost = 0
        if fixed_frames:
            for key in fixed_frames:
                ref_pose = fixed_frames[key]['pose']
                coeffs = fixed_frames[key]['coeffs']
                key_found = False
                # check if the fixed frame is an end-effector
                if key == self.model.end_effectors[side]:
                    # pose is the last transform of the chain
                    pose = chain[-1]
                    key_found = True
                elif key in self.active_joints[side]:
                    # get the frame from the chain
                    pose = chain[self.active_joints[side].index(key)]
                    key_found = True
                # if the key have been found 
                if key_found:
                    # convert the pose to tuple
                    pose = transformations.m4x4_to_list(sympy_to_numpy(pose))
                    # calculate the distance wrt to the fixed frame
                    cost += caclulate_distance_to_frame(pose, ref_pose, coeffs)
        return cost

    def calculate_reba_cost(self, joints):
        # convert joints to REBA norms
        reba_data = self.reba.from_joints_to_reba(joints, self.model.joint_names)
        # use the reba library to calculate the cost
        cost = self.reba.reba_optim(reba_data)
        return cost

    def cost_function(self, q, side=0, fixed_joints={}, fixed_frames={}):
        joints = q
        # replace the value of fixed joints
        for key, value in fixed_joints.iteritems():
            joints[self.model.joint_names.index(key)] = value
        # first get the active joints
        active = self.get_active_joints_value(joints, side)
        # calculate the forward kinematic
        chain = self.model.forward_kinematic(active, side)
        T = chain[-1]
        # calculate the cost based on safety distance
        C_safe = self.calculate_safety_cost(T)
        # calculate cost based on the fixed frames
        C_fixed_frame = self.calculate_fixed_frame_cost(chain, fixed_frames, side)
        # calculate REBA score
        C_reba = self.calculate_reba_cost(joints)
        # return the final score
        return self.cost_factors[0]*C_reba + self.cost_factors[1]*C_safe + self.cost_factors[2]*C_fixed_frame

    def optimize_posture(self, joints, side=0, var=0.1, fixed_joints={}, fixed_frames={}):
        joint_limits = self.model.joint_limits()['limits']
        # call optimization from scipy
        res = minimize(self.cost_function, joints, args=(side, fixed_joints, fixed_frames ), method='L-BFGS-B', bounds=joint_limits, options={'eps':var})
        # replace the value of fixed joints
        for key, value in fixed_joints.iteritems():
            res.x[self.model.joint_names.index(key)] = value
        return res
