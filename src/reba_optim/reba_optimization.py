import numpy as np
from sympy.mpmath import *
from sympy import *
from human_moveit_config.human_model import HumanModel
from .reba_assess import RebaAssess
from scipy.optimize import minimize
import math
import transformations
from kinect_skeleton_publisher.joint_transformations import sympy_to_numpy, inverse


class RebaOptimization(object):
    def __init__(self, safety_dist, cost_factors=None):
        if cost_factors is None:
            self.cost_factors = [1, 1, 1, 1]
        else:
            self.cost_factors = cost_factors
        self.safety_dist = safety_dist
        # initialize human model
        self.model = HumanModel()
        # initialize REBA technique
        self.reba = RebaAssess()

    def get_active_joints_value(self, joints, side=0):
        joint_values = []
        for name in self.active_joints[side]:
            joint_values.append(joints[self.model.joint_names.index(name)])
        return joint_values

    def calculate_safety_cost(self, pose):
        cost = 0
        for i in range(3):
            p = pose[0][i]
            if p < self.safety_dist[i][0]:
                cost += abs(p - self.safety_dist[i][0])
            elif p > self.safety_dist[i][1]:
                cost += abs(p - self.safety_dist[i][1])
        return cost

    def calculate_fixed_frame_cost(self, chains, fixed_frames):
        def caclulate_distance_to_frame(pose, ref_pose, coeffs):
            # calculate distance in position
            d_position = np.linalg.norm(pose[0]-ref_pose[0])
            # calculate distance in quaternions
            d_rotation = math.acos(2*np.inner(pose[1], ref_pose[1])**2-1)
            # return the sum of both
            return coeffs[0]*d_position + coeffs[1]*d_rotation

        def get_frame_pose(frame_name):
            key_found = False
            side = 0
            while not key_found and side < len(self.model.end_effectors):
                # check if the fixed frame is an end-effector
                if frame_name == self.model.end_effectors[side]:
                    # pose is the last transform of the chain
                    pose = chains[side][-1]
                    key_found = True
                elif frame_name in self.active_joints[side]:
                    # get the frame from the chain
                    pose = chains[side][self.active_joints[side].index(frame_name)]
                    key_found = True
            # convert the pose to tuple
            pose = transformations.m4x4_to_list(sympy_to_numpy(pose))
            return pose

        def get_pose_in_reference_frame(frame_name, frame_reference=None):
            # get frame in hip reference
            fixed = get_frame_pose(frame_name)
            # if reference frame is specified get it
            if frame_reference is not None:
                ref = get_frame_pose(frame_reference)
                # calculate the transformation between them
                link = inverse(ref)*fixed
                return link
            return fixed
        cost = 0
        if fixed_frames:
            for key in fixed_frames:
                frame_dict = fixed_frames[key]
                coeffs = frame_dict['coeffs']
                des_pose = frame_dict['desired_pose']
                ref_frame = frame_dict['reference_frame']
                # get the pose of the fixed frame
                pose = self.get_pose_in_reference_frame(key, ref_frame)
                # calculate the distance wrt to the fixed frame
                cost += caclulate_distance_to_frame(pose, des_pose, coeffs)
        return cost

    def calculate_reba_cost(self, joints, save_score=False):
        # use the reba library to calculate the cost
        cost = self.reba.assess_posture(joints, self.model.get_joint_names(), save_score)
        return cost

    def fixed_joints_cost(self, joint_array, dict_values):
        cost = 0
        for key, value in dict_values.iteritems():
            cost += abs(joint_array[self.model.get_joint_names().index(key)]-value)
        return cost

    def return_value(self, joint_array, key):
        return joint_array[self.model.get_joint_names().index(key)]

    # def assign_leg_values(self, joint_array):
    #     def assign_per_leg(side):
    #         knee = self.return_value(joint_array, side+'_knee')
    #         hip, ankle = self.model.calculate_leg_joints(knee)
    #         self.assign_value(joint_array, {side+'_hip_1': hip, side+'_ankle_1': hip})
    #     assign_per_leg('right')
    #     assign_per_leg('left')

    def calculate_sight_cost(self, obj_pos, head_frame):
        # calculate head to object vector
        OH = np.array(head_frame[0]) - np.array(obj_pos)
        OH /= np.linalg.norm(OH)
        # calculate head x vector
        q = np.array(head_frame[1])
        Hx = [1-2*q[1]*q[1]-2*q[2]*q[2],
              2*(q[0]*q[1]+q[2]*q[3]),
              2*(q[0]*q[2]-q[1]*q[3])]
        # check colinearity between the two vectors
        cost = 0
        for i in range(3):
            for j in range(i, 3):
                cost += (Hx[i]*OH[j]-Hx[j]*OH[i])**2
        return cost

    def cost_function(self, q, side=0, fixed_joints={}, fixed_frames={}):
        C_reba = 0
        C_safe = 0
        C_sight = 0
        C_fixed_frame = 0
        C_fixed_joints = 0
        # replace the value of fixed joints
        C_fixed_joints = self.fixed_joints_cost(q, fixed_joints)
        # check the necessity to perform the operations
        if (self.cost_factors[1] != 0 or
            self.cost_factors[2] != 0 or
           (self.cost_factors[3] != 0 and fixed_frames)):
            # get current state
            js = self.model.get_current_state().joint_state
            # set the new joint values
            js.position = q
            # calculate the forward kinematic
            fk_dict = self.model.full_forward_kinematic(js)
            # extract hand pose
            if side == 0:
                hand_pose = fk_dict['right_hand_tip']
            else:
                hand_pose = fk_dict['left_hand_tip']
            # calculate the cost based on safety distance
            if self.cost_factors[1] != 0:
                C_safe = self.calculate_safety_cost(hand_pose)
            # calculate the cost of having the object in sight
            if self.cost_factors[2] != 0:
                # extract head pose
                head_pose = fk_dict['head_tip']
                C_sight = self.calculate_sight_cost(hand_pose[0], head_pose)
            if self.cost_factors[2] != 0 and fixed_frames:
                # calculate cost based on the fixed frames
                C_fixed_frame = self.calculate_fixed_frame_cost(chains, fixed_frames)
        # calculate REBA score
        if self.cost_factors[0] != 0:
            C_reba = self.calculate_reba_cost(q)
        # return the final score
        cost = (C_fixed_joints +
                self.cost_factors[0]*C_reba +
                self.cost_factors[1]*C_safe +
                self.cost_factors[2]*C_sight +
                self.cost_factors[3]*C_fixed_frame)
        return cost

    def optimize_posture(self, joints, side=0, var=0.1, fixed_joints={}, fixed_frames={}):
        # by default hip and ankles angles are fixed
        fixed_joints['right_hip_0'] = 0.
        fixed_joints['right_hip_1'] = 0.
        fixed_joints['right_hip_2'] = 0.
        fixed_joints['right_knee'] = 0.
        fixed_joints['right_ankle_0'] = 0.
        fixed_joints['right_ankle_1'] = 0.

        fixed_joints['left_hip_0'] = 0.
        fixed_joints['left_hip_1'] = 0.
        fixed_joints['left_hip_2'] = 0.
        fixed_joints['left_knee'] = 0.
        fixed_joints['left_ankle_0'] = 0.
        fixed_joints['left_ankle_1'] = 0.
        # get the joints limits for the optimization
        joint_limits = self.model.get_joint_limits()
        # call optimization from scipy
        res = minimize(self.cost_function, joints,
                       args=(side, fixed_joints, fixed_frames, ),
                       method='L-BFGS-B', bounds=joint_limits,
                       options={'eps': var})
        return res
