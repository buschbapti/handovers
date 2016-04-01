import numpy as np
from sympy.mpmath import *
from sympy import *
from human_moveit_config.human_model import HumanModel
from .reba_assess import RebaAssess
from scipy.optimize import minimize
import math
import transformations
from kinect_skeleton_publisher.joint_transformations import sympy_to_numpy, inverse
from copy import copy


class RebaOptimization(object):
    def __init__(self, cost_factors=None):
        if cost_factors is None:
            self.cost_factors = [1, 1, 1, 1]
        else:
            self.cost_factors = cost_factors
        # initialize human model
        self.model = HumanModel()
        # initialize REBA technique
        self.reba = RebaAssess()
        self.previous_joints = []
        # initialize task dependant informations
        self.safety_dist = [[0.5, 1.5], [-10., 10.], [-10, 10]]
        self.object_pose = []
        self.set_screwing_parameters(0.42, 0.2)

    def set_screwing_parameters(self, object_length, screwdriver_length):
        self.object_length = object_length
        self.screwdriver_length = screwdriver_length
        self.circle_rad = ((self.object_length+self.screwdriver_length)/2)
        self.circle_rad2 = self.circle_rad**2

        print self.circle_rad

    def get_active_joints_value(self, joints, side=0):
        joint_values = []
        for name in self.active_joints[side]:
            joint_values.append(joints[self.model.joint_names.index(name)])
        return joint_values

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

    def calculate_reba_cost(self, joints):
        # use the reba library to calculate the cost
        cost = self.reba.assess_posture(joints, self.model.get_joint_names())
        return cost

    def jacobian_reba_cost(self, joints):
        # use the reba library to calculate the cost
        jac = self.reba.deriv_assess_posture(joints, self.model.get_joint_names())
        return jac

    def fixed_joints_cost(self, joint_array, dict_values):
        cost = 0
        for key, value in dict_values.iteritems():
            cost += (joint_array[self.model.get_joint_names().index(key)]-value)**2
        return cost

    def jacobian_fixed_joints_cost(self, joint_array, dict_values):
        jac_fix = np.zeros(len(self.model.get_joint_names()))
        for key, value in dict_values.iteritems():
            index = self.model.get_joint_names().index(key)
            jac_fix[index] = 2*(joint_array[index]-value)
        return jac_fix

    # def return_value(self, joint_array, key):
    #     return joint_array[self.model.get_joint_names().index(key)]

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
        cost = np.dot(OH, Hx) + 1
        return cost

    def jacobian_sight_cost(self, obj_pos, head_frame, jac_human):
        jac_cost = np.zeros(len(jac_human[0]))
        diff_OH = np.array(head_frame[0]) - np.array(obj_pos)
        norm_OH = np.linalg.norm(diff_OH)
        OH = diff_OH
        # calculate head x vector
        q = np.array(head_frame[1])
        Hx = [1-2*q[1]*q[1]-2*q[2]*q[2],
              2*(q[0]*q[1]+q[2]*q[3]),
              2*(q[0]*q[2]-q[1]*q[3])]
        # calculate jacobian of the cost
        for i in range(len(jac_cost)):
            dOH = np.zeros(3)
            d_norm_OH = 0
            dHx = [-4*jac_human[5, i]*q[1]-4*jac_human[6, i]*q[2],
                   2*(q[0]*jac_human[5, i]+jac_human[4, i]*q[1] + q[2]*jac_human[3, i]+jac_human[6, i]*q[3]),
                   2*(q[0]*jac_human[6, i]+jac_human[4, i]*q[2] - q[1]*jac_human[3, i]-jac_human[5, i]*q[3])]
            for d in range(3):
                d_norm_OH += jac_human[d, i]*diff_OH[d]
                dOH[d] = jac_human[d, i]
                # calculate derivative of the dot product
                jac_cost[i] += OH[d]*dHx[d] + dOH[d]*Hx[d]
            d_norm_OH /= norm_OH
            # add the derivative of the norm
            jac_cost[i] += d_norm_OH

        # calculate the jacobian for all the joints
        joint_names = self.model.get_joint_names()
        group_joints = self.model.get_joint_names('head')
        jac_sight = np.zeros(len(joint_names))
        for i in range(len(group_joints)):
            index = joint_names.index(group_joints[i])
            jac_sight[index] = jac_cost[i]
        return jac_sight

    def calculate_velocity_cost(self, joints, dt=0.1):
        dq = (joints-self.previous_joints)/dt
        if dq.max() > 5:
            return 1000
        else:
            return 0

    def calculate_safety_cost(self, pose):
        cost = 0
        for i in range(3):
            p = pose[0][i]
            if p < self.safety_dist[i][0]:
                cost += (p - self.safety_dist[i][0])**2
            elif p > self.safety_dist[i][1]:
                cost += (p - self.safety_dist[i][1])**2
        return cost

    def calculate_screwing_cost(self, fk_hand):
        # check the height
        z_diff = (self.object_pose[2] - fk_hand[0][2])**2
        # check that the hand lie on a circle with object as center
        circle_diff = ((fk_hand[0][0]-self.object_pose[0])**2 +
                       (fk_hand[0][1]-self.object_pose[1])**2 -
                       self.circle_rad2)**2
        return circle_diff + z_diff

    def jacobian_screwing_cost(self, fk_hand, jac_human):
        jac_cost = np.zeros(len(jac_human[0]))
        z_diff = -2*(self.object_pose[2] - fk_hand[0][2])
        x_diff = fk_hand[0][0] - self.object_pose[0]
        y_diff = fk_hand[0][1] - self.object_pose[1]
        circle_diff = 4*((fk_hand[0][0]-self.object_pose[0])**2 +
                         (fk_hand[0][1]-self.object_pose[1])**2 -
                         self.circle_rad2)
        for i in range(len(jac_cost)):
            jac_z = jac_human[2, i]*z_diff
            jac_circle = (jac_human[0, i]*x_diff + jac_human[1, i]*y_diff)*circle_diff
            jac_cost[i] = jac_z + jac_circle
        return jac_cost

    def calculate_reaching_cost(self, fk_hand):
        return np.sum((self.object_pose-fk_hand[0])**2)

    def jacobian_reaching_cost(self, fk_hand, jac_human):
        jac_cost = np.zeros(len(jac_human[0]))
        pos_diff = fk_hand[0] - self.object_pose
        for i in range(len(jac_cost)):
            for j in range(3):
                jac_cost[i] += jac_human[j, i]*pos_diff[j]
            jac_cost[i] *= 2
        return jac_cost

    def calculate_task_cost(self, fk_hand):
        if self.task == 'screw':
            return self.calculate_screwing_cost(fk_hand)
        elif self.task == 'receive':
            return self.calculate_safety_cost(fk_hand)
        elif self.task == 'reach':
            return self.calculate_reaching_cost(fk_hand)
        else:
            return 0

    def jacobian_task_cost(self, fk_hand, jac_human, group_name):
        joint_names = self.model.get_joint_names()
        group_joints = self.model.get_joint_names(group_name)
        jac_task = np.zeros(len(joint_names))
        if self.task == 'screw':
            jac = self.jacobian_screwing_cost(fk_hand, jac_human)
        elif self.task == 'receive':
            jac = self.jacobian_safety_cost(fk_hand, jac_human)
        elif self.task == 'reach':
            jac = self.jacobian_reaching_cost(fk_hand, jac_human)
        # complete the jacobian for all the joints
        for i in range(len(group_joints)):
            index = joint_names.index(group_joints[i])
            jac_task[index] = jac[i]
        return jac_task

    def cost_function(self, q, side=0, use_velocity=False, fixed_joints={}):
        C_reba = 0
        C_task = 0
        C_sight = 0
        # C_fixed_frame = 0
        C_fixed_joints = 0
        C_velocity = 0
        # replace the value of fixed joints
        C_fixed_joints = self.fixed_joints_cost(q, fixed_joints)
        # check the necessity to perform the operations
        if (self.cost_factors[1] != 0 or
            self.cost_factors[2] != 0 or
           (self.cost_factors[3] != 0 and fixed_frames)):
            # get current state
            js = self.model.get_current_state()
            # set the new joint values
            js.position = q
            # calculate the forward kinematic
            fk_dict = self.model.forward_kinematic(js, group_name='upper_body')
            # extract hand pose
            if side == 0:
                hand_pose = fk_dict['right_hand_tip']
            else:
                hand_pose = fk_dict['left_hand_tip']
            # calculate the cost based on safety distance
            if self.cost_factors[1] != 0:
                C_task = self.calculate_task_cost(hand_pose)
            # calculate the cost of having the object in sight
            if self.cost_factors[2] != 0:
                # extract head pose
                head_pose = fk_dict['head_tip']
                C_sight = self.calculate_sight_cost(self.object_pose, head_pose)
            # if self.cost_factors[2] != 0 and fixed_frames:
            #     # calculate cost based on the fixed frames
            #     C_fixed_frame = self.calculate_fixed_frame_cost(chains, fixed_frames)
        # calculate REBA score
        if self.cost_factors[0] != 0:
            C_reba = self.calculate_reba_cost(q)
        # calculate the score based on the velocity
        if use_velocity:
            C_velocity = self.calculate_velocity_cost(q)
        # return the final score
        cost = (C_fixed_joints +
                C_velocity +
                self.cost_factors[0]*C_reba +
                self.cost_factors[1]*C_task +
                self.cost_factors[2]*C_sight)

        print cost
        return cost

    def jacobian_cost_function(self, q, side=0, use_velocity=False, fixed_joints={}):
        jac_sight = np.zeros(len(self.model.get_joint_names()))
        jac_task = np.zeros(len(self.model.get_joint_names()))
        jac_reba = np.zeros(len(self.model.get_joint_names()))
        # calculate the jacobian of the fix joints criterion
        jac_fix = self.jacobian_fixed_joints_cost(q, fixed_joints)
        # check the necessity to perform the operations
        if (self.cost_factors[1] != 0 or
            self.cost_factors[2] != 0 or
           (self.cost_factors[3] != 0 and fixed_frames)):
            # get current state
            js = self.model.get_current_state()
            # set the new joint values
            js.position = q
            # calculate the forward kinematic
            fk_dict = self.model.forward_kinematic(js, group_name='upper_body')
            # extract hand pose and human jacobian
            if side == 0:
                hand_pose = fk_dict['right_hand_tip']
                jac_hand = self.model.jacobian('right_arm', js, use_quaternion=True)
                group_name = 'right_arm'
            else:
                hand_pose = fk_dict['left_hand_tip']
                jac_hand = self.model.jacobian('left_arm', js, use_quaternion=True)
                group_name = 'left_arm'
            # calculate the cost specific to the task
            if self.cost_factors[1] != 0:
                jac_task = self.jacobian_task_cost(hand_pose, jac_hand, group_name)
            # calculate the cost of having the object in sight
            if self.cost_factors[2] != 0:
                # extract head pose
                head_pose = fk_dict['head_tip']
                jac_head = self.model.jacobian('head', js, use_quaternion=True)
                jac_sight = self.jacobian_sight_cost(self.object_pose, head_pose, jac_head)
        # calculate REBA jacobian
        if self.cost_factors[0] != 0:
            jac_reba = self.jacobian_reba_cost(q)
        jac_cost = (jac_fix +
                    self.cost_factors[0]*jac_reba +
                    self.cost_factors[1]*jac_task +
                    self.cost_factors[2]*jac_sight)
        return jac_cost

    def optimize_posture(self, joints, task, side=0, nb_points=1, fixed_joints={}):
        self.task = task
        # initialize the trajectory
        joint_traj = []
        # get the joints limits for the optimization
        joint_limits = self.model.get_joint_limits()
        # get the initial_joint
        init_joints = copy(joints)
        joint_traj.append(init_joints)
        self.previous_joints = init_joints
        # check if the velocity is constrained
        use_velocity = nb_points > 1
        for i in range(nb_points):
            # call optimization from scipy
            res = minimize(self.cost_function, init_joints,
                           jac=self.jacobian_cost_function,
                           args=(side, use_velocity, fixed_joints, ),
                           method='L-BFGS-B',
                           bounds=joint_limits)
            # options={'maxfun': 100})
            init_joints = res.x
            self.previous_joints = init_joints
            joint_traj.append(init_joints)

            print res
            print "point "+str(i)+" calculated"
        return joint_traj
