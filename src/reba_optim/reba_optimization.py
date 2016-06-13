import numpy as np
from human_moveit_config.human_model import HumanModel
from .reba_assess import RebaAssess
from scipy.optimize import minimize
import math
from copy import deepcopy


class RebaOptimization(object):
    def __init__(self, params={}):
        if 'cost_factors' in params:
            self.cost_factors = params['cost_factors']
        else:
            self.cost_factors = [1, 1, 1, 1]
        if 'safety_dist' in params:
            self.safety_dist = params['safety_dist']
        else:
            self.safety_dist = [[0.35, 0.45], [-0.1, 0.1], [0.15, 10]]
        if 'object_pose' in params:
            self.obj_pose = params['object_pose']
        else:
            self.object_pose = []
        if 'screwing_params' in params:
            p = params['screwing_params']
            self.set_screwing_parameters(p[0], p[1])
        else:
            self.set_screwing_parameters(0.42, 0.2)
        if 'fixed_frame_coeffs' in params:
            self.fixed_frame_coeffs = params['fixed_frame_coeffs']
        else:
            self.fixed_frame_coeffs = [1, 2]
        # initialize human model
        self.model = HumanModel()
        # initialize REBA technique
        self.reba = RebaAssess()
        self.previous_joints = []

    def set_screwing_parameters(self, object_length, screwdriver_length):
        self.object_length = object_length
        self.screwdriver_length = screwdriver_length
        self.circle_rad = ((self.object_length + self.screwdriver_length) / 2)
        self.circle_rad2 = self.circle_rad**2

    def calculate_fixed_frame_cost(self, fk_dict, fixed_frames):
        def distance_to_frame(pose, ref_pose):
            d_position = 0
            d_rotation = 0
            # calculate distance in position
            if self.fixed_frame_coeffs[0] != 0:
                d_position = np.linalg.norm(np.array(pose[0]) - np.array(ref_pose[0]))
            # calculate distance in quaternions
            if self.fixed_frame_coeffs[1] != 0:
                d_rotation = math.acos(2 * np.inner(pose[1], ref_pose[1])**2 - 1)
            # return the sum of both
            return self.fixed_frame_coeffs[0] * d_position + self.fixed_frame_coeffs[1] * d_rotation

        fixed_cost = 0
        # loop through all the fixed frames
        for key, value in fixed_frames.iteritems():
            fixed_cost += distance_to_frame(fk_dict[key], value)
        return fixed_cost

    def calculate_reba_cost(self, joints):
        # use the reba library to calculate the cost
        cost = self.reba.assess_posture(joints, self.joint_names)
        return cost

    # def jacobian_reba_cost(self, joints):
    #     # use the reba library to calculate the cost
    #     jac = self.reba.deriv_assess_posture(joints, self.model.get_joint_names())
    #     return jac

    def fixed_joints_cost(self, joint_array, dict_values):
        cost = 0
        for key, value in dict_values.iteritems():
            if key in self.joint_names:
                cost += (joint_array[self.joint_names.index(key)] - value)**2
        return cost

    # def jacobian_fixed_joints_cost(self, joint_array, dict_values):
    #     jac_fix = np.zeros(len(self.model.get_joint_names()))
    #     for key, value in dict_values.iteritems():
    #         index = self.model.get_joint_names().index(key)
    #         jac_fix[index] = 2 * (joint_array[index] - value)
    #     return jac_fix

    def calculate_sight_cost(self, obj_pose, head_frame, angle_tresh=1.0472):
        # calculate head to object vector
        OH = np.array(head_frame[0]) - np.array(obj_pose[0])
        OH /= np.linalg.norm(OH)
        # calculate head x vector
        q = np.array(head_frame[1])
        Hx = [1 - 2 * q[1] * q[1] - 2 * q[2] * q[2],
              2 * (q[0] * q[1] + q[2] * q[3]),
              2 * (q[0] * q[2] - q[1] * q[3])]
        # check colinearity between the two vectors
        cost = 2 * (np.dot(OH, Hx) + 1)
        # check additional constraints based on object orientation
        qo = np.array(obj_pose[1])
        z_obj = [2 * (qo[0] * qo[2] + qo[1] * qo[3]),
                 2 * (qo[1] * qo[2] - qo[0] * qo[3]),
                 1 - 2 * qo[0] * qo[0] - 2 * qo[1] * qo[1]]
        theta = math.acos(np.dot(-np.array(Hx), z_obj))
        if abs(theta) > angle_tresh:
            cost += 1 * abs(theta - angle_tresh)
        return cost

    # def jacobian_sight_cost(self, obj_pos, head_frame, jac_human):
    #     jac_cost = np.zeros(len(jac_human[0]))
    #     diff_OH = np.array(head_frame[0]) - np.array(obj_pos)
    #     norm_OH = np.linalg.norm(diff_OH)
    #     OH = diff_OH
    #     # calculate head x vector
    #     q = np.array(head_frame[1])
    #     Hx = [1 - 2 * q[1] * q[1] - 2 * q[2] * q[2],
    #           2 * (q[0] * q[1] + q[2] * q[3]),
    #           2 * (q[0] * q[2] - q[1] * q[3])]
    #     # calculate jacobian of the cost
    #     for i in range(len(jac_cost)):
    #         dOH = np.zeros(3)
    #         d_norm_OH = 0
    #         dHx = [- 4 * jac_human[5, i] * q[1] - 4 * jac_human[6, i] * q[2],
    #                2 * (q[0] * jac_human[5, i] + jac_human[4, i] * q[1] + q[2] *
    #                jac_human[3, i] + jac_human[6, i] * q[3]),
    #                2 * (q[0] * jac_human[6, i] + jac_human[4, i] * q[2] - q[1] *
    #                jac_human[3, i] - jac_human[5, i] * q[3])]
    #         for d in range(3):
    #             d_norm_OH += jac_human[d, i] * diff_OH[d]
    #             dOH[d] = jac_human[d, i]
    #             # calculate derivative of the dot product
    #             jac_cost[i] += OH[d] * dHx[d] + dOH[d] * Hx[d]
    #         d_norm_OH /= norm_OH
    #         # add the derivative of the norm
    #         jac_cost[i] += d_norm_OH

    #     # calculate the jacobian for all the joints
    #     joint_names = self.model.get_joint_names()
    #     group_joints = self.model.get_joint_names('head')
    #     jac_sight = np.zeros(len(joint_names))
    #     for i in range(len(group_joints)):
    #         index = joint_names.index(group_joints[i])
    #         jac_sight[index] = jac_cost[i]
    #     return jac_sight

    def calculate_velocity_cost(self, joints, dt=0.1):
        dq = (joints - self.previous_joints) / dt
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
        circle_diff = ((fk_hand[0][0] - self.object_pose[0])**2 +
                       (fk_hand[0][1] - self.object_pose[1])**2 -
                       self.circle_rad2)**2
        return circle_diff + z_diff

    # def jacobian_screwing_cost(self, fk_hand, jac_human):
    #     jac_cost = np.zeros(len(jac_human[0]))
    #     z_diff = -2 * (self.object_pose[2] - fk_hand[0][2])
    #     x_diff = fk_hand[0][0] - self.object_pose[0]
    #     y_diff = fk_hand[0][1] - self.object_pose[1]
    #     circle_diff = 4 * ((fk_hand[0][0] - self.object_pose[0])**2 +
    #                        (fk_hand[0][1] - self.object_pose[1])**2 -
    #                        self.circle_rad2)
    #     for i in range(len(jac_cost)):
    #         jac_z = jac_human[2, i] * z_diff
    #         jac_circle = (jac_human[0, i] * x_diff + jac_human[1, i] * y_diff) * circle_diff
    #         jac_cost[i] = jac_z + jac_circle
    #     return jac_cost

    def calculate_reaching_cost(self, fk_hand):
        return np.sum((self.object_pose - fk_hand[0])**2)

    # def jacobian_reaching_cost(self, fk_hand, jac_human):
    #     jac_cost = np.zeros(len(jac_human[0]))
    #     pos_diff = fk_hand[0] - self.object_pose
    #     for i in range(len(jac_cost)):
    #         for j in range(3):
    #             jac_cost[i] += jac_human[j, i] * pos_diff[j]
    #         jac_cost[i] *= 2
    #     return jac_cost

    def calculate_task_cost(self, fk_hand):
        if self.task == 'screw':
            return self.calculate_screwing_cost(fk_hand)
        elif self.task == 'receive':
            return self.calculate_safety_cost(fk_hand)
        elif self.task == 'reach':
            return self.calculate_reaching_cost(fk_hand)
        else:
            return 0

    # def jacobian_task_cost(self, fk_hand, jac_human, group_name):
    #     joint_names = self.model.get_joint_names()
    #     group_joints = self.model.get_joint_names(group_name)
    #     jac_task = np.zeros(len(joint_names))
    #     if self.task == 'screw':
    #         jac = self.jacobian_screwing_cost(fk_hand, jac_human)
    #     elif self.task == 'receive':
    #         jac = self.jacobian_safety_cost(fk_hand, jac_human)
    #     elif self.task == 'reach':
    #         jac = self.jacobian_reaching_cost(fk_hand, jac_human)
    #     # complete the jacobian for all the joints
    #     for i in range(len(group_joints)):
    #         index = joint_names.index(group_joints[i])
    #         jac_task[index] = jac[i]
    #     return jac_task

    def cost_function(self, q, side='right', use_velocity=False, fixed_joints={}, fixed_frames={}, cost_details={}):
        C_reba = 0
        C_task = 0
        C_sight = 0
        C_fixed_frame = 0
        C_fixed_joints = 0
        C_velocity = 0
        # replace the value of fixed joints
        C_fixed_joints = self.fixed_joints_cost(q, fixed_joints)
        # get current state
        js = self.model.get_current_state()
        # set the new joint values
        js.position = q
        js.name = self.joint_names
        # calculate the forward kinematic
        fk_dict = self.model.forward_kinematic(js, links='all')
        # calculate cost based on the fixed frames
        C_fixed_frame = self.calculate_fixed_frame_cost(fk_dict, fixed_frames)
        # extract hand pose
        hand_pose = fk_dict[side + '_hand']
        # calculate the cost based on safety distance
        C_task = self.calculate_task_cost(hand_pose)
        # calculate the cost of having the object in sight
        head_pose = fk_dict['head']
        # if the task is received the model look at its hand
        if self.task == 'receive':
            C_sight = self.calculate_sight_cost(hand_pose, head_pose)
        # otherwise it looks at the object
        else:
            C_sight = self.calculate_sight_cost(self.object_pose, head_pose)
        # calculate REBA score
        C_reba = self.calculate_reba_cost(q)
        # calculate the score based on the velocity
        if use_velocity:
            C_velocity = self.calculate_velocity_cost(q)
        # return the final score
        cost = (C_fixed_joints +
                C_fixed_frame +
                C_velocity +
                self.cost_factors[0] * C_reba +
                self.cost_factors[1] * C_task +
                self.cost_factors[2] * C_sight)
        # append the detail of each cost
        cost_details['reba'] = C_reba
        cost_details['task'] = C_task
        cost_details['sight'] = C_sight

        print cost
        return cost

    # def jacobian_cost_function(self, q, side='right', use_velocity=False, fixed_joints={}, fixed_frames={}):
    #     jac_sight = np.zeros(len(self.model.get_joint_names()))
    #     jac_task = np.zeros(len(self.model.get_joint_names()))
    #     jac_reba = np.zeros(len(self.model.get_joint_names()))
    #     # calculate the jacobian of the fix joints criterion
    #     jac_fix = self.jacobian_fixed_joints_cost(q, fixed_joints)
    #     # check the necessity to perform the operations
    #     if (self.cost_factors[1] != 0 or self.cost_factors[2] != 0):
    #         # get current state
    #         js = self.model.get_current_state()
    #         # set the new joint values
    #         js.position = q
    #         # calculate the forward kinematic
    #         fk_dict = self.model.forward_kinematic(js, links='all')
    #         # extract hand pose and human jacobian
    #         hand_pose = fk_dict[side + '_hand']
    #         jac_hand = self.model.jacobian(side + '_arm', js, use_quaternion=True)
    #         group_name = side + '_arm'
    #         # calculate the cost specific to the task
    #         if self.cost_factors[1] != 0:
    #             jac_task = self.jacobian_task_cost(hand_pose, jac_hand, group_name)
    #         # calculate the cost of having the object in sight
    #         if self.cost_factors[2] != 0:
    #             # extract head pose
    #             head_pose = fk_dict['head']
    #             jac_head = self.model.jacobian('head', js, use_quaternion=True)
    #             jac_sight = self.jacobian_sight_cost(self.object_pose, head_pose, jac_head)
    #     # calculate REBA jacobian
    #     if self.cost_factors[0] != 0:
    #         jac_reba = self.jacobian_reba_cost(q)
    #     jac_cost = (jac_fix +
    #                 self.cost_factors[0] * jac_reba +
    #                 self.cost_factors[1] * jac_task +
    #                 self.cost_factors[2] * jac_sight)
    #     return jac_cost

    def optimize_posture(self, init_state, task, side='right', group_names='whole_body',
                         nb_points=1, fixed_joints={}, fixed_frames={}, maxiter=100):
        self.task = task
        # initialize the trajectory
        joint_traj = []
        # get the number of joints according to the required group
        if type(group_names) is not list:
            group_names = [group_names]
        joint_names_set = set()

        for g in group_names:
            joint_names_set.update(self.model.get_joint_names(g))
        self.joint_names = list(joint_names_set)
        # get only joints to optimize
        init_joints = []
        for name in self.joint_names:
            init_joints.append(init_state.position[init_state.name.index(name)])
        # get the initial_joint
        joint_traj.append(init_joints)
        self.previous_joints = init_joints
        # get the joints limits for the optimization
        joint_limits = self.model.get_joint_limits(self.joint_names)
        # check if the velocity is constrained
        use_velocity = nb_points > 1
        cost_details = []
        for i in range(nb_points):
            # call optimization from scipy
            costs = {}
            res = minimize(self.cost_function, init_joints,
                           args=(side, use_velocity, fixed_joints, fixed_frames, costs),
                           method='L-BFGS-B',
                           bounds=joint_limits,
                           options={'maxfun': maxiter})
            # options={'maxfun': 100})
            init_joints = res.x
            self.previous_joints = init_joints
            # convert it to a joint state
            js = self.model.get_current_state()
            js.position = list(js.position)
            js.name = list(js.name)
            # replace joint values with optimized ones
            for i in range(len(self.joint_names)):
                name = self.joint_names[i]
                js.position[js.name.index(name)] = init_joints[i]
            # replace fixed values
            for key, value in fixed_joints.iteritems():
                js.position[js.name.index(key)] = value

            print res
            print "point " + str(i) + " calculated"

            joint_traj.append(js)
            cost_details.append(deepcopy(costs))
        return joint_traj, cost_details
