import numpy as np
from human_moveit_config.human_model import HumanModel
from .reba_assess import RebaAssess
from scipy.optimize import minimize
from scipy.optimize import differential_evolution
# from scipy.optimize import basinhopping
import math
from numpy import linspace
from numpy import sqrt
from transformations import multiply_transform
from numpy.linalg import norm
from numpy.random import uniform


class RebaOptimization(object):
    def __init__(self, params={}):
        if 'optimizer' in params:
            self.optimizer = params['optimizer']
        else:
            self.optimizer = 'gradient'
        if 'cost_factors' in params:
            self.cost_factors = params['cost_factors']
        else:
            self.cost_factors = [1, 1, 1, 1]
        if 'safety_dist' in params:
            self.safety_dist = params['safety_dist']
        else:
            self.safety_dist = [[0.35, 0.45], [-0.1, 0.1], [0.15, 10]]
        if 'object_pose' in params:
            self.object_pose = params['object_pose']
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
        if 'assessment_method' in params:
            self.assessment_method = params['assessment_method']
        else:
            self.assessment_method = 'polynomial'
        # initialize human model
        self.model = HumanModel()
        # initialize REBA technique
        self.reba = RebaAssess()
        self.previous_joints = []
        self.nb_params = 0
        self.task = ''

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

    # def calculate_reba_cost(self, state):
    #     # use the reba library to calculate the cost
    #     cost = self.reba.assess_from_neural_model(state)
    #     return cost

    def calculate_reba_cost(self, joint_state):
        cost = self.reba.assess_posture(joint_state, self.assessment_method)
        return cost

    def fixed_joints_cost(self, joint_array, dict_values):
        cost = 0
        for key, value in dict_values.iteritems():
            if key in self.joint_names:
                cost += (joint_array[self.joint_names.index(key)] - value)**2
        return cost

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
        value = min(1, max(np.dot(-np.array(Hx), z_obj), -1))
        theta = math.acos(value)
        if abs(theta) > angle_tresh:
            cost += 1 * abs(theta - angle_tresh)
        return cost

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

    def calculate_reaching_cost(self, fk_hand):
        return np.sum((self.object_pose - fk_hand[0])**2)

    def calculate_welding_cost(self, fk_hand, current_point):
        # transform the current point
        point = multiply_transform(self.object_pose, self.welding_points[current_point])
        d_position = np.linalg.norm(np.array(point[0]) - np.array(fk_hand[0]))
        cost = d_position
        return cost

    def calculate_task_cost(self, fk_hand, current_point=0):
        if self.task == 'screw':
            return self.calculate_screwing_cost(fk_hand)
        elif self.task == 'receive':
            return self.calculate_safety_cost(fk_hand)
        elif self.task == 'reach':
            return self.calculate_reaching_cost(fk_hand)
        elif self.task == 'welding':
            return self.calculate_welding_cost(fk_hand, current_point)
        else:
            return 0

    def use_parameters(self, opt_params, coeff=5):
        if self.task == 'welding':
            # transform the object pose with the parameters
            if self.nb_params == 3:
                self.object_pose = [opt_params.tolist(), self.object_pose[1]]
            elif self.nb_params == 4:
                self.object_pose = [self.object_pose[0], opt_params.tolist()]
                return coeff * abs(norm(opt_params) - 1)
            elif self.nb_params == 7:
                self.object_pose = [opt_params[:3].tolist(), opt_params[3:].tolist()]
                return coeff * abs(norm(opt_params[3:]) - 1)
        return 0

    def cost_function(self, q, side='right', fixed_joints={}, fixed_frames={},
                      cost_details={}, nb_points=1):
        C_reba = 0
        C_task = 0
        C_sight = 0
        C_fixed_frame = 0
        C_fixed_joints = 0
        C_parameters = 0
        cost = []
        # extract the parameters
        opt_params = q[len(q) - self.nb_params:]
        nb_joints = (len(q) - self.nb_params) / nb_points
        # use the parameters of the optimzation
        C_parameters = self.use_parameters(opt_params)
        # loop through all the points of the trajectory
        for i in range(nb_points):
            joint_values = q[i * nb_joints:(i + 1) * nb_joints]
            # replace the value of fixed joints
            C_fixed_joints = self.fixed_joints_cost(joint_values, fixed_joints)
            # get current state
            js = self.model.get_current_state()
            # set the new joint values
            js.position = joint_values
            js.name = self.joint_names
            # calculate the forward kinematic
            fk_dict = self.model.forward_kinematic(js, links='all')
            # calculate cost based on the fixed frames
            C_fixed_frame = self.calculate_fixed_frame_cost(fk_dict, fixed_frames)
            # extract hand pose
            hand_pose = fk_dict[side + '_hand']
            # calculate the cost based on the task
            C_task = self.calculate_task_cost(hand_pose, i)
            # calculate the cost of having the object in sight
            head_pose = fk_dict['head']
            # if the task is received the model look at its hand
            if self.task == 'receive':
                C_sight = self.calculate_sight_cost(hand_pose, head_pose)
            # otherwise it looks at the object
            else:
                C_sight = self.calculate_sight_cost(self.object_pose, head_pose)
            # calculate REBA score
            C_reba = self.calculate_reba_cost(js)
            # return the final score
            cost.append(C_fixed_joints +
                        C_fixed_frame +
                        C_parameters +
                        self.cost_factors[0] * C_reba +
                        self.cost_factors[1] * C_task +
                        self.cost_factors[2] * C_sight)
            # append the detail of each cost
            cost_details['reba'][i] = C_reba
            cost_details['task'][i] = C_task
            cost_details['sight'][i] = C_sight

        print cost
        return max(cost)

    def generate_welding_points(self, welding_radius=0.2, nb_points=10):
        assert nb_points % 2 == 0
        self.welding_points = []
        x = linspace(-welding_radius, welding_radius, nb_points / 2)
        y = sqrt(welding_radius**2 - x**2)
        for i in range(nb_points / 2):
            self.welding_points.append([[x[i], y[i], 0], [0, 0, 0, 1]])
            self.welding_points.append([[x[i], -y[i], 0], [0, 0, 0, 1]])

    def initialize_task(self, nb_points):
        if self.task == 'welding':
            self.generate_welding_points(nb_points=nb_points)
            x = uniform(0.25, 1)
            y = uniform(-1., 1.)
            z = uniform(-0.5, 1.)
            q = uniform(-1., 1., 4)
            q /= norm(q)
            q = list(q)
            # create a quaternion for rotation
            init_pose = []
            limits = []
            if self.nb_params != 4:
                init_pose = [x, y, z]
                limits.append([0., 1.5])
                limits.append([-1., 1.])
                limits.append([-0.5, 1.])
            if self.nb_params != 3:
                init_pose += q
                limits += 4 * [[-1., 1.]]
            return init_pose, limits
        return [], []

    def optimize_posture(self, init_state, task, side='right', group_names='whole_body',
                         fixed_joints={}, fixed_frames={}, maxiter=100, nb_points=1, nb_params=0):
        self.task = task
        self.nb_params = nb_params
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
        joint_limits = []
        costs = {}
        costs['reba'] = []
        costs['task'] = []
        costs['sight'] = []
        for i in range(nb_points):
            costs['reba'].append(0)
            costs['task'].append(0)
            costs['sight'].append(0)
            for name in self.joint_names:
                init_joints.append(init_state.position[init_state.name.index(name)])
            # get the joints limits for the optimization
            joint_limits += self.model.get_joint_limits(self.joint_names)
        # add eventual parameters for the optimization
        opt_params, param_limits = self.initialize_task(nb_points)

        # account for maxiter
        if maxiter != -1:
            options = {'maxfun': maxiter}
        else:
            options = None
            maxiter = 1000

        # call optimization from scipy
        if self.optimizer == 'gradient':
            res = minimize(self.cost_function, init_joints + opt_params,
                           args=(side, fixed_joints, fixed_frames, costs, nb_points),
                           method='L-BFGS-B',
                           bounds=joint_limits + param_limits,
                           options=options)
        elif self.optimizer == 'evolution':
            res = differential_evolution(self.cost_function,
                                         args=(side, fixed_joints, fixed_frames, costs, nb_points),
                                         bounds=joint_limits + param_limits,
                                         maxiter=maxiter)

        nb_joints = (len(res.x) - self.nb_params) / nb_points
        for i in range(nb_points):
            opt_joints = res.x[i * nb_joints:(i + 1) * nb_joints]
            # convert it to a joint state
            js = self.model.get_current_state()
            js.position = list(js.position)
            js.name = list(js.name)
            # replace joint values with optimized ones
            for j in range(len(self.joint_names)):
                name = self.joint_names[j]
                js.position[js.name.index(name)] = opt_joints[j]
            # replace fixed values
            for key, value in fixed_joints.iteritems():
                js.position[js.name.index(key)] = value
            joint_traj.append(js)

        print res
        return joint_traj, costs
