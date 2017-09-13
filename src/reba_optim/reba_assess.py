import numpy as np
from collections import deque
import json
import os.path
from keras.models import load_model
import rospkg
from os.path import join
from moveit_msgs.msg import RobotState


class RebaAssess(object):
    def __init__(self, deque_size=-1):
        # initialize the reba table
        self.reba_table_init()
        # initialize the logger dict
        self.reba_logger_init(deque_size)
        # initialize polynomial fit for optimization
        self.polynomial_fit = {}
        self.polynomial_fit2 = {}
        self.A_coeff = []
        self.B_coeff = []
        self.init_reba_dict()
        self.init_poly_fit()
        self.init_poly_fit2()
        self.init_coeffs()
        # load network for REBA assess
        rospack = rospkg.RosPack()
        pkg_dir = rospack.get_path('reba_optim')
        config_dir = join(pkg_dir, 'config')

        modelName = "reba3"
        modelVersion = 2
        modelName = "model-" + modelName + '-' + str(modelVersion)
        self.neural_model = load_model(join(config_dir, modelName + '-full.h5'))

        self.assess_joints = ['neck_0', 'neck_1', 'neck_2', 'spine_0', 'spine_1', 'spine_2', 'right_knee_0',
                              'left_knee_0', 'right_shoulder_0', 'right_shoulder_1', 'left_shoulder_0',
                              'left_shoulder_1', 'right_elbow_0', 'left_elbow_0', 'right_wrist_0',
                              'right_wrist_1', 'right_wrist_2', 'left_wrist_0', 'left_wrist_1', 'left_wrist_2']

    def reba_table_init(self):
        # group A table
        self.reba_A_table = np.zeros((3, 5, 4))
        self.reba_A_table[0] = [[1, 2, 3, 4], [2, 3, 4, 5],
                                [2, 4, 5, 6], [3, 5, 6, 7],
                                [4, 6, 7, 8]]
        self.reba_A_table[1] = [[1, 2, 3, 4], [3, 4, 5, 6],
                                [4, 5, 6, 7], [5, 6, 7, 8],
                                [6, 7, 8, 9]]
        self.reba_A_table[2] = [[3, 3, 5, 6], [4, 5, 6, 7],
                                [5, 6, 7, 8], [6, 7, 8, 9],
                                [7, 7, 8, 9]]
        # group B table
        self.reba_B_table = np.zeros((2, 6, 3))
        self.reba_B_table[0] = [[1, 2, 2], [1, 2, 3],
                                [3, 4, 5], [4, 5, 5],
                                [6, 7, 8], [7, 8, 8]]
        self.reba_B_table[1] = [[1, 2, 3], [2, 3, 4],
                                [4, 5, 5], [5, 6, 7],
                                [7, 8, 8], [8, 9, 9]]
        # joint C table
        self.reba_C_table = np.zeros((12, 12))
        self.reba_C_table[0] = [1, 1, 1, 2, 3, 3, 4, 5, 6, 7, 7, 7]
        self.reba_C_table[1] = [1, 2, 2, 3, 4, 4, 5, 6, 6, 7, 7, 8]
        self.reba_C_table[2] = [2, 3, 3, 3, 4, 5, 6, 7, 7, 8, 8, 8]
        self.reba_C_table[3] = [3, 4, 4, 4, 5, 6, 7, 8, 8, 9, 9, 9]
        self.reba_C_table[4] = [4, 4, 4, 5, 6, 7, 8, 8, 9, 9, 9, 9]
        self.reba_C_table[5] = [6, 6, 6, 7, 8, 8, 9, 9, 10, 10, 10, 10]
        self.reba_C_table[6] = [7, 7, 7, 8, 9, 9, 9, 10, 10, 11, 11, 11]
        self.reba_C_table[7] = [8, 8, 8, 9, 10, 10, 10, 10, 10, 11, 11, 11]
        self.reba_C_table[8] = [9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12]
        self.reba_C_table[9] = [10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12]
        self.reba_C_table[10] = [11, 11, 11, 11, 12,
                                 12, 12, 12, 12, 12, 12, 12]
        self.reba_C_table[11] = [12, 12, 12, 12, 12,
                                 12, 12, 12, 12, 12, 12, 12]

    def init_coeffs(self):
        # calculated from the notebook
        self.A_coeff = [0.03822852, 0.75622637, 0.7621335]
        self.B_coeff = [0.06068095, 0.68147439, 0.03933057]

        self.discount_factor = {}
        self.discount_factor['neck'] = [0.30283341, 1.00822059, 0.52364692]
        self.discount_factor['trunk'] = [1.51872174, 1.39895715, 1.85629944]
        self.discount_factor['legs'] = [0.41615015]
        self.discount_factor['elbows'] = [0.10742893, 0.10742893]
        self.discount_factor['shoulders'] = [0.24304587, 0.55354204]
        self.discount_factor['wrists'] = [0.09637051, 0.84326106, 0.69796194]
        self.discount_factor['residual'] = -0.20583487

    def reba_logger_init(self, deque_size):
        # create list of keys
        self.keys = ['neck', 'right_shoulder', 'right_elbow', 'right_wrist',
                     'left_shoulder', 'left_elbow', 'left_wrist',
                     'shoulders', 'elbows', 'wrists', 'trunk',
                     'legs', 'right_leg', 'left_leg',
                     'groupA', 'groupB', 'reba']
        # create the map of deque the reba score
        self.reba_log = {}
        for k in self.keys:
            # default deque size -1 mean infinite log
            if deque_size == -1:
                self.reba_log[k] = []
            else:
                self.reba_log[k] = deque(maxlen=deque_size)
            self.reba_log[k + "_windowed"] = 0.0

    def init_poly_fit(self):
        # legs
        self.polynomial_fit["legs"] = []
        self.polynomial_fit["legs"].append(
            np.polyfit([-1.0472, 0., 1.0472],
                       [2., 0., 2.], 2))  # knee flexion
        self.polynomial_fit["deriv_legs"] = []
        # for p in self.polynomial_fit["legs"]:
        #     self.polynomial_fit["deriv_legs"].append(
        #         np.polyder(p))
        # shoulders
        self.polynomial_fit["shoulders"] = []
        self.polynomial_fit["shoulders"].append(
            np.polyfit([0.7854, 1.5708, 2.3561], [1., 0., 1.], 2))  # abduction
        self.polynomial_fit["shoulders"].append(
            np.polyfit([-1.5708, 0., 1.5708],
                       [3., 0., 3.], 2))  # flexion
        self.polynomial_fit["shoulders"].append(
            # np.polyfit([-1.5708, 0, 1.5708], [1, 0, 1], 2))  # twist
            np.polyfit([-0.7854, 0., 0.7854],
                       [1., 0., 1.], 2))
        # self.polynomial_fit["deriv_shoulders"] = []
        # for p in self.polynomial_fit["shoulders"]:
        #     self.polynomial_fit["deriv_shoulders"].append(
        #         np.polyder(p))
        # elbows
        self.polynomial_fit["elbows"] = []
        self.polynomial_fit["elbows"].append(
            np.polyfit([0., 1.0472, 1.7453], [1., 0., 1.], 2))  # flexion right side
        self.polynomial_fit["elbows"].append(
            np.polyfit([0., -1.0472, -1.7453], [1., 0., 1.], 2))  # flexion left side
        # self.polynomial_fit["deriv_elbows"] = []
        # for p in self.polynomial_fit["elbows"]:
        #     self.polynomial_fit["deriv_elbows"].append(
        #         np.polyder(p))
        # wrists
        self.polynomial_fit["wrists"] = []
        self.polynomial_fit["wrists"].append(
            # np.polyfit([-1.5708, 0, 1.5708], [1, 0, 1], 2))  # bend
            np.polyfit([-0.7854, 0., 0.7854], [1., 0., 1.], 2))
        self.polynomial_fit["wrists"].append(
            np.polyfit([-0.7854, 0., 0.7854], [1., 0., 1.], 2))  # flexion
        self.polynomial_fit["wrists"].append(
            # np.polyfit([-1.5708, 0, 1.5708], [1, 0, 1], 2))  # twist
            np.polyfit([-0.7854, 0., 0.7854], [1., 0., 1.], 2))
        # self.polynomial_fit["deriv_wrists"] = []
        # for p in self.polynomial_fit["wrists"]:
        #     self.polynomial_fit["deriv_wrists"].append(
        #         np.polyder(p))
        # trunk
        self.polynomial_fit["trunk"] = []
        self.polynomial_fit["trunk"].append(
            # np.polyfit([-1.5708, 0, 1.5708], [1, 0, 1], 2))  # bend
            np.polyfit([-0.7854, 0., 0.7854], [1., 0., 1.], 2))
        self.polynomial_fit["trunk"].append(
            np.polyfit([-1.5708, 0., 1.5708], [3., 0., 3.], 2))  # flexion
        self.polynomial_fit["trunk"].append(
            # np.polyfit([-1.5708, 0, 1.5708], [1, 0, 1], 2))  # twist
            np.polyfit([-0.7854, 0.0, 0.7854], [1., 0., 1.], 2))
        # self.polynomial_fit["deriv_trunk"] = []
        # for p in self.polynomial_fit["trunk"]:
        #     self.polynomial_fit["deriv_trunk"].append(
        #         np.polyder(p))
        # neck
        self.polynomial_fit["neck"] = []
        self.polynomial_fit["neck"].append(
            # np.polyfit([-1.5708, 0, 1.5708], [1, 0, 1], 2))  # bend
            np.polyfit([-0.78539816339, 0.0, 0.78539816339], [1., 0, 1.], 2))
        self.polynomial_fit["neck"].append(
            np.polyfit([-0.6109, 0.1745, 0.9599], [1., 0., 1.], 2))  # flexion
        self.polynomial_fit["neck"].append(
            # np.polyfit([-1.5708, 0, 1.5708], [1, 0, 1], 2))  # twist
            np.polyfit([-0.78539816339, 0.0, 0.78539816339], [1, 0, 1], 2))
        # self.polynomial_fit["deriv_neck"] = []
        # for p in self.polynomial_fit["neck"]:
        #     self.polynomial_fit["deriv_neck"].append(
        #         np.polyder(p))

    def init_poly_fit2(self):
        # legs
        self.polynomial_fit2["legs"] = []
        self.polynomial_fit2["legs"].append(
            np.polyfit([-1.0472, 0, 1.0472],
                       [3, 1, 3], 2))  # knee flexion
        # shoulders
        self.polynomial_fit2["shoulders"] = []
        self.polynomial_fit2["shoulders"].append(
            np.polyfit([0.0, 1.5708, 3.14157], [1, 0, 1], 2))  # abduction
        self.polynomial_fit2["shoulders"].append(
            np.polyfit([-1.5708, 0.0, 1.5708],
                       [4, 1, 4], 2))  # flexion
        self.polynomial_fit2["shoulders"].append(
            # np.polyfit([-1.5708, 0, 1.5708], [1, 0, 1], 2))  # twist
            np.polyfit([-1.5708, 0.0, 1.5708],
                       [0, 0, 0], 2))
        # elbows
        self.polynomial_fit2["elbows"] = []
        self.polynomial_fit2["elbows"].append(
            np.polyfit([0, 1.0472, 2.53073], [2, 1, 2], 2))  # flexion right side
        self.polynomial_fit2["elbows"].append(
            np.polyfit([0, -1.0472, -2.53073], [2, 1, 2], 2))  # flexion left side
        # wrists
        self.polynomial_fit2["wrists"] = []
        self.polynomial_fit2["wrists"].append(
            # np.polyfit([-1.5708, 0, 1.5708], [1, 0, 1], 2))  # bend
            np.polyfit([-1.0472, 0, 1.0472], [1, 0, 1], 2))
        self.polynomial_fit2["wrists"].append(
            np.polyfit([-1.0472, 0, 1.0472], [2, 1, 2], 2))  # flexion
        self.polynomial_fit2["wrists"].append(
            # np.polyfit([-1.5708, 0, 1.5708], [1, 0, 1], 2))  # twist
            np.polyfit([-1.0472, 0, 1.0472], [1, 0, 1], 2))
        # trunk
        self.polynomial_fit2["trunk"] = []
        self.polynomial_fit2["trunk"].append(
            # np.polyfit([-1.5708, 0, 1.5708], [1, 0, 1], 2))  # bend
            np.polyfit([-1.0472, 0.0, 1.0472], [1, 0, 1], 2))
        self.polynomial_fit2["trunk"].append(
            np.polyfit([-1.0472, 0.0, 1.0472], [3, 1, 3], 2))  # flexion
        self.polynomial_fit2["trunk"].append(
            # np.polyfit([-1.5708, 0, 1.5708], [1, 0, 1], 2))  # twist
            np.polyfit([-1.0472, 0.0, 1.0472], [1, 0, 1], 2))
        # neck
        self.polynomial_fit2["neck"] = []
        self.polynomial_fit2["neck"].append(
            # np.polyfit([-1.5708, 0, 1.5708], [1, 0, 1], 2))  # bend
            np.polyfit([-1.39626, 0.174533, 1.5708], [1, 0, 1], 2))
        self.polynomial_fit2["neck"].append(
            np.polyfit([-1.39626, 0.174533, 1.5708], [2, 1, 2], 2))  # flexion
        self.polynomial_fit2["neck"].append(
            # np.polyfit([-1.5708, 0, 1.5708], [1, 0, 1], 2))  # twist
            np.polyfit([-1.39626, 0.174533, 1.5708], [1, 0, 1], 2))

    def save_score(self, key, score):
        self.reba_log[key].append(score)
        # calculate the windowed score
        w_score = sum(self.reba_log[key]) / len(self.reba_log[key])
        self.reba_log[key + "_windowed"] = w_score

    def save_score_dict(self, data):
        for key, value in data.iteritems():
            self.save_score(key, value)

    def windowed_score(self, key):
        return self.reba_log[key + "_windowed"]

    def serialize_logger(self):
        data = {}
        for k in self.keys:
            data[k] = list(self.reba_log[k])
            data[k + '_windowed'] = self.reba_log[k + '_windowed']
        return data

    def save_log(self):
        # read the json file score
        if os.path.isfile('/tmp/pause_log.json'):
            with open('/tmp/pause_log.json') as data_file:
                log_data = json.load(data_file)
        else:
            log_data = {}
        # append the score
        log_data["reba_asses"] = self.serialize_logger()
        with open('/tmp/pause_log.json', 'w') as outfile:
            json.dump(log_data, outfile, indent=4, sort_keys=True)

    def assign_value(self, joint, group_name, index):
        coeffs = self.polynomial_fit[group_name][index]
        return np.polyval(coeffs, joint) * self.discount_factor[group_name][index]

    def assign_value2(self, joint, group_name, index):
        coeffs = self.polynomial_fit2[group_name][index]
        return np.polyval(coeffs, joint)

    def init_reba_dict(self):
        self.reba_dict = {}
        for i in range(2):
            self.reba_dict['neck_' + str(i)] = {'group_name': 'neck', 'index': i}
            self.reba_dict['spine_' + str(i)] = {'group_name': 'trunk', 'index': i}
            self.reba_dict['right_shoulder_' + str(i)] = {'group_name': 'shoulders', 'index': i}
            self.reba_dict['left_shoulder_' + str(i)] = {'group_name': 'shoulders', 'index': i}
            self.reba_dict['right_wrist_' + str(i)] = {'group_name': 'wrists', 'index': i}
            self.reba_dict['left_wrist_' + str(i)] = {'group_name': 'wrists', 'index': i}
        # add missing joints
        self.reba_dict['neck_2'] = {'group_name': 'neck', 'index': 2}
        self.reba_dict['spine_2'] = {'group_name': 'trunk', 'index': 2}
        # self.reba_dict['right_shoulder_2'] = {'group_name': 'shoulders', 'index': 2}
        # self.reba_dict['left_shoulder_2'] = {'group_name': 'shoulders', 'index': 2}
        self.reba_dict['right_wrist_2'] = {'group_name': 'wrists', 'index': 2}
        self.reba_dict['left_wrist_2'] = {'group_name': 'wrists', 'index': 2}
        self.reba_dict['right_knee_0'] = {'group_name': 'legs', 'index': 0}
        self.reba_dict['left_knee_0'] = {'group_name': 'legs', 'index': 0}
        self.reba_dict['right_elbow_0'] = {'group_name': 'elbows', 'index': 0}
        self.reba_dict['left_elbow_0'] = {'group_name': 'elbows', 'index': 1}

    def deriv_assess_posture(self, joints, names):
        # calculate each score individually
        deriv = []
        for i in range(len(names)):
            if names[i] in self.reba_dict:
                reba_group = self.reba_dict[names[i]]
                deriv.append(self.assign_value(joints[i],
                                               'deriv_' + reba_group['group_name'],
                                               reba_group['index']))
            else:
                deriv.append(0)
        return deriv

    def assess_posture_close(self, joints, names, save_dict):
        # calculate the reba based on sum of polynoms
        for i in range(len(names)):
            if names[i] in self.reba_dict:
                reba_group = self.reba_dict[names[i]]
                value = self.assign_value2(joints[i],
                                           reba_group['group_name'],
                                           reba_group['index'])
                if names[i] in save_dict.keys():
                    save_dict[names[i]].append(value)

    def assess_group(self, state, keys):
        score_dict = {}
        for k in keys:
            score = 0
            for i in range(k[1]):
                name = k[0] + '_' + str(i)
                reba_group = self.reba_dict[name]
                value = self.assign_value2(state.position[state.name.index(name)],
                                           reba_group['group_name'],
                                           reba_group['index'])
                score += value
            score_dict[k[0]] = score
        return score_dict

    def _assess_from_polynomes(self, joint_state):
        # calculate the reba based on sum of polynoms
        sum_reba = 0.
        names = joint_state.name
        joints = joint_state.position
        for i in range(len(names)):
            if names[i] in self.reba_dict:
                reba_group = self.reba_dict[names[i]]
                value = self.assign_value(joints[i],
                                          reba_group['group_name'],
                                          reba_group['index'])
                sum_reba += value
        return sum_reba + self.discount_factor['residual']

    def _assess_from_neural_model(self, joint_state):
        X = []
        for j in self.assess_joints:
            if j in joint_state.name:
                X.append(joint_state.position[joint_state.name.index(j)])
            else:
                X.append(0.0)
        prediction = self.neural_model.predict([np.array([X])])
        return float(np.argmax(prediction)) + 1.

    def assess_posture(self, state, method):
        if isinstance(state, RobotState):
            state = state.joint_state
        if method == 'polynomial':
            return self._assess_from_polynomes(state)
        elif method == 'neural_network':
            return self._assess_from_neural_model(state)
        else:
            print 'Unknown assessment method'
            return 0
