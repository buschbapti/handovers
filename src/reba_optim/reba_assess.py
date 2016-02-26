import numpy as np
from collections import deque
import json
import os.path
from .tools.interpolation import *


class RebaAssess(object):
    def __init__(self, save_score=False, deque_size=-1):
        self.save = save_score
        # initialize the reba table
        self.reba_table_init()
        # initialize the logger dict
        self.reba_logger_init(deque_size)
        # initialize polynomial fit for optimization
        self.polynomial_fit = {}
        self.A_coeff = []
        self.B_coeff = []
        self.init_poly_fit()
        self.init_coeffs()

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
        self.A_coeff = [0.03822852,  0.75622637,  0.7621335]
        self.B_coeff = [0.06068095,  0.68147439, 0.03933057]

    def reba_logger_init(self, deque_size):
        # create list of keys
        self.keys = ['neck', 'shoulder_R', 'elbow_R', 'wrist_R',
                     'shoulder_L', 'elbow_L', 'wrist_L',
                     'shoulders', 'elbows', 'wrists', 'trunk',
                     'legs', 'leg_R', 'leg_L',
                     'groupA', 'groupB', 'reba']
        # create the map of deque the reba score
        self.reba_log = {}
        for k in self.keys:
            # default deque size -1 mean infinite log
            if deque_size == -1:
                self.reba_log[k] = []
            else:
                self.reba_log[k] = deque(maxlen=deque_size)
            self.reba_log[k+"_windowed"] = 0.0

    def init_poly_fit(self):
        # legs
        self.polynomial_fit["legs"] = []
        self.polynomial_fit["legs"].append(
            np.polyfit([-1.0472, -0.523599, 0, 0.523599, 1.0472],
                       [3, 2, 1, 2, 3], 2))  # knee flexion
        # shoulders
        self.polynomial_fit["shoulders"] = []
        self.polynomial_fit["shoulders"].append(
            np.polyfit([-3.14157, -1.5708, 0], [1, 0, 1], 2))  # abduction
        self.polynomial_fit["shoulders"].append(
            np.polyfit([-1.5708, -0.785398, -0.349066, 0, 0.349066, 0.785398, 1.5708],
                       [4, 3, 2, 1, 2, 3, 4], 2))  # flexion
        self.polynomial_fit["shoulders"].append(
            np.polyfit([-1.5708, 0, 1.5708], [1, 0, 1], 2))  # twist
        # elbows
        self.polynomial_fit["elbows"] = []
        self.polynomial_fit["elbows"].append(
            np.polyfit([0, 1.0472, 1.74533], [2, 1, 2], 2))  # flexion right side
        self.polynomial_fit["elbows"].append(
            np.polyfit([0, -1.0472, -1.74533], [2, 1, 2], 2))  # flexion left side
        # wrists
        self.polynomial_fit["wrists"] = []
        self.polynomial_fit["wrists"].append(
            np.polyfit([-1.5708, 0, 1.5708], [1, 0, 1], 2))  # bend
        self.polynomial_fit["wrists"].append(
            np.polyfit([-0.78539816339, 0, 0.78539816339], [2, 1, 2], 2))  # flexion
        self.polynomial_fit["wrists"].append(
            np.polyfit([-1.5708, 0, 1.5708], [1, 0, 1], 2))  # twist
        # trunk
        self.polynomial_fit["trunk"] = []
        self.polynomial_fit["trunk"].append(
            np.polyfit([-1.5708, 0, 1.5708], [1, 0, 1], 2))  # bend
        self.polynomial_fit["trunk"].append(
            np.polyfit([-1.0472, -0.349066, 0, 0.349066, 1.0472], [3, 2, 1, 2, 3], 2))  # flexion
        self.polynomial_fit["trunk"].append(
            np.polyfit([-1.5708, 0, 1.5708], [1, 0, 1], 2))  # twist
        # neck
        self.polynomial_fit["neck"] = []
        self.polynomial_fit["neck"].append(np.polyfit(
            [-1.5708, 0, 1.5708], [1, 0, 1], 2))  # bend
        self.polynomial_fit["neck"].append(
            np.polyfit([-1.39626, 0.174533, 1.5708], [2, 1, 2], 2))  # flexion
        self.polynomial_fit["neck"].append(np.polyfit(
            [-1.5708, 0, 1.5708], [1, 0, 1], 2))  # twist

    def save_score(self, key, score):
        self.reba_log[key].append(score)
        # calculate the windowed score
        w_score = sum(self.reba_log[key])/len(self.reba_log[key])
        self.reba_log[key+"_windowed"] = w_score

    def save_score_dict(self, data):
        for key, value in data.iteritems():
            self.save_score(key, value)

    def windowed_score(self, key):
        return self.reba_log[key+"_windowed"]

    def serialize_logger(self):
        data = {}
        for k in self.keys:
            data[k] = list(self.reba_log[k])
            data[k+'_windowed'] = self.reba_log[k+'_windowed']
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

    def assess_posture(self, joints, names):
        def assign_value(name, group_name, index):
            joint = joints[names.index(name)]
            coeffs = self.polynomial_fit[group_name][index]
            return np.polyval(coeffs, joint)

        # calculate each score individually
        score = {}
        score['neck'] = 0
        score['trunk'] = 0
        score['legs'] = 0
        score['shoulders'] = 0
        score['elbows'] = 0
        score['wrists'] = 0
        score['leg_R'] = 0
        score['shoulder_R'] = 0
        score['elbow_R'] = 0
        score['wrist_R'] = 0
        score['leg_L'] = 0
        score['shoulder_L'] = 0
        score['elbow_L'] = 0
        score['wrist_L'] = 0
        # start with group of two joints
        for i in range(2):
            score['neck'] += assign_value('neck_'+str(i), 'neck', i)
            score['trunk'] += assign_value('spine_'+str(i), 'trunk', i)
            score['shoulder_R'] += assign_value('right_shoulder_'+str(i), 'shoulders', i)
            score['shoulder_L'] += assign_value('left_shoulder_'+str(i), 'shoulders', i)
            score['wrist_R'] += assign_value('right_wrist_'+str(i), 'wrists', i)
            score['wrist_L'] += assign_value('left_wrist_'+str(i), 'wrists', i)
        # add missing joints
        score['neck'] += assign_value('neck_'+str(2), 'neck', 2)
        score['trunk'] += assign_value('spine_'+str(2), 'trunk', 2)
        score['leg_R'] += assign_value('right_knee', 'legs', 0)
        score['leg_L'] = assign_value('left_knee', 'legs', 0)
        score['shoulder_R'] += assign_value('right_shoulder_'+str(2), 'shoulders', 2)
        score['shoulder_L'] += assign_value('left_shoulder_'+str(2), 'shoulders', 2)
        score['elbow_R'] += assign_value('right_elbow_'+str(0), 'elbows', 0)
        score['elbow_L'] += assign_value('left_elbow_'+str(0), 'elbows', 1)
        # specific case elbow twisting which is considered as wrist in reba
        score['wrist_R'] += assign_value('right_elbow_'+str(1), 'wrists', 2)
        score['wrist_L'] += assign_value('left_elbow_'+str(1), 'wrists', 2)
        # for legs, wrists, elbows and joints takes the average on both side
        score['legs'] += (score['leg_R'] + score['leg_L'])/2
        score['wrists'] += (score['wrist_R'] + score['wrist_L'])/2
        score['elbows'] += (score['elbow_R'] + score['elbow_L'])/2
        score['shoulders'] += (score['shoulder_R'] + score['shoulder_L'])/2
        # calculate the A and B score
        score['groupA'] = (self.A_coeff[0]*score['neck'] +
                           self.A_coeff[1]*score['trunk'] +
                           self.A_coeff[2]*score['legs'])
        score['groupB'] = (self.B_coeff[0]*score['elbows'] +
                           self.B_coeff[1]*score['shoulders'] +
                           self.B_coeff[2]*score['wrists'])
        # calculate the total reba score
        score['reba'] = score['groupA'] + score['groupB']
        # save the score
        if self.save:
            self.save_score_dict(score)
        return score['reba']
