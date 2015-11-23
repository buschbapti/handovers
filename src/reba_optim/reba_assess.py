import numpy as np
from collections import deque
import json
import os.path
from .tools.interpolation import *


class RebaAssess(object):
    def __init__(self, window_size=1, save_score=False, sum_optim=False):
        self.save = save_score
        self.sum_optim = sum_optim
        # initialize the reba table
        self.reba_table_init()
        self.reba_table_optim_init()
        # initialize the logger dict
        self.reba_logger_init(window_size)
        # factor for converting radians to degrees
        self.degrees_factor = 180./np.pi
        # initialize polynomial fit for optimization
        self.polynomial_fit = {}
        self.init_poly_fit()

    def reba_table_init(self):
        # group A table
        self.reba_A_table = np.zeros((3,5,4))
        self.reba_A_table[0] = [[1,2,3,4],[2,3,4,5],[2,4,5,6],[3,5,6,7],[4,6,7,8]]
        self.reba_A_table[1] = [[1,2,3,4],[3,4,5,6],[4,5,6,7],[5,6,7,8],[6,7,8,9]]
        self.reba_A_table[2] = [[3,3,5,6],[4,5,6,7],[5,6,7,8],[6,7,8,9],[7,7,8,9]]
        # group B table
        self.reba_B_table = np.zeros((2,6,3))
        self.reba_B_table[0] = [[1,2,2],[1,2,3],[3,4,5],[4,5,5],[6,7,8],[7,8,8]]
        self.reba_B_table[1] = [[1,2,3],[2,3,4],[4,5,5],[5,6,7],[7,8,8],[8,9,9]]
        # joint C table
        self.reba_C_table = np.zeros((12,12))
        self.reba_C_table[0] = [1,1,1,2,3,3,4,5,6,7,7,7]
        self.reba_C_table[1] = [1,2,2,3,4,4,5,6,6,7,7,8]
        self.reba_C_table[2] = [2,3,3,3,4,5,6,7,7,8,8,8]
        self.reba_C_table[3] = [3,4,4,4,5,6,7,8,8,9,9,9]
        self.reba_C_table[4] = [4,4,4,5,6,7,8,8,9,9,9,9]
        self.reba_C_table[5] = [6,6,6,7,8,8,9,9,10,10,10,10]
        self.reba_C_table[6] = [7,7,7,8,9,9,9,10,10,11,11,11]
        self.reba_C_table[7] = [8,8,8,9,10,10,10,10,10,11,11,11]
        self.reba_C_table[8] = [9,9,9,10,10,10,11,11,11,12,12,12]
        self.reba_C_table[9] = [10,10,10,11,11,11,11,12,12,12,12,12]
        self.reba_C_table[10] = [11,11,11,11,12,12,12,12,12,12,12,12]
        self.reba_C_table[11] = [12,12,12,12,12,12,12,12,12,12,12,12]

    def reba_table_optim_init(self):
        # create point and values for each table
        A_values = {}
        A_values['points'] = []
        # neck
        for i in range(3):
            # legs
            for j in range(4):
                # trunk
                for k in range(5):
                    A_values['points'].append([i+1,j+1,k+1])
        values = [1,2,2,3,4,2,3,4,5,6,3,4,5,6,7,4,5,6,7,8,1,3,4,5,6,2,4,5,6,7,3,5,6,7,8,4,6,7,8,9,3,4,5,6,7,3,5,6,7,8,5,6,7,8,9,6,7,8,9,9]
        A_values['values'] = values
        # B table
        B_values = {}
        B_values['points'] = []
        # lower arm
        for i in range(2):
            # wrist
            for j in range(3):
                # upper_arm
                for k in range(6):
                    B_values['points'].append([i+1,j+1,k+1])
        values = [1,1,3,4,6,7,2,2,4,5,7,8,2,3,5,5,8,8,1,2,4,5,7,8,2,3,5,6,8,9,3,4,5,7,8,9]
        B_values['values'] = values 
        # C table
        C_values = {}
        C_values['points'] = []
        # A table
        for i in range(12):
            # B table
            for j in range(12):
                C_values['points'].append([i+1,j+1])
        values = [1,1,1,2,3,3,4,5,6,7,7,7,1,2,2,3,4,4,5,6,6,7,7,8,2,3,3,3,4,5,6,7,7,8,8,8,3,4,4,4,5,6,7,8,8,9,9,9,4,4,4,5,6,7,8,8,9,9,9,9,
                6,6,6,7,8,8,9,9,10,10,10,10,7,7,7,8,9,9,9,10,10,11,11,11,8,8,8,9,10,10,10,10,10,11,11,11,9,9,9,10,10,10,11,11,11,12,12,12,
                10,10,10,11,11,11,11,12,12,12,12,12,11,11,11,11,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12]
        C_values['values'] = values 
        # use 3D interpolations to find the grid
        self.A_optim_table = get_table(A_values)
        self.B_optim_table = get_table(B_values)
        self.C_optim_table = get_table(C_values)

    def reba_logger_init(self, window_size):
        # create list of keys
        self.keys = ['neck','shoulder_R','elbow_R','wrist_R','shoulder_L','elbow_L','wrist_L',
                    'shoulders','elbows','wrists','trunk','legs','leg_R','leg_L',
                    'groupA','optimA','groupB','optimB','reba']
        # create the map of deque the reba score 
        self.reba_log = {}
        for k in self.keys:
            self.reba_log[k] = deque(maxlen=window_size)
            self.reba_log[k+"_windowed"] = 0.0

    def init_poly_fit(self):
        def side_poly_fit(side):
            # legs
            self.polynomial_fit["leg_"+side] = []
            self.polynomial_fit["leg_"+side].append(np.polyfit([-60,-30,0,30,60],[3,2,1,2,3],3)) # knee flexion
            # shoulders
            self.polynomial_fit["shoulder_"+side] = []
            self.polynomial_fit["shoulder_"+side].append(np.polyfit([-90,-45,-20,0,20,45,90],[4,3,2,1,2,3,4],3)) # flexion
            self.polynomial_fit["shoulder_"+side].append(np.polyfit([-90,-45,-20,0,20,45,90],[1,0.9,0.75,0,0.75,0.9,1],3)) # abduction
            self.polynomial_fit["shoulder_"+side].append(np.polyfit([-90,-45,-20,0,20,45,90],[1,0.9,0.75,0,0.75,0.9,1],3)) # twist
            # elbows
            self.polynomial_fit["elbow_"+side] = []
            self.polynomial_fit["elbow_"+side].append(np.polyfit([0,60,100],[2,1,2],3)) # flexion
            # wrists
            self.polynomial_fit["wrist_"+side] = []
            self.polynomial_fit["wrist_"+side].append(np.polyfit([-15,0,15],[2,1,2],3)) # flexion
            self.polynomial_fit["wrist_"+side].append(np.polyfit([-90,-45,-20,0,20,45,90],[1,0.9,0.75,0,0.75,0.9,1],3)) # bend
            self.polynomial_fit["wrist_"+side].append(np.polyfit([-90,-45,-20,0,20,45,90],[1,0.9,0.75,0,0.75,0.9,1],3)) # twist
        # trunk
        self.polynomial_fit["trunk"] = []
        self.polynomial_fit["trunk"].append(np.polyfit([-60,-20,0,20,60],[3,2,1,2,3],3)) # flexion
        self.polynomial_fit["trunk"].append(np.polyfit([-90,-45,-20,0,20,45,90],[1,0.9,0.75,0,0.75,0.9,1],3)) # bend
        self.polynomial_fit["trunk"].append(np.polyfit([-90,-45,-20,0,20,45,90],[1,0.9,0.75,0,0.75,0.9,1],3)) # twist
        # neck
        self.polynomial_fit["neck"] = []
        self.polynomial_fit["neck"].append(np.polyfit([-20,0,20],[2,1,2],3)) # flexion
        self.polynomial_fit["neck"].append(np.polyfit([-90,-45,-20,0,20,45,90],[1,0.9,0.75,0,0.75,0.9,1],3)) # bend
        self.polynomial_fit["neck"].append(np.polyfit([-90,-45,-20,0,20,45,90],[1,0.9,0.75,0,0.75,0.9,1],3)) # twist
        # sides
        side_poly_fit('R')
        side_poly_fit('L')

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

    def assess(self, data, epsilon=5):
        score = self.reba_assess(data,epsilon)
        return score

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

    def from_joints_to_reba(self, joints, names):
        def assign_value(name, sign=1, offset=0.):
            return sign*joints[names.index(name)]*self.degrees_factor + offset
        data = {}
        # trunk
        data["trunk"] = []
        data["trunk"].append(assign_value("spine_1")) # flexion
        data["trunk"].append(assign_value("spine_0")) # bend
        data["trunk"].append(assign_value("spine_2")) # twist
        # neck
        data["neck"] = []
        data["neck"].append(assign_value("neck_1")) # flexion
        data["neck"].append(assign_value("neck_0")) # bend
        data["neck"].append(assign_value("neck_2")) # twist
        # legs
        data["leg_R"] = [assign_value("right_knee",-1)] # TODO check ground contact
        data["leg_L"] = [assign_value("left_knee",-1)] # TODO check ground contact
        # shoulders
        data["shoulder_R"] = []
        data["shoulder_R"].append(assign_value("right_shoulder_1",1)) # flexion
        data["shoulder_R"].append(assign_value("right_shoulder_0",-1,90)) # abduction
        data["shoulder_R"].append(assign_value("right_shoulder_2")) # twist
        data["shoulder_L"] = []
        data["shoulder_L"].append(assign_value("left_shoulder_1",-1,)) # flexion
        data["shoulder_L"].append(assign_value("left_shoulder_0",-1,90)) # abduction
        data["shoulder_L"].append(assign_value("left_shoulder_2")) # twist
        # elbows
        data["elbow_R"] = [assign_value("right_elbow_0")]
        data["elbow_L"] = [assign_value("left_elbow_0",-1)]
        # wrist
        data["wrist_R"] = []
        data["wrist_R"].append(assign_value("right_wrist_1")) # flexion
        data["wrist_R"].append(assign_value("right_wrist_0")) # deviation
        data["wrist_R"].append(assign_value("right_elbow_1")) # twist
        data["wrist_L"] = []
        data["wrist_L"].append(assign_value("left_wrist_1")) # flexion
        data["wrist_L"].append(assign_value("left_wrist_0")) # deviation
        data["wrist_L"].append(assign_value("left_elbow_1",-1)) # twist
        return data

    def reba_optim(self, data):
        def polynomial_reba_value(joint, coeff):
            joint2 = joint*joint
            joint3 = joint2*joint
            return coeff[3] + joint*coeff[2] + joint2*coeff[1] + joint3*coeff[0]
        def score_dict(names):
            score = {} 
            for name in names:
                score[name] = 0
                joints = data[name]
                for i in range(len(joints)):
                    score[name] += polynomial_reba_value(joints[i], self.polynomial_fit[name][i])
            return score
        def sum_score_dict(d):
            dict_score = 0
            for key, value in d.iteritems():
                dict_score += value
            return dict_score
        def get_A_score():
            joints = ['neck', 'trunk', 'leg_R', 'leg_L']
            score = score_dict(joints)
            score['legs'] = max(score['leg_R'],score['leg_L'])
            # write the log
            if self.save:
                self.save_score_dict(score)
            # return the value from the table
            if score['neck'] > 3 or score['legs'] > 4 or score['trunk'] > 5 or self.sum_optim:
                return sum_score_dict(score), True
            else:    
                point = [score['neck'], score['legs'], score['trunk']]
                return get_value(point, self.A_optim_table), False
        def get_B_score():
            joints = ['shoulder_R', 'elbow_R', 'wrist_R', 'shoulder_L', 'elbow_L', 'wrist_L']
            score = score_dict(joints)    
            score['shoulders'] = max(score['shoulder_R'],score['shoulder_L'])
            score['elbows'] = max(score['elbow_R'],score['elbow_L'])
            score['wrists'] = max(score['wrist_R'],score['wrist_L'])
            # write the log
            if self.save:
                self.save_score_dict(score)
            # return the value from the table
            if score['shoulders'] > 6 or score['elbows'] > 2 or score['wrists'] > 3 or self.sum_optim:
                return sum_score_dict(score), True
            else:
                point = [score['elbows'], score['wrists'], score['shoulders']]
                return get_value(point, self.B_optim_table), False
        # get both A and B scores
        A_score, summed_A = get_A_score()
        B_score, summed_B = get_B_score()
        # save the log
        if self.save:
            self.save_score("optimA",A_score)
            self.save_score("optimB",B_score)
        # calculate reba score from C table
        if summed_A or summed_B:
            reba_score = A_score + B_score
        else:
            point = [A_score, B_score]
            reba_score = A_score + B_score # TODO replace by table calculation
            # reba_score =  get_value(point, self.C_optim_table)
        # save final log
        if self.save:
            self.save_score("reba",reba_score)
            self.save_log()
        return reba_score 
  
    def reba_assess(self,data,epsilon=5):
        # group A score calculation (trunk,neck,legs)
        def get_A_score():
            trunk_score = 0
            neck_score = 0
            legs_score = 0
            leg_R_score = 0
            leg_L_score = 0
            # get absolute values of angles
            trunk = np.absolute(data["trunk"])
            neck = np.absolute(data["neck"])
            leg_R = np.absolute(data["leg_R"])
            leg_L = np.absolute(data["leg_L"])
            ########### TRUNK ###############
            # flexion/extension
            if trunk[0] < epsilon:
                trunk_score += 1
            elif trunk[0] < 20:
                trunk_score += 2
            elif trunk[0] < 60:
                trunk_score += 3
            else:
                trunk_score += 4
            # side bending/twisting
            if trunk[1] > epsilon or trunk[2] > epsilon:
                trunk_score += 1
            ########## NECK ############
            # flexion/extension
            if neck[0] < 20:
                neck_score += 1
            else:
                neck_score += 2
            # side bending/twisting
            if neck[1] > epsilon or neck[2] > epsilon:
                neck_score += 1
            ########## LEGS ###########
            # both legs in contact with the ground
            if leg_R[1] == 1 and leg_L[1] == 1:
                legs_score += 1
            else:
                legs_score += 2
            # knees inclination
            if leg_R[0] > 60:
                leg_R_score += 2
            elif leg_R[0] > 30:
                leg_R_score += 1
            if leg_L[0] > 60:
                leg_L_score += 2
            elif leg_L[0] > 30:
                leg_L_score += 1
            ######### A_SCORE ########
            # calcualte optim score
            optim_score = trunk_score + neck_score + leg_R_score + leg_L_score + legs_score
            # calcalute reba score
            legs_score += max(leg_R_score, leg_L_score)
            score = self.reba_A_table[neck_score-1,trunk_score-1,legs_score-1] 
            ######### LOAD ###########
            if data["load"] > 5 and data["load"] < 10:
                score += 1
            elif data["load"] > 10:
                score += 2
            # write the log
            if self.save:
                self.save_score("trunk",trunk_score)
                self.save_score("neck",neck_score)
                self.save_score("legs",legs_score)
                self.save_score("leg_R",leg_R_score)
                self.save_score("leg_L",leg_L_score)
                self.save_score("groupA",score)
                self.save_score("optimA",optim_score)
            # return the score
            return score, optim_score
        # group B score calculation (shoulders,elbows,wrists)
        def get_B_score():
            def side_score(side):
                shoulder_score = 0
                elbow_score = 0
                wrist_score = 0
                # get absolute values of angles
                shoulder = np.absolute(data["shoulder_"+side])
                elbows = np.absolute(data["elbow_"+side])
                wrists = np.absolute(data["wrist_"+side])
                ######### SHOULDERS ##########
                # flexion/extension
                if shoulder[0] < 20 + epsilon:
                    shoulder_score += 1
                elif shoulder[0] < 45 + epsilon:
                    shoulder_score += 2
                elif shoulder[0] < 90 + epsilon:
                    shoulder_score += 3
                else:
                    shoulder_score += 4
                # abduction/rotation
                if shoulder[1] > epsilon or shoulder[2] > epsilon:
                    shoulder_score += 1
                ########## ELBOWS ############
                # flexion/etension
                if elbows[0] > 60 - epsilon and elbows[0] < 100 + epsilon:
                    elbow_score += 1
                else:
                    elbow_score += 2
                ######### WRISTS ###########
                # flexion/extension
                if wrists[0] < 15 + epsilon:
                    wrist_score += 1
                else:
                    wrist_score += 2
                # deviation/twisting
                if wrists[1] > epsilon or wrists[2] > epsilon:
                    wrist_score += 1
                # write the log
                if self.save:
                    self.save_score("shoulder_"+side,shoulder_score)
                    self.save_score("elbow_"+side,elbow_score)
                    self.save_score("wrist_"+side,wrist_score)
                return shoulder_score, elbow_score, wrist_score
            # calculate scores for both sides
            shoulder_R_score, elbow_R_score, wrist_R_score = side_score('R')
            shoulder_L_score, elbow_L_score, wrist_L_score = side_score('L')
            optim_score = shoulder_R_score + shoulder_L_score + elbow_R_score + elbow_L_score + wrist_R_score + wrist_L_score
            # calculate reba score
            shoulders_score = max(shoulder_R_score, shoulder_L_score)
            elbows_score = max(elbow_R_score, elbow_L_score)
            wrists_score = max(wrist_R_score, wrist_L_score)
            score = self.reba_B_table[elbows_score-1,shoulders_score-1,wrists_score-1]
            # write the log
            if self.save:
                self.save_score("shoulders",shoulders_score)
                self.save_score("elbows",elbows_score)
                self.save_score("wrists",wrists_score)
                self.save_score("groupB",score)
                self.save_score("optimB",optim_score)
            # return the score
            return score, optim_score
        # first get A and B score
        A_score, optimA = get_A_score()
        B_score, optimB = get_B_score()
        # calculate reba score from reba table
        reba_score = self.reba_C_table[A_score-1,B_score-1]
        # add activity score
        if data["static"] or data["high_dynamic"]:
            reba_score += 1
        # save the log
        if self.save:
            self.save_score("reba",reba_score)
            self.save_log()
        # return reba score
        return reba_score, (optimA+optimB)



