from sympy import *
import rospy
from kinect_skeleton_publisher.joint_transformations import *
from sympy import *

class ReadModel:
    def __init__(self, model=None):
        # get the lengths of the human
        self.nb_joints = 10
        self.lengths = rospy.get_param('/kinect/human_lengths')
        self.from_model_to_transformations(model)

    def from_model_to_transformations(self, model):
        # for now create a fake model
        self.end_effectors = ['right_hand']
        self.chain = []
        # create sympy variables
        t = []
        for i in range(self.nb_joints):
            t.append(Symbol('t'+str(i)))
        # get the lengths of each segmen
        waist = self.lengths['waist_length']
        shoulder_width = self.lengths['shoulder_offset_width']
        shoulder_height = self.lengths['shoulder_offset_height']
        upper_arm = self.lengths['upper_arm_length']
        forearm = self.lengths['forearm_length']
        hand = self.lengths['hand_length']
        # create transformations chains
        spine0 = translation([0,0,waist])*rotation_x(t[0])
        spine1 = rotation_y(t[1])
        spine2 = rotation_z(t[2])
        shoulder0 = translation([0,-shoulder_width,shoulder_height])*rotation_z(-pi/2)*rotation_z(t[3])
        shoulder1 = rotation_y(t[4])
        shoulder2 = rotation_x(t[5])
        elbow0 = translation([upper_arm,0,0])*rotation_z(t[6])
        elbow1 = rotation_x(t[7])
        wrist0 = translation([forearm,0,0])*rotation_z(t[8])
        wrist1 = rotation_y(t[9])
        hand = translation([hand,0,0])
        # append all transformations
        self.chain.append(spine0)
        self.chain.append(spine1)
        self.chain.append(spine2)
        self.chain.append(shoulder0)
        self.chain.append(shoulder1)
        self.chain.append(shoulder2)
        self.chain.append(elbow0)
        self.chain.append(elbow1)
        self.chain.append(wrist0)
        self.chain.append(wrist1)
        self.chain.append(hand)

    def forward_kinematic(self, joints):
        T = eye(4)
        for i in range(len(self.chain)-1):
            # evaluate the transformation matrix with the corresponding joint value
            temp = self.chain[i].subs('t'+str(i), joints[i]).evalf()
            # postmultiply the chain
            T = T*temp
        # multiply with the transformation to the end-effector
        T = T*self.chain[-1]
        return T


