from sympy import *
import rospy
from kinect_skeleton_publisher.joint_transformations import *
import xmltodict

class ReadModel:
    def __init__(self):
        # get the lengths of the human
        self.nb_joints = [10,6]
        self.lengths = rospy.get_param('/kinect/human_lengths')
        self.end_effectors = []
        self.joint_names = []
        self.init_joint_names()
        self.leg_angles = []
        self.from_model_to_transformations()
        self.init_leg_angles()

    def init_joint_names(self):
        # create the list of all joints by appending first spherical joints
        self.end_effectors.append('right_hand')
        self.end_effectors.append('left_hand')
        self.end_effectors.append('right_foot')
        self.end_effectors.append('left_foot')
        for i in range(3):
            self.joint_names.append('spine_'+str(i))
            self.joint_names.append('neck_'+str(i))
            self.joint_names.append('right_shoulder_'+str(i))
            self.joint_names.append('left_shoulder_'+str(i))
            self.joint_names.append('right_hip_'+str(i))
            self.joint_names.append('left_hip_'+str(i))
        # then append universal joints
        for i in range(2):
            self.joint_names.append('right_elbow_'+str(i))
            self.joint_names.append('left_elbow_'+str(i))
            self.joint_names.append('right_wrist_'+str(i))
            self.joint_names.append('left_wrist_'+str(i))
            self.joint_names.append('right_ankle_'+str(i))
            self.joint_names.append('left_ankle_'+str(i))
        # finish by the knee joint
        self.joint_names.append('right_knee')
        self.joint_names.append('left_knee')

    def init_leg_angles(self):
        thigh = self.lengths['thigh_length']
        shin = self.lengths['shin_length']
        foot = self.lengths['foot_length']
        k = Symbol('k')
        # create the symbolic angles that will be used for the calculation
        a = sqrt(shin**2 + thigh**2 - 2*shin*thigh*cos(k))
        self.leg_angles.append(acos((a**2 + thigh**2 - shin**2)/(2*a*thigh)))
        self.leg_angles.append(acos((a**2 + shin**2 - thigh**2)/(2*a*shin)))

    def from_model_to_transformations(self):
        def kinematic_arm_chain(side):
            # for now create a fake model
            chain = []
            # create sympy variables
            t = []
            for i in range(self.nb_joints[0]):
                t.append(Symbol('t'+str(i)))
            # get the lengths of each segmen
            waist = self.lengths['waist_length']
            upper_arm = self.lengths['upper_arm_length']
            forearm = self.lengths['forearm_length']
            hand = self.lengths['hand_length']
            shoulder_width = self.lengths['shoulder_offset_width']
            shoulder_height = self.lengths['shoulder_offset_height']
            # create transformations chains
            spine0 = translation([0,0,waist])*rotation_x(t[0])
            spine1 = rotation_y(t[1])
            spine2 = rotation_z(t[2])
            if side == 'right':
                shoulder0 = translation([0,-shoulder_width,shoulder_height])*rotation_z(-pi/2)*rotation_y(t[3])
            else:
                shoulder0 = translation([0,shoulder_width,shoulder_height])*rotation_z(pi/2)*rotation_y(t[3])
            shoulder1 = rotation_z(t[4])
            shoulder2 = rotation_x(t[5])
            elbow0 = translation([upper_arm,0,0])*rotation_z(t[6])
            elbow1 = rotation_x(t[7])
            wrist0 = translation([forearm,0,0])*rotation_z(t[8])
            wrist1 = rotation_y(t[9])
            hand0 = translation([hand,0,0])
            # append all transformations
            chain.append(spine0)
            chain.append(spine1)
            chain.append(spine2)
            chain.append(shoulder0)
            chain.append(shoulder1)
            chain.append(shoulder2)
            chain.append(elbow0)
            chain.append(elbow1)
            chain.append(wrist0)
            chain.append(wrist1)
            chain.append(hand0)
            return chain
        def kinematic_leg_chain(side):
            # for now create a fake model
            chain = []
            # create sympy variables
            t = []
            for i in range(self.nb_joints[1]):
                t.append(Symbol('t'+str(i)))
            # get the lengths of each segmen
            thigh = self.lengths['thigh_length']
            shin = self.lengths['shin_length']
            foot = self.lengths['foot_length']
            hip_width = self.lengths['hip_offset_width']
            hip_height = self.lengths['hip_offset_height']
            # create transformations chains
            if side == 'right':
                hip0 = translation([0,-hip_width,hip_height])*rotation_y(pi/2)*rotation_z(t[0])
            else:
                hip0 = translation([0,hip_width,hip_height])*rotation_y(pi/2)*rotation_z(t[0])
            hip1 = rotation_y(t[1])
            hip2 = rotation_x(t[2])
            knee = translation([thigh,0,0])*rotation_x(-pi/2)*rotation_z(t[3])
            ankle0 = translation([shin,0,0])*rotation_x(-pi/2)*rotation_z(pi/2)*rotation_z(t[4])
            ankle1 = rotation_y(t[5])
            foot0 = translation([foot,0,0])
            # append all transformations
            chain.append(hip0)
            chain.append(hip1)
            chain.append(hip2)
            chain.append(knee)
            chain.append(ankle0)
            chain.append(ankle1)
            chain.append(foot0)
            return chain
        # create chains for both arms
        self.chains = []
        self.chains.append(kinematic_arm_chain('left'))
        self.chains.append(kinematic_arm_chain('right'))
        self.chains.append(kinematic_leg_chain('left'))
        self.chains.append(kinematic_leg_chain('right'))

    def forward_kinematic(self, joints):
        T = []
        for side in range(len(self.end_effectors)):
            T.append([])
            T[side].append(self.chains[side][0].subs('t'+str(0), joints[0]).evalf())
            for i in range(1,len(self.chains[side])-1):
                # evaluate the transformation matrix with the corresponding joint value
                temp = self.chains[side][i].subs('t'+str(i), joints[i]).evalf()
                # postmultiply the chain
                T[side].append(T[side][i-1]*temp)
            # multiply with the transformation to the end-effector
            T[side].append(T[side][-1]*self.chains[side][-1])
        return T
                
    def joint_limits(self):
        xml_urdf = rospy.get_param('human_description')
        dict_urdf = xmltodict.parse(xml_urdf)
        joints_urdf = []
        joints_urdf.append([j['@name'] for j in dict_urdf['robot']['joint'] if j['@name'] in self.joint_names])
        joints_urdf.append([[float(j['limit']['@lower']), float(j['limit']['@upper'])] for j in dict_urdf['robot']['joint'] if j['@name'] in self.joint_names])
        # reorder the joints limits
        limits = {}
        limits['joint_names'] = self.joint_names
        limits['limits'] = [joints_urdf[1][joints_urdf[0].index(name)] for name in self.joint_names]
        # reorder if limits are inversed
        for limit in limits['limits']:
            if limit[1] < limit[0]:
                temp = limit[0]
                limit[0] = limit[1]
                limit[1] = temp
        return limits

    def get_random_pose(self):
        # create a random joint state
        bounds = np.array(self.joint_limits()['limits'])
        joint_state = np.random.uniform(bounds[:,0], bounds[:,1], len(self.joint_names))
        return joint_state

    def calculate_leg_joints(self, knee_angle):
        k_val = -pi - knee_angle
        hip_angle = float(-self.leg_angles[0].subs('k', k_val))
        ankle_angle = float(-self.leg_angles[1].subs('k', k_val))
        return hip_angle, ankle_angle
