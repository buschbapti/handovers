from sympy import *
import rospy
from kinect_skeleton_publisher.joint_transformations import *
from sympy import *
import xmltodict

class ReadModel:
    def __init__(self, model=None):
        # get the lengths of the human
        self.nb_joints = 10
        self.lengths = rospy.get_param('/kinect/human_lengths')
        self.init_joint_names()
        self.from_model_to_transformations()

    def init_joint_names(self):
        # create the list of all joints byt appending first spherical joints
        self.joint_names = []
        for i in range(3):
            self.joint_names.append('spine_'+str(i))
            self.joint_names.append('neck_'+str(i))
            self.joint_names.append('right_shoulder_'+str(i))
            self.joint_names.append('left_shoulder_'+str(i))
        # then append universal joints
        for i in range(2):
            self.joint_names.append('right_elbow_'+str(i))
            self.joint_names.append('left_elbow_'+str(i))
            self.joint_names.append('right_wrist_'+str(i))
            self.joint_names.append('left_wrist_'+str(i))
        # finish by the knee joint
        self.joint_names.append('right_knee')
        self.joint_names.append('left_knee')

    def from_model_to_transformations(self):
        def kinematic_chain(side):
            # for now create a fake model
            chain = []
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
            hand = translation([hand,0,0])
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
            chain.append(hand)
            return chain
        # create chains for both arms
        self.chains = []
        self.chains.append(kinematic_chain('right'))
        self.chains.append(kinematic_chain('left'))

    def forward_kinematic(self, joints, side=0):
        T = eye(4)
        for i in range(len(self.chains[side])-1):
            # evaluate the transformation matrix with the corresponding joint value
            temp = self.chains[side][i].subs('t'+str(i), joints[i]).evalf()
            # postmultiply the chain
            T = T*temp
        # multiply with the transformation to the end-effector
        T = T*self.chains[side][-1]
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

