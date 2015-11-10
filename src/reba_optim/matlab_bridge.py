import rospy
import os.path
from reba_optim.srv import *
from trajectory_msgs.msg import JointTrajectory
import transformations

class MatlabBridge:
    def __init__(self, matlab_flag='matlab_flag.txt', ros_flag='ros_flag.txt', shared_folder='/tmp/matlab_bridge/'):
        # create share_folder if necessary
        if not os.path.exists(shared_folder):
            os.makedirs(shared_folder)
        # set the flag path
        self.shared_folder = shared_folder
        self.matlab_flag = shared_folder + matlab_flag
        self.ros_flag = shared_folder + ros_flag
        self.rate = rospy.Rate(10)

    def is_matlab_flag_set(self):
        return os.path.isfile(self.matlab_flag)

    def set_ros_flag(self):
        open(self.ros_flag, 'a').close()

    def unset_matlab_flag(self):
        os.remove(self.matlab_flag)

    def wait_for_matlab_flag(self):
        rospy.loginfo('Waiting for Matlab flag to be set.')
        # loop until the flag is set
        while not rospy.is_shutdown() and not self.is_matlab_flag_set():
            self.rate.sleep()
        # unset the flag
        self.unset_matlab_flag()

    def write_poses_in_file(self, handOverLocation, viaPointLocation=None):
        # TODO write file
        # set the ros flag
        self.set_ros_flag()

    def read_trajectory_file(self):
        trajectory = JointTrajectory()
        # wait for matlab flag
        self.wait_for_matlab_flag()
        rospy.loginfo('Reading trajectory.')
        # TODO read trajectory file
        return trajectory 

    def handle_trajectory_from_reba(self, req):
        rospy.loginfo('Received new trajectory query.')
        # get the requested pose
        handOverLocation = transformations.pose_to_list(req.hand_over_location)
        # query the bridge for the trajectory
        self.write_poses_in_file(handOverLocation)
        # wait for the response
        trajectory = self.read_trajectory_file()
        # return the calculated trajectory
        rospy.loginfo('Sending trajectory.')
        return TrajectoryFromRebaResponse(trajectory)

    def run(self):
        # intialize ros srv
        rospy.Service('trajectory_from_reba', TrajectoryFromReba, self.handle_trajectory_from_reba)
        rospy.loginfo('Ready to receive trajectory queries.')
        # wait for queries
        rospy.spin()


