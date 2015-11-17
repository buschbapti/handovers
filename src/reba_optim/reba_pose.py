import rospy
from reba_optim.srv import RebaPose, RebaPoseResponse

class RebaPose:
    def __init__(self):

    def handle_reba_pose(self, req):
        # get hip pose

    def run(self):
        # intialize ros srv
        rospy.Service('reba_pose', RebaPose, self.handle_reba_pose)
        rospy.loginfo('Ready to receive pose queries.')
        # wait for queries
        rospy.spin()