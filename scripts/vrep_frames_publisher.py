#!/usr/bin/env python

import rospy
import yaml
import tf
import transformations
import rospkg

class VREPFramesPublisher():
    def __init__(self, rate, side):
        self.rospack = rospkg.RosPack()
        self.tfb = tf.TransformBroadcaster()
        self.rate = rospy.Rate(rate)
        self.world = '/base'
        self.side = side

        with open(self.rospack.get_path("reba_optim")+"/config/calibration_matrix_vrep.yaml") as f:
            self.calibration_matrix_vrep = yaml.load(f)

        with open(self.rospack.get_path("reba_optim")+"/config/calibration_matrix_tip.yaml") as f:
            self.calibration_matrix_tip = yaml.load(f)

    def run(self):
        while not rospy.is_shutdown():
            self.tfb.sendTransform(self.calibration_matrix_tip[0], self.calibration_matrix_tip[1], rospy.Time.now(), '/vrep_tip', self.side)
            self.tfb.sendTransform(self.calibration_matrix_vrep[0], self.calibration_matrix_vrep[1], rospy.Time.now(), '/vrep_frame', self.world)
            self.rate.sleep()


if __name__=='__main__':
    rospy.init_node('vrep_frames_publisher')
    VREPFramesPublisher(20, 'right_gripper').run()