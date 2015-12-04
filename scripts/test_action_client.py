#! /usr/bin/env python

import rospy
import actionlib
import os
import sys

from reba_optim.msg import RebaHandOverGoal, RebaHandOverAction

if __name__ == '__main__':
    rospy.init_node('reba_comfort_pose_action_server')
    client = actionlib.SimpleActionClient('/reba/comfort_pose_action_server', RebaHandOverAction)
    client.wait_for_server()

    # wait for user being ready
    rospy.sleep(10)
    os.system('beep')

    goal = RebaHandOverGoal()
    goal.object = '/toolbox/side_right'
    goal.prefered_hand_to_handover = int(sys.argv[1])
    client.send_goal(goal)

    #client_l.wait_for_result(rospy.Duration.from_sec(100.0))
    client.wait_for_result(rospy.Duration.from_sec(20.0))
    #print client_l.get_result()
    print client.get_result()
