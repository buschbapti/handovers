#! /usr/bin/env python

import rospy
import actionlib

from reba_optim.msg import RebaHandOverActionGoal, RebaHandOverAction

if __name__ == '__main__':
    rospy.init_node('/reba/comfort_pose_action_server')
    client = actionlib.SimpleActionClient('/thr/run_mdp_action', RebaHandOverAction)
    client.wait_for_server()

    goal = RebaHandOverActionGoal()
    goal.goal.object = '/human/wrist'
    client.send_goal(goal)

    #client_l.wait_for_result(rospy.Duration.from_sec(100.0))
    client.wait_for_result(rospy.Duration.from_sec(20.0))
    #print client_l.get_result()
    print client.get_result()
