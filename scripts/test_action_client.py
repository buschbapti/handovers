#! /usr/bin/env python

import rospy
import actionlib

from reba_optim.msg import RebaHandOverGoal, RebaHandOverAction

if __name__ == '__main__':
    rospy.init_node('reba_comfort_pose_action_server')
    client = actionlib.SimpleActionClient('/reba/comfort_pose_action_server', RebaHandOverAction)
    client.wait_for_server()

    goal = RebaHandOverGoal()
    goal.object = '/toolbox/side_right'
    client.send_goal(goal)

    #client_l.wait_for_result(rospy.Duration.from_sec(100.0))
    client.wait_for_result(rospy.Duration.from_sec(20.0))
    #print client_l.get_result()
    print client.get_result()
