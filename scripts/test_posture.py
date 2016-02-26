#!/usr/bin/env python
from human_moveit_config.human_model import HumanModel
from sensor_msgs.msg import JointState
import rospy


def main():
    reba_publisher = rospy.Publisher('reba_assess', JointState, queue_size=1)
    human = HumanModel()
    rospy.sleep(0.1)

    r_arm_joints = [1.57, 0.0, 0.0, 1.0472, 0.0, 0.0, 0.0]
    l_arm_joints = [1.57, 0.0, 0.0, -1.0472, 0.0, 0.0, 0.0]
    head_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    human.send_joint_values('right_arm', r_arm_joints)
    rospy.sleep(0.1)
    human.send_joint_values('left_arm', l_arm_joints)
    rospy.sleep(0.1)
    human.send_joint_values('head', head_joints)

    # get current state
    js = human.get_current_state().joint_state
    # publish it to reba
    rate = rospy.Rate(10)
    for i in range(100):
        reba_publisher.publish(js)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('test_posture')
    main()
