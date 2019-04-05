#!/usr/bin/env python2
from hw_controller import *
import rospy
import re
import std_msgs.msg
import jason_msgs.msg

def act(msg, pub):
    action_controller = ActionController()
    action_controller.read_manifest()

    action_status = jason_msgs.msg.ActionStatus()
    action_status.id = msg.header.seq

    action_status.result = action_controller.perform_action(msg.action_name, msg.parameters)

    pub.publish(action_status)


def main():
    print("Starting HwBridge node.")
    rospy.init_node('HwBridge')
    rate = rospy.Rate(1)

    jason_percepts_pub = rospy.Publisher(
    '/jason/percepts',
    jason_msgs.msg.Perception,
    queue_size=1,
    latch=False)


    perception_controller = PerceptionController()
    perception_controller.read_manifest()
    perception_controller.start_perceiving()

    jason_actions_status_pub = rospy.Publisher(
    '/jason/actions_status',
    jason_msgs.msg.ActionStatus,
    queue_size=1,
    latch=False)

    jason_action_sub = rospy.Subscriber(
        '/jason/actions',
        jason_msgs.msg.Action,
        act, jason_actions_status_pub)


    while not rospy.is_shutdown():
        print(perception_controller.perceptions.values())
        for p in perception_controller.perceptions.values():
            jason_percepts_pub.publish(p)

        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
