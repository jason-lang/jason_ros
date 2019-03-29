#!/usr/bin/env python2
from hw_controller import *
import rospy
import std_msgs.msg
import re
import jason_msgs.msg

def decompose(data):
    predicate = re.match('[^\(]*', data).group(0)
    args_dict = dict()
    try:
        arguments = re.findall('\((.*?)\)', data)[0].split(',')
        for args in arguments:
            args_ = args.split('=')
            args_dict[args_[0]] = args_[1]
    except IndexError:
        pass

    return predicate, args_dict

def act(msg, pub):
    action_controller = ActionController()
    action_controller.read_manifest()

    action_name, args = decompose(msg.data)
    action_controller.perform_action(action_name, **args)
    pub.publish("done("+action_name+")")


def main():
    print("Starting HwBridge node.")
    rospy.init_node('HwBridge')
    rate = rospy.Rate(1)

    jason_percepts_pub = rospy.Publisher(
    '/jason/percepts',
    std_msgs.msg.String,
    queue_size=1,
    latch=False)


    perception_controller = PerceptionController()
    perception_controller.read_manifest()
    perception_controller.start_perceiving()

    jason_actions_status_pub = rospy.Publisher(
    '/jason/actions_status',
    std_msgs.msg.String,
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
