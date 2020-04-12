#!/usr/bin/env python2
from hw_controller import *
import rospy
import argparse
import std_msgs.msg
import jason_msgs.msg
import signal

def arg_parser():
    parser = argparse.ArgumentParser(description="HardwareBridge node")

    parser.add_argument("-a","--action", help="Action manifest path", nargs=1, type=str)
    parser.add_argument("-p","--perception", help="Perception manifest path", nargs=1, type=str)
    parser.add_argument("-r","--param", help="Rosparam .yaml", nargs=1, type=str)

    args, unknown = parser.parse_known_args()
    return vars(args)

def act(msg, args):
    action_controller = args[0]
    pub = args[1]

    action_status = jason_msgs.msg.ActionStatus()
    action_status.id = msg.header.seq

    action_status.result = action_controller.perform_action(msg.action_name, msg.parameters)

    pub.publish(action_status)


def main():
    print("Starting HwBridge node.")
    rospy.init_node('HwBridge')

    args = arg_parser()
    if args["param"] != None:
        import yaml
        import rosparam
        with open(args["param"][0], 'r') as stream:
            try:
                yaml_file = yaml.safe_load(stream)
                rosparam.upload_params("/", yaml_file)
            except yaml.YAMLError as exc:
                print(exc)

    action_controller = ActionController()
    if args["action"] != None:
        action_controller.read_manifest(args["action"][0])
    else:
        action_controller.read_manifest()

    node_namespace = rospy.get_namespace()
    jason_actions_status_pub = rospy.Publisher(
    node_namespace+ 'jason/actions_status',
    jason_msgs.msg.ActionStatus,
    queue_size=1,
    latch=False)

    jason_action_sub = rospy.Subscriber(
        node_namespace + 'jason/actions',
        jason_msgs.msg.Action,
        act, (action_controller,jason_actions_status_pub))


    perception_controller = PerceptionController()
    if args["perception"] != None:
        perception_controller.read_manifest(args["perception"][0])
    else:
        perception_controller.read_manifest()

    perception_controller.start_perceiving()

    jason_percepts_pub = rospy.Publisher(
    node_namespace + 'jason/percepts',
    jason_msgs.msg.Perception,
    queue_size=(2*perception_controller.comm_len),
    latch=False)

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    rate = rospy.Rate(perception_controller.rate)
    while not rospy.is_shutdown():
        perception_controller.p_lock.acquire()
        for p in perception_controller.perceptions.items():
            if p[1] != None:
                jason_percepts_pub.publish(p[1])
                perception_controller.perceptions[p[0]] = None
        perception_controller.p_lock.release()
        perception_controller.p_event.wait()
        perception_controller.p_event.clear()
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
