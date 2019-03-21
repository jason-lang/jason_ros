#!/usr/bin/env python2
from hw_controller import *

def main():
    print("Starting HwBridge node.")
    rospy.init_node('HwBridge')
    rate = rospy.Rate(1)

    a = ActionController()
    a.read_manifest()
    if a.executable_action('teste'):
        a.perform_action('teste', data='hello topics')

    if a.executable_action('set_mode'):
        a.perform_action('set_mode', custom_mode='GUIDED')

    p = PerceptionController()
    p.read_manifest()
    p.start_perceiving()
    while not rospy.is_shutdown():
        print(p.perceptions.values())
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
