#!/usr/bin/env python2
from socket import *
import rospy
import jason_ros_msgs.msg

def send_msg(msg, agents_ip):
    data = msg.data
    receiver = data.split(',')[3]
    s = socket(AF_INET, SOCK_DGRAM)

    node_namespace = rospy.get_namespace()
    agent_name = rospy.get_param(node_namespace + 'jason/agent_name')

    agent_sent = []
    if receiver == "null":
        for addr in agents_ip.iteritems():
            if addr[0]!= "null" and addr[1][0] not in agent_sent:
                print(agent_name)
                s.sendto(data, (addr[1][0], addr[1][1]))
                agent_sent.append(addr[0])
    else:
        IP = agents_ip[receiver][0]
        PORT = agents_ip[receiver][1]
        s.sendto(data, (IP, PORT))

    s.close()
    # print(agent_name + " Sending: " + data)

def main():
    print("Starting Communication node.")
    rospy.init_node('jason_comm')
    node_namespace = rospy.get_namespace()
    node_name = rospy.get_name()

    param_names = rospy.get_param_names()
    agents_ip = {name[len(node_name + '/address_'):]:rospy.get_param(name) for name in param_names if name.startswith(node_name + '/address_')}

    send_msg_sub = rospy.Subscriber(
        node_namespace + 'jason/send_msg',
        jason_ros_msgs.msg.Message,
        send_msg, agents_ip)

    comm_message_pub = rospy.Publisher(
        node_namespace + 'jason/receive_msg',
        jason_ros_msgs.msg.Message,
        queue_size=1,
        latch=False)

    # rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if rospy.has_param(node_namespace + 'jason/agent_name'):
            agent_name = rospy.get_param(node_namespace + 'jason/agent_name')
            IP = agents_ip[agent_name][0]
            PORT = agents_ip[agent_name][1]
            try:
                s = socket(AF_INET, SOCK_DGRAM)
                s.bind((IP, PORT))
                m = s.recvfrom(1024)
                s.close()
                if m[1][0]:
                    message = jason_ros_msgs.msg.Message()
                    message.data = m[0]
                    # print(agent_name + " Received " + message.data)
                    comm_message_pub.publish(message)
            except timeout:
                s.close()

        # rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
