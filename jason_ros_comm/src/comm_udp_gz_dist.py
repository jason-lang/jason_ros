#!/usr/bin/env python2
from socket import *
import rospy
import jason_ros_msgs.msg
import gazebo_msgs.srv


class GzModelDistances:
    def __init__(self, my_name):
        self.my_name = my_name

        rospy.wait_for_service('/gazebo/get_model_state')
        self.service_proxy = rospy.ServiceProxy('/gazebo/get_model_state',
                                                gazebo_msgs.srv.GetModelState)
        try:
            node_namespace = rospy.get_namespace()
            self.comm_range = rospy.get_param(node_namespace + '/comm_range')
        except KeyError:
            self.comm_range = 10

    def inCommRange(self, model_name):
        self.myStateRes = self.service_proxy(self.my_name, '')
        modelStateRes = self.service_proxy(model_name, '')

        dist = ((self.myStateRes.pose.position.x
                 - modelStateRes.pose.position.x)**2
                + (self.myStateRes.pose.position.y
                    - modelStateRes.pose.position.y)**2)**0.5
        print("my_name: {} model_name: {} dist: {} comm_range: {}".format(
                self.my_name, model_name, dist, self.comm_range))

        return (dist <= self.comm_range)


def send_msg(msg, agents_ip):
    data = msg.data
    receiver = data.split(',')[3]
    s = socket(AF_INET, SOCK_DGRAM)

    node_namespace = rospy.get_namespace()
    agent_name = rospy.get_param(node_namespace + 'jason/agent_name')

    modelDists = GzModelDistances(agent_name)

    agent_sent = [agent_name]
    if receiver == "null":
        for addr in agents_ip.iteritems():
            if addr[0] != "null" and addr[0] not in agent_sent \
             and modelDists.inCommRange(addr[0]):
                s.sendto(data, (addr[1][0], addr[1][1]))
                agent_sent.append(addr[0])
    elif modelDists.inCommRange(receiver):
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
    agents_ip = {name[len(node_name + '/address_'):]: rospy.get_param(name)
                 for name in param_names
                 if name.startswith(node_name + '/address_')}

    send_msg_sub = rospy.Subscriber(
        node_namespace + 'jason/send_msg',
        jason_ros_msgs.msg.Message,
        send_msg, agents_ip)

    comm_message_pub = rospy.Publisher(
        node_namespace + 'jason/receive_msg',
        jason_ros_msgs.msg.Message,
        queue_size=1,
        latch=False)

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

    rospy.spin()


if __name__ == '__main__':
    main()
