#!/usr/bin/env python2
import ConfigParser
import importlib
import itertools
import rospy
import __builtin__
import std_msgs.msg
from pathlib import Path
from collections import OrderedDict

class CommInfo:
    def __init__(self):
        self.method = ""
        self.msg_type = ""
        self.name = ""
        self.dependencies = ""
        self.module = None
        self.data = []
        self.params_dict = OrderedDict()

    def fill_data(self, name, reader):
        if reader.has_option(name, "method"):
            self.method = reader.get(name, "method")
        if reader.has_option(name, "msg_type"):
            self.msg_type = reader.get(name, "msg_type")
        if reader.has_option(name, "name"):
            self.name = reader.get(name, "name")
        if reader.has_option(name, "dependencies"):
            self.dependencies = reader.get(name, "dependencies")
            self.module = importlib.import_module(self.dependencies)
        if reader.has_option(name, "data"):
            data_aux = reader.get(name, "data")
            self.data = [x.strip() for x in data_aux.split(',')]
        if reader.has_option(name, "params_name"):
            aux_name = reader.get(name, "params_name")
            if reader.has_option(name, "params_type"):
                aux_type = reader.get(name, "params_type")
                self.params_dict = OrderedDict((x.strip(),y.strip())  for x,y in itertools.izip(aux_name.split(','),aux_type.split(',')))
            else:
                self.params_dict = OrderedDict((x.strip(),"str")  for x in aux_name.split(','))

    def convert_params(self, params):
        converted = dict()
        for p,k in zip(params, self.params_dict):
            converted[k]= getattr(__builtin__, self.params_dict[k])(p)

        return converted

class CommController:
    def __init__(self):
        self.comm_dict = dict()
        self.default_path = ""

    def read_manifest(self, *args):
        reader = ConfigParser.RawConfigParser()
        if len(args) == 0:
            man_path = Path(self.default_path)
        else:
            man_path = Path(args[0])

        if man_path.is_file():
            reader.read(str(man_path))
            self.get_info(reader)


    def get_info(self, reader):
        comm_list = reader.sections()
        for comm in comm_list:
            comm_info = CommInfo()
            comm_info.fill_data(comm, reader)
            self.comm_dict[comm] = comm_info


class ActionController(CommController):
    def __init__(self):
        CommController.__init__(self)
        self.default_path = "actions_manifest"

    def executable_action(self, action_name):
        return (action_name in self.comm_dict.keys())

    def perform_action(self, action_name, params):
        action = self.comm_dict[action_name]
        converted_kw = action.convert_params(params)
        if action.method == "service":
            rospy.wait_for_service(action.name)
            try:
                service_aux = rospy.ServiceProxy(action.name, getattr(action.module, action.msg_type))
                service_aux(**converted_kw)
            except rospy.ServiceException as e:
                print("service "+ action.name +" call failed: %s." % e)
        elif action.method == "topic":
            topic_pub = rospy.Publisher(
                action.name,
                getattr(action.module, action.msg_type),
                queue_size=1,
                latch=True) #TODO:Verificar esse queue_size e latch se precisa ser esses

            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            converted_kw['header'] = header

            topic_pub.publish(**converted_kw)
        else:
            print("method" + action.method + " not available.")

    def action_completed(self, action_name):
        pass

class PerceptionController(CommController):
    def __init__(self):
        CommController.__init__(self)
        self.default_path = "perceptions_manifest"
        self.subsciber_dict = dict()
        self.perceptions = dict()

    def start_perceiving(self):
        for comm in self.comm_dict.items():
            name = comm[0]
            perception = comm[1]
            self.subsciber_dict[name] = rospy.Subscriber(
                perception.name,
                getattr(perception.module, perception.msg_type),
                self.subscriber_callback,
                name
            )

    def subscriber_callback(self, msg, name):
        perception_param = []
        for d in self.comm_dict[name].data:
            if hasattr(msg, d):
                perception_param.append(getattr(msg,d))
        perception_param = map(str, perception_param)
        self.perceptions[name] = name + '(' + ','.join(perception_param) + ')';
