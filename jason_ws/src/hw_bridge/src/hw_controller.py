#!/usr/bin/env python2
import ConfigParser
import importlib
import itertools
import rospy
import __builtin__
import std_msgs.msg
import jason_msgs.msg
from pathlib import Path
from collections import OrderedDict

class CommInfo:
    def __init__(self):
        self.method = ""
        self.msg_type = ""
        self.name = ""
        self.dependencies = ""
        self.module = None
        self.args = []
        self.params_dict = OrderedDict()
        self.buf = ""

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
        if reader.has_option(name, "args"):
            args_aux = reader.get(name, "args")
            self.args = [y.split('.') for y in [x.strip() for x in args_aux.split(',')]]
        if reader.has_option(name, "params_name"):
            aux_name = reader.get(name, "params_name")
            if reader.has_option(name, "params_type"):
                aux_type = reader.get(name, "params_type")
                self.params_dict = OrderedDict((x.strip(),y.strip())  for x,y in itertools.izip(aux_name.split(','),aux_type.split(',')))
            else:
                self.params_dict = OrderedDict((x.strip(),"str")  for x in aux_name.split(','))
        if reader.has_option(name, "buf"):
            self.buf = reader.get(name, "buf")

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
        action_completed = False
        try:
            action = self.comm_dict[action_name]
            converted_kw = action.convert_params(params)
            msg_type = getattr(action.module, action.msg_type)
            if action.method == "service":
                rospy.wait_for_service(action.name)
                try:
                    service_aux = rospy.ServiceProxy(action.name, msg_type)
                    service_aux(**converted_kw)
                except rospy.ServiceException as e:
                    print("service "+ action.name +" call failed: %s." % e)
                action_completed = True
            elif action.method == "topic":
                topic_pub = rospy.Publisher(
                    action.name,
                    msg_type,
                    queue_size=1,
                    latch=True) #TODO:Verificar esse queue_size e latch se precisa ser esses


                if hasattr(msg_type, "header"):
                    header = std_msgs.msg.Header()
                    header.stamp = rospy.Time.now()
                    converted_kw['header'] = header

                topic_pub.publish(**converted_kw)
                action_completed = True
            else:
                print("method " + action.method + " not available.")
                action_completed = True
        except KeyError:
            action_completed = True

        return action_completed

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
        for data in self.comm_dict[name].args:
            obj = msg
            for d in data:
                if hasattr(obj, d):
                    obj = getattr(obj, d)
                    if d is data[-1]:
                        perception_param.append(obj)
        perception_param = map(str, perception_param)

        perception = jason_msgs.msg.Perception()
        perception.perception_name = name
        perception.parameters = perception_param
        if(self.comm_dict[name].buf == "add"):
            perception.update = False
        else:
            perception.update = True

        self.perceptions[name] = perception
