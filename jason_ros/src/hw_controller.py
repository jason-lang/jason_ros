#!/usr/bin/env python2
import ConfigParser
import importlib
import itertools
import rospy
import __builtin__
import std_msgs.msg
import jason_ros_msgs.msg
from pathlib import Path
from collections import OrderedDict
from threading import Event
from threading import RLock

def str2bool(v):
  return v.lower() in ("yes", "true", "t", "1")

def setattr_recursive(obj_list, attr_list, value):
    aux = obj_list.pop()
    setattr(aux, attr_list.pop(), value)
    if len(obj_list) > 0:
        return setattr_recursive(obj_list, attr_list, aux)
    else:
        return aux

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
        self.latch = "True"

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
        if reader.has_option(name, "latch"):
            self.latch = reader.get(name, "latch")

    def convert_params(self, params):

        if self.method == "service":
            msg_type = self.msg_type + "Request"
        elif self.method == "topic":
            msg_type = self.msg_type
        else:
            return None

        param_class = getattr(self.module, msg_type)
        param_instance = param_class()
        converted = param_instance
        for p,k in zip(params, self.params_dict):
            param_attr = k.split('.')
            obj_list = [param_instance]
            for attr in param_attr:
                if hasattr(obj_list[-1], attr):
                    if attr is not param_attr[-1]:
                        obj_list.append(getattr(obj_list[-1], attr))
                    else:
                        value = getattr(__builtin__, self.params_dict[k])(p)
                        converted = setattr_recursive(obj_list, param_attr, value)

        return converted


class CommController:
    def __init__(self):
        self.comm_dict = dict()
        self.default_path = ""
        self.comm_len = 0

    def read_manifest(self, *args):
        reader = ConfigParser.RawConfigParser()

        if len(args) > 0 and Path(args[0]).is_file():
            man_path = Path(args[0])
        else:
            man_path = Path(self.default_path)

        if man_path.is_file():
            reader.read(str(man_path))
            self.get_info(reader)


    def get_info(self, reader):
        comm_list = reader.sections()
        self.comm_len = len(comm_list)
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
            converted_params = action.convert_params(params)
            msg_type = getattr(action.module, action.msg_type)
            node_namespace = rospy.get_namespace()
            if action.method == "service":
                rospy.wait_for_service(node_namespace + (action.name[1:] if action.name.startswith('/') else action.name))
                try:
                    service_aux = rospy.ServiceProxy(node_namespace + (action.name[1:] if action.name.startswith('/') else action.name), msg_type)
                    service_aux(converted_params)
                except rospy.ServiceException as e:
                    print("service "+ action.name +" call failed: %s." % e)
                action_completed = True
            elif action.method == "topic":
                latch = str2bool(action.latch)
                topic_pub = rospy.Publisher(
                    node_namespace + (action.name[1:] if action.name.startswith('/') else action.name),
                    msg_type,
                    queue_size=1,
                    latch=latch)

                if hasattr(converted_params, "header"):
                    header = std_msgs.msg.Header()
                    header.stamp = rospy.Time.now()
                    setattr(converted_params, "header", header)

                topic_pub.publish(converted_params)
                action_completed = True
            else:
                print("method " + action.method + " not available.")
                action_completed = False
        except KeyError:
            action_completed = False

        return action_completed


class PerceptionController(CommController):
    def __init__(self):
        CommController.__init__(self)
        self.default_path = "perceptions_manifest"
        self.subsciber_dict = dict()
        self.perceptions = dict()
        self.rate = None
        self.p_event = Event()
        self.p_lock = RLock()

    def get_info(self, reader):
        default_section = reader.defaults()
        try:
            self.rate = float(default_section["rate"])
        except KeyError:
            self.rate = 30
        CommController.get_info(self, reader)

    def start_perceiving(self):
        for comm in self.comm_dict.items():
            name = comm[0]
            perception = comm[1]
            node_namespace = rospy.get_namespace()
            self.subsciber_dict[name] = rospy.Subscriber(
                node_namespace + (perception.name[1:] if perception.name.startswith('/') else perception.name),
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

        perception = jason_ros_msgs.msg.Perception()
        perception.perception_name = name
        perception.parameters = perception_param
        if(self.comm_dict[name].buf == "add"):
            perception.update = False
        else:
            perception.update = True

        self.p_lock.acquire()
        self.perceptions[name] = perception
        self.p_event.set()
        self.p_lock.release()
