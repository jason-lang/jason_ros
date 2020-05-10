#!/usr/bin/env python2
import ConfigParser
import importlib
import itertools
import rospy
import __builtin__
import std_msgs.msg
import jason_ros_msgs.msg
import re
import ast
from pathlib import Path
from collections import OrderedDict
from threading import Event
from threading import RLock

def str2bool(v):
  return v.lower() in ("yes", "true", "t", "1")


def get_obj_list(msg_instance, param_name):
    param_attrs = param_name.split('.')
    obj_list = [msg_instance]
    for attr in param_attrs:
        if hasattr(obj_list[-1], attr) and attr is not param_attrs[-1]:
            obj_list.append(getattr(obj_list[-1], attr))
    return param_attrs, obj_list


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
        self.args = []
        self.params_dict = OrderedDict()
        self.buf = ""
        self.latch = "True"

    def fill_data(self, name, reader):
        if reader.has_option(name, "method"):
            self.method = reader.get(name, "method")
        if reader.has_option(name, "msg_type"):
            try:
                msg_type_vector = reader.get(name, "msg_type").split('/');
                if self.method == "service":
                    self.msg_type = (msg_type_vector[1], importlib.import_module(msg_type_vector[0]+".srv"))
                else:
                    self.msg_type = (msg_type_vector[1], importlib.import_module(msg_type_vector[0]+".msg"))
            except IndexError:
                if reader.has_option(name, "dependencies"):
                    self.msg_type = (msg_type_vector[0],importlib.import_module(reader.get(name, "dependencies")))
                else:
                    print("Did you specify the whole path of msg_type, or defined dependencies? e.g: geometry_msgs/Point32")
                    raise
            except ImportError:
                print("Wrong path of msg_type")
                raise
        if reader.has_option(name, "name"):
            self.name = reader.get(name, "name")
        if reader.has_option(name, "args"):
            args_aux = reader.get(name, "args")
            self.args = [y.split('.') for y in [x.strip() for x in args_aux.split(',')]]
        if reader.has_option(name, "params_name"):
            aux_name = reader.get(name, "params_name")
            if reader.has_option(name, "params_type"):
                aux_type = reader.get(name, "params_type")
                self.params_dict = OrderedDict((x.strip(),y.strip())  for x,y in itertools.izip(aux_name.split(','),re.split(r',\s*(?![^())]*\))',aux_type))) # #split ,if they are not between []
            else:
                self.params_dict = OrderedDict((x.strip(),"str")  for x in aux_name.split(','))
        if reader.has_option(name, "buf"):
            self.buf = reader.get(name, "buf")
        if reader.has_option(name, "latch"):
            self.latch = reader.get(name, "latch")

    def convert_params(self, params):

        if self.method == "service":
            msg_type = self.msg_type[0] + "Request"
        elif self.method == "topic":
            msg_type = self.msg_type[0]
        else:
            return None

        msg_class = getattr(self.msg_type[1], msg_type)
        msg_instance = msg_class()
        converted = msg_instance
        for param, param_name in zip(params, self.params_dict):
            param_attrs, obj_list = get_obj_list(msg_instance, param_name)
            value = None
            if re.search(r'\[(.*?)\]', self.params_dict[param_name]) and re.search(r'\((.*?)\)',self.params_dict[param_name]):
                param_type_split = self.params_dict[param_name].split('/')
                param_type_module = importlib.import_module(param_type_split[0] + '.msg')
                param_type_class = getattr(param_type_module, param_type_split[1].split('[')[0])
                value = []
                for p in ast.literal_eval(param):
                    param_type_instance = param_type_class()
                    param_type_attrs = re.search(r'\((.*?)\)',param_type_split[1]).group(0).strip('()').split(',')
                    for attr,p_aux in zip(param_type_attrs,p):
                        attr_list, param_type_obj_list = get_obj_list(param_type_instance, attr)
                        param_type_instance = setattr_recursive(param_type_obj_list, attr_list, p_aux)
                    value.append(param_type_instance)
            elif re.search(r'\[(.*?)\]', self.params_dict[param_name]):
                value = ast.literal_eval(param)
            else:
                value = getattr(__builtin__, self.params_dict[param_name])(param)

            converted = setattr_recursive(obj_list, param_attrs, value)
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
            msg_type = getattr(action.msg_type[1], action.msg_type[0])
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
                getattr(perception.msg_type[1], perception.msg_type[0]),
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
