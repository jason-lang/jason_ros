#!/usr/bin/env python2
import ConfigParser
import importlib
from pathlib import Path
import rospy

class ActionInfo:
    def __init__(self):
        self.name = ""
        self.comm_method = ""
        self.comm_msg = ""
        self.comm_name = ""
        self.dependencies = ""
        self.module = None

    def fill_data(self, action_name, reader):
        self.name = action_name
        if reader.has_option(action_name, "comm_method"):
            self.comm_method = reader.get(action_name, "comm_method")
        if reader.has_option(action_name, "comm_msg"):
            self.comm_msg = reader.get(action_name, "comm_msg")
        if reader.has_option(action_name, "comm_name"):
            self.comm_name = reader.get(action_name, "comm_name")
        if reader.has_option(action_name, "dependencies"):
            self.dependencies = reader.get(action_name, "dependencies")
            self.module = importlib.import_module(self.dependencies)


class ActionController:
    def __init__(self):
        self.action_dict = dict()

    def read_manifest(self, filename="actions_manifest"):
        reader = ConfigParser.RawConfigParser()
        man_path = Path(filename)
        if man_path.is_file():
            reader.read(str(man_path))
            self.get_info(reader)


    def get_info(self, reader):
        action_list = reader.sections()
        for action in action_list:
            action_info = ActionInfo()
            action_info.fill_data(action, reader)
            self.action_dict[action] = action_info

    def executable_action(self, action_name):
        return (action_name in self.action_dict.keys())

    def perform_action(self, action_name, **kw):
        action = self.action_dict[action_name]
        if action.comm_method == "service":
            rospy.wait_for_service(action.comm_name)
            try:
                service_aux = rospy.ServiceProxy(action.comm_name, getattr(action.module, action.comm_msg))
                service_aux(**kw)
            except rospy.ServiceException as e:
                print("service "+ action.comm_name +" call failed: %s." % e)
        elif action.comm_method == "topic":
            topic_pub = rospy.Publisher(
                action.comm_name,
                getattr(action.module, action.comm_msg),
                queue_size=1,
                latch=True) #TODO:Verificar esse queue_size e latch se precisa ser esses
            topic_pub.publish(**kw)
        else:
            print("Comm_method" + action.comm_method + " not available.")

    def action_completed(self, action_name):
        pass
