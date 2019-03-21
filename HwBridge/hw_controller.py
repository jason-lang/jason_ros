#!/usr/bin/env python2
import ConfigParser
import importlib
from pathlib import Path
import rospy

class CommInfo:
    def __init__(self):
        self.name = ""
        self.comm_method = ""
        self.comm_msg = ""
        self.comm_name = ""
        self.dependencies = ""
        self.module = None

    def fill_data(self, comm_name, reader):
        self.name = comm_name
        if reader.has_option(comm_name, "comm_method"):
            self.comm_method = reader.get(comm_name, "comm_method")
        if reader.has_option(comm_name, "comm_msg"):
            self.comm_msg = reader.get(comm_name, "comm_msg")
        if reader.has_option(comm_name, "comm_name"):
            self.comm_name = reader.get(comm_name, "comm_name")
        if reader.has_option(comm_name, "dependencies"):
            self.dependencies = reader.get(comm_name, "dependencies")
            self.module = importlib.import_module(self.dependencies)

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

    def perform_action(self, action_name, **kw):
        action = self.comm_dict[action_name]
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
