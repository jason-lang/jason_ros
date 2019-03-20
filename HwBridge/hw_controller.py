#!/usr/bin/env python2
import ConfigParser
import importlib
from pathlib import Path

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
        if reader.has_option(action_name, "com_method"):
            self.comm_method = reader.get(action_name, "com_method")
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
