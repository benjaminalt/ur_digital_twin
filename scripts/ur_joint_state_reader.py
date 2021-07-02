#!/usr/bin/env python

from rtde import rtde
import rtde.rtde_config as rtde_config


class URJointStateReader(object):
    def __init__(self, config_file, host, port, sampling_interval=0.008):
        self.config_file = config_file
        self.host = host
        self.port = port
        self.sampling_interval = sampling_interval
        self.connection = None

    def __enter__(self):
        conf = rtde_config.ConfigFile(self.config_file)
        output_names, output_types = conf.get_recipe('out')
        self.connection = rtde.RTDE(self.host, self.port)
        self.connection.connect()
        if not self.connection.send_output_setup(output_names, output_types, frequency = 1/self.sampling_interval):
            raise RuntimeError('Unable to configure output')
        if not self.connection.send_start():
            raise RuntimeError('Unable to start synchronization')
        return self.connection

    def __exit__(self, *args):
        self.connection.send_pause()
        self.connection.disconnect()
